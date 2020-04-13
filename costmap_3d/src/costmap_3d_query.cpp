/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Badger Technologies LLC
 *  All rights reserved.
  *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: C. Andy Martin
 *********************************************************************/
#include <costmap_3d/costmap_3d_query.h>
#include <chrono>
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance_result.h>
#include <fcl/geometry/shape/sphere.h>
#include <pcl/io/vtk_lib_io.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>

namespace costmap_3d
{

// Define and initialize static thread_local variables.
// statics.
thread_local unsigned int Costmap3DQuery::tls_last_layered_costmap_update_number_ = 0;
thread_local Costmap3DQuery* Costmap3DQuery::tls_last_instance_ = nullptr;
thread_local Costmap3DQuery::DistanceCacheEntry* Costmap3DQuery::tls_last_cache_entries_[Costmap3DQuery::MAX] = {nullptr};

Costmap3DQuery::Costmap3DQuery(
    const LayeredCostmap3D* layered_costmap_3d,
    const std::string& mesh_resource,
    double padding,
    unsigned int pose_bins_per_meter,
    unsigned int pose_bins_per_radian,
    unsigned int pose_milli_bins_per_meter,
    unsigned int pose_milli_bins_per_radian,
    unsigned int pose_micro_bins_per_meter,
    unsigned int pose_micro_bins_per_radian)
  : layered_costmap_3d_(layered_costmap_3d),
    pose_bins_per_meter_(pose_bins_per_meter),
    pose_bins_per_radian_(pose_bins_per_radian),
    pose_milli_bins_per_meter_(pose_milli_bins_per_meter),
    pose_milli_bins_per_radian_(pose_milli_bins_per_radian),
    pose_micro_bins_per_meter_(pose_micro_bins_per_meter),
    pose_micro_bins_per_radian_(pose_micro_bins_per_radian)
{
  init();
  updateMeshResource(mesh_resource, padding);
}

Costmap3DQuery::Costmap3DQuery(const Costmap3DConstPtr& costmap_3d,
    const std::string& mesh_resource,
    double padding,
    unsigned int pose_bins_per_meter,
    unsigned int pose_bins_per_radian,
    unsigned int pose_milli_bins_per_meter,
    unsigned int pose_milli_bins_per_radian,
    unsigned int pose_micro_bins_per_meter,
    unsigned int pose_micro_bins_per_radian)
  : layered_costmap_3d_(nullptr),
    pose_bins_per_meter_(pose_bins_per_meter),
    pose_bins_per_radian_(pose_bins_per_radian),
    pose_milli_bins_per_meter_(pose_milli_bins_per_meter),
    pose_milli_bins_per_radian_(pose_milli_bins_per_radian),
    pose_micro_bins_per_meter_(pose_micro_bins_per_meter),
    pose_micro_bins_per_radian_(pose_micro_bins_per_radian)
{
  init();
  // Make a local copy of the costmap in question
  // It would be awesome if the Costmap3D had a way to snapshot
  // or copy-on-write. As it stands, for many scenarios involving
  // space-limited local costmaps, copying a 3D costmap will only take a
  // couple millseconds and is better than leaving the costmap locked for
  // an entire planning cycle.
  octree_ptr_.reset(new Costmap3D(*costmap_3d));
  updateMeshResource(mesh_resource, padding);
}

Costmap3DQuery::~Costmap3DQuery()
{
}

void Costmap3DQuery::init()
{
  clearStatistics();
  milli_cache_threshold_ = 2.5 / pose_milli_bins_per_meter_;
  micro_cache_threshold_ = 2.5 / pose_micro_bins_per_meter_;
}

// Caller must already hold an upgradable shared lock via the passed
// upgrade_lock, which we can use to upgrade to exclusive access if necessary.
void Costmap3DQuery::checkCostmap(Costmap3DQuery::upgrade_lock& upgrade_lock)
{
  // First check if we need to update w/ just the read lock held
  bool need_update = false;
  {
    if (layered_costmap_3d_)
    {
      if (layered_costmap_3d_->getCostmap3D() != octree_ptr_ ||
         (last_layered_costmap_update_number_ != layered_costmap_3d_->getNumberOfUpdates()))
      {
        need_update = true;
      }
    }
  }
  if (need_update)
  {
    // get write access
    upgrade_to_unique_lock write_lock(upgrade_lock);

    if (layered_costmap_3d_->getCostmap3D() != octree_ptr_)
    {
      // The octomap has been reallocated, change where we are pointing.
      octree_ptr_ = layered_costmap_3d_->getCostmap3D();
    }
    // The costmap has been updated since the last query, reset our caches

    // For simplicity, on every update, clear out the collision cache.
    // This is not strictly necessary in the case where the costmap has been
    // updated. The mesh is stored in the cache and does not change. The only
    // thing that changed is the costmap. We could go through the delta map
    // and only remove entries that have had the corresponding octomap cell go
    // away. This may be implemented as a future improvement. It would be OK
    // to keep entries in the distance_cache_ as long as their respective
    // octomap boxes did not get removed, as the distance_cache not having the
    // best answer in the presence of a new closer octomap cell does not make
    // the result incorrect (just less efficient). However, such an improvement
    // may not yield any practical increase in performance due to the
    // tls_last_cache_entries_ and the fact that most queries query poses on a path,
    // which makes a distance_cache miss much less costly.
    distance_cache_.clear();
    // We must always drop the milli cache and micro cache, as new cells will
    // invalidate the old results, and there is no simple way to figure out
    // which ones might still be valid.
    milli_distance_cache_.clear();
    micro_distance_cache_.clear();
    exact_distance_cache_.clear();
    printStatistics();
    clearStatistics();
    last_layered_costmap_update_number_ = layered_costmap_3d_->getNumberOfUpdates();
  }
  // Check if any thread local state must be invalidated.
  // We must handle if either the costmap has been updated since this thread's
  // storage was invalidated, or if a different instance of the object is
  // being used in the same thread.
  if (tls_last_layered_costmap_update_number_ != last_layered_costmap_update_number_ ||
      tls_last_instance_ != this)
  {
    for (unsigned int i=0; i<MAX; ++i)
    {
      tls_last_cache_entries_[i] = nullptr;
    }
    tls_last_layered_costmap_update_number_ = last_layered_costmap_update_number_;
    tls_last_instance_ = this;
  }
}

void Costmap3DQuery::clearStatistics()
{
  queries_since_clear_ = 0;
  hits_since_clear_ = 0;
  fast_milli_hits_since_clear_ = 0;
  slow_milli_hits_since_clear_ = 0;
  fast_micro_hits_since_clear_ = 0;
  slow_micro_hits_since_clear_ = 0;
  exact_hits_since_clear_ = 0;
  reuse_results_since_clear_ = 0;
  misses_since_clear_us_ = 0;
  hits_since_clear_us_ = 0;
  fast_milli_hits_since_clear_us_ = 0;
  slow_milli_hits_since_clear_us_ = 0;
  fast_micro_hits_since_clear_us_ = 0;
  slow_micro_hits_since_clear_us_ = 0;
  exact_hits_since_clear_us_ = 0;
  reuse_results_since_clear_us_ = 0;
  hit_fcl_bv_distance_calculations_ = 0;
  hit_fcl_primative_distance_calculations_ = 0;
  miss_fcl_bv_distance_calculations_ = 0;
  miss_fcl_primative_distance_calculations_ = 0;
}

void Costmap3DQuery::printStatistics()
{
  unsigned int cache_misses = queries_since_clear_ -
      hits_since_clear_ -
      fast_milli_hits_since_clear_ -
      slow_milli_hits_since_clear_ -
      fast_micro_hits_since_clear_ -
      slow_micro_hits_since_clear_ -
      reuse_results_since_clear_ -
      exact_hits_since_clear_;
  double hit_ratio = (double)hits_since_clear_ / queries_since_clear_;
  double fast_milli_hit_ratio = (double)fast_milli_hits_since_clear_ / queries_since_clear_;
  double slow_milli_hit_ratio = (double)slow_milli_hits_since_clear_ / queries_since_clear_;
  double fast_micro_hit_ratio = (double)fast_micro_hits_since_clear_ / queries_since_clear_;
  double slow_micro_hit_ratio = (double)slow_micro_hits_since_clear_ / queries_since_clear_;
  double exact_hit_ratio = (double)exact_hits_since_clear_ / queries_since_clear_;
  uint64_t total_us = misses_since_clear_us_ +
      hits_since_clear_us_ +
      fast_milli_hits_since_clear_us_ +
      slow_milli_hits_since_clear_us_ +
      fast_micro_hits_since_clear_us_ +
      slow_micro_hits_since_clear_us_ +
      reuse_results_since_clear_us_ +
      exact_hits_since_clear_us_;
  ROS_DEBUG_STREAM_NAMED(
      "query_statistics",
      "Costmap3DQuery statistics:"
      "\n\tqueries this cycle: " << queries_since_clear_ <<
      "\n\tcache misses: " << cache_misses <<
      "\n\tcache hits: " << hits_since_clear_ <<
      "\n\tcache hit ratio: " << hit_ratio <<
      "\n\tslow milli cache hits: " << slow_milli_hits_since_clear_ <<
      "\n\tslow milli cache hit ratio: " << slow_milli_hit_ratio <<
      "\n\tfast milli cache hits: " << fast_milli_hits_since_clear_ <<
      "\n\tfast milli cache hit ratio: " << fast_milli_hit_ratio <<
      "\n\tslow micro cache hits: " << slow_micro_hits_since_clear_ <<
      "\n\tslow micro cache hit ratio: " << slow_micro_hit_ratio <<
      "\n\tfast micro cache hits: " << fast_micro_hits_since_clear_ <<
      "\n\tfast micro cache hit ratio: " << fast_micro_hit_ratio <<
      "\n\texact cache hits: " << exact_hits_since_clear_ <<
      "\n\texact cache hit ratio: " << exact_hit_ratio <<
      "\n\treuse past results: " << reuse_results_since_clear_ <<
      "\n\ttotal usecs: " << total_us <<
      "\n\tmiss usecs/query: " << (double)misses_since_clear_us_ / cache_misses <<
      "\n\thit usecs/query: " << (double)hits_since_clear_us_ / hits_since_clear_ <<
      "\n\tslow milli hit usecs/query: " << (double)slow_milli_hits_since_clear_us_ / slow_milli_hits_since_clear_ <<
      "\n\tfast milli hit usecs/query: " << (double)fast_milli_hits_since_clear_us_ / fast_milli_hits_since_clear_ <<
      "\n\tslow micro hit usecs/query: " << (double)slow_micro_hits_since_clear_us_ / slow_micro_hits_since_clear_ <<
      "\n\tfast micro hit usecs/query: " << (double)fast_micro_hits_since_clear_us_ / fast_micro_hits_since_clear_ <<
      "\n\texact hit usecs/query: " << (double)exact_hits_since_clear_us_ / exact_hits_since_clear_ <<
      "\n\treuse results usecs/query: " << (double)reuse_results_since_clear_us_ / reuse_results_since_clear_ <<
      "\n\tmiss FCL BV distance calculations: " << miss_fcl_bv_distance_calculations_ <<
      "\n\tmiss FCL primative distance calculations: " << miss_fcl_primative_distance_calculations_ <<
      "\n\thit FCL BV distance calculations: " << hit_fcl_bv_distance_calculations_ <<
      "\n\thit FCL primative distance calculations: " << hit_fcl_primative_distance_calculations_);
}

void Costmap3DQuery::addPCLPolygonToFCLTriangles(
    const pcl::Vertices& polygon,
    std::vector<fcl::Triangle>* fcl_triangles)
{
  // Assume the polygons are convex. Break them into triangles.
  const std::size_t zero_index = polygon.vertices[0];
  for (int i=1; i < polygon.vertices.size() - 1; ++i)
  {
    fcl_triangles->push_back(fcl::Triangle(zero_index, polygon.vertices[i], polygon.vertices[i+1]));
  }
}


void Costmap3DQuery::addPCLPolygonMeshToRobotModel(
    const pcl::PolygonMesh& pcl_mesh,
    double padding,
    FCLRobotModel* robot_model)
{
  robot_mesh_points_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_mesh.cloud, *robot_mesh_points_);

  padPoints(robot_mesh_points_, padding);

  std::vector<fcl::Vector3<FCLFloat>> fcl_points;
  std::vector<fcl::Triangle> fcl_triangles;

  for (auto pcl_point : *robot_mesh_points_)
  {
    fcl_points.push_back(convertPCLPointToFCL(pcl_point));
  }

  for (auto polygon : pcl_mesh.polygons)
  {
    addPCLPolygonToFCLTriangles(polygon, &fcl_triangles);
  }

  robot_model->addSubModel(fcl_points, fcl_triangles);
}

void Costmap3DQuery::updateMeshResource(const std::string& mesh_resource, double padding)
{
  std::string filename = getFileNameFromPackageURL(mesh_resource);
  if (filename.size() == 0)
  {
    return;
  }
  int pcl_rv = pcl::io::loadPolygonFileSTL(filename, robot_mesh_);
  if (pcl_rv < 0)
  {
    ROS_ERROR_STREAM("Costmap3DQuery: unable to load STL mesh file " << filename
                     << " query object will always return collision!");
    return;
  }
  robot_model_.reset(new FCLRobotModel());
  robot_model_->beginModel();
  addPCLPolygonMeshToRobotModel(robot_mesh_, padding, robot_model_.get());
  robot_model_->endModel();

  crop_hull_.setHullCloud(robot_mesh_points_);
  crop_hull_.setHullIndices(robot_mesh_.polygons);
  crop_hull_.setCropOutside(true);
}

std::string Costmap3DQuery::getFileNameFromPackageURL(const std::string& url)
{
  /* Unfortunately the resource retriever does not have a way to get a path from a URL
   * (it only returns the contents of a URL in memory), and equally unfortunate, PCL does not
   * have a way to parse an STL from memory. Therefore we have to duplicate some of the
   * (slightly-modified) resource retriever code here. */
  // Reference: https://github.com/ros/resource_retriever/blob/kinetic-devel/src/retriever.cpp
  std::string mod_url = url;
  if (url.find("package://") == 0)
  {
    mod_url.erase(0, strlen("package://"));
    size_t pos = mod_url.find("/");
    if (pos == std::string::npos)
    {
      ROS_ERROR_STREAM("Costmap3DQuery: Could not parse package:// format URL "
                       << url
                       << " query object will always return collision!");
      return "";
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = ros::package::getPath(package.c_str());

    if (package_path.empty())
    {
      ROS_ERROR_STREAM("Costmap3DQuery: Package [" << package << "] from URL "
                       << url << " does not exist, "
                       << " query object will always return collision!");
      return "";
    }

    mod_url = package_path + mod_url;
  }
  else if (url.find("file://") == 0)
  {
    mod_url.erase(0, strlen("file://"));
  }

  return mod_url;
}

double Costmap3DQuery::footprintCost(geometry_msgs::Pose pose, Costmap3DQuery::QueryRegion query_region)
{
  // TODO: implement as cost query. For now, just translate a collision to cost
  return footprintCollision(pose, query_region) ? -1.0 : 0.0;
}

bool Costmap3DQuery::footprintCollision(geometry_msgs::Pose pose, Costmap3DQuery::QueryRegion query_region)
{
  // It is more correct and even more efficient to query the distance to find
  // collisions than it is to use FCL to directly find octomap collisions.
  // This is because our distance query correctly handles interior collisions,
  // which requires finding the nearest octomap box, which an FCL collision
  // will not do.
  return footprintDistance(pose, query_region) <= 0.0;
}

// Discern if the given octomap box is an interior collision and adjust
// distance or signed distance appropriately.
// This is done by using PCL's point-in-a-mesh call which works on concave
// meshes that represent closed polyhedra. FCL handles concave meshes as
// surfaces, not volumes.
double Costmap3DQuery::handleDistanceInteriorCollisions(
      double distance,
      bool signed_distance,
      const DistanceCacheEntry& cache_entry,
      const geometry_msgs::Pose& pose)
{
  if (distance < 0.0)
  {
    // Fast case, already a collision, nothing left to do.
    // This is fine in the signed_distance case as it is already calculating
    // the correct penetration depth.
    return distance;
  }

  // First find the penetration depth between the box and the mesh triangle.
  // If the box is outside the halfspace of the mesh triangle, the box is
  // outside the mesh and we are done. If the box is inside, we will
  // ultimately use this penetration depth as the new signed distance.
  fcl::Transform3<FCLFloat> pose_tf = poseToFCLTransform<FCLFloat>(pose);
  FCLFloat box_halfspace_distance = cache_entry.boxHalfspaceSignedDistance(pose_tf);
  if (box_halfspace_distance > 0.0)
  {
    return distance;
  }

  // Find out if the center of the box is inside the given footprint volume mesh.
  bool interior_collision = false;

  // Find the transform from the given octomap box to the robot model at the given pose.
  fcl::Transform3<FCLFloat> map_to_robot_transform(
      pose_tf.inverse() * cache_entry.octomap_box_tf);

  // transform the center of the box (which is the origin) into the robot frame
  fcl::Vector3<FCLFloat> box_center(map_to_robot_transform * fcl::Vector3<FCLFloat>::Zero());

  if (robot_model_->aabb_local.contain(box_center))
  {
    // The center of the octomap box is inside the AABB of the robot model
    // It may be inside the volume of the mesh footprint.
    // Create a PCL crop hull filter to see if the center of the box is inside
    // the volume of the mesh.
    pcl::PointCloud<pcl::PointXYZ> test_cloud;
    test_cloud.resize(1);
    test_cloud.is_dense = true;
    test_cloud.width = 1;
    test_cloud.height = 1;
    test_cloud.points[0] = convertFCLPointToPCL(box_center);
    std::vector<int> indices;
    crop_hull_.applyFilter(test_cloud, indices);
    if (indices.size() > 0)
    {
      interior_collision = true;
    }
  }
  if (interior_collision)
  {
    if (signed_distance)
    {
      // It might be tempting to create a more accurate penetration distance
      // than using the box-halfspace distance. However, it is important to
      // stay consistent with the case when the mesh and box touch, which uses
      // the box-halfspace distance for penetration depth. This is important,
      // as planners which use the derivative to move a pose might get an
      // inward derivative around a boundary if using a more accurate
      // measurement here, as it may be closer than the penetration depth
      // modeling with a halfspace near concave parts of the mesh.
      distance = box_halfspace_distance;
    }
    else
    {
      // Non-signed distance case, just return a lethal collision
      // Note we could use the box_halfspace_distance, but we would need to
      // change 0.0 to a negative to indicate collision. Just always return
      // -1.0 if doing non-signed distance.
      distance = -1.0;
    }
  }
  return distance;
}

Costmap3DQuery::FCLCollisionObjectPtr Costmap3DQuery::getRobotCollisionObject(const geometry_msgs::Pose& pose) const
{
  // We must make our own FCLCollisionObject so we can modify the transform.
  // Otherwise, the queries will not be thread safe, as we do not have a write
  // lock except for maintaining the query caches.
  FCLCollisionObjectPtr robot_obj(new FCLCollisionObject(robot_model_));
  robot_obj->setTransform(poseToFCLTransform<FCLFloat>(pose));
  return robot_obj;
}

Costmap3DQuery::FCLCollisionObjectPtr Costmap3DQuery::getWorldCollisionObject(const geometry_msgs::Pose& pose,
                                                                              Costmap3DQuery::QueryRegion query_region) const
{
  // We must make our own fcl::OcTree so we can modify the region of interest.
  // Otherwise, the queries will not be thread safe, as we do not have a write
  // lock except for maintaining the query caches.
  FCLCollisionObjectPtr world_obj;
  std::shared_ptr<fcl::OcTree<FCLFloat>> fcl_octree_ptr;
  fcl_octree_ptr.reset(new fcl::OcTree<FCLFloat>(octree_ptr_));
  if (query_region == LEFT || query_region == RIGHT)
  {
    fcl::Vector3<FCLFloat> normal;
    // Note, for FCL halfspaces, the inside space is that below the plane, so
    // the normal needs to point away from the query region
    if (query_region == LEFT)
    {
      normal << 0.0, -1.0, 0.0;
    }
    else if(query_region == RIGHT)
    {
      normal << 0.0, 1.0, 0.0;
    }
    fcl::Halfspace<FCLFloat> halfspace(fcl::transform(
        fcl::Halfspace<FCLFloat>(normal, 0.0), poseToFCLTransform<FCLFloat>(pose)));
    fcl_octree_ptr->addToRegionOfInterest(halfspace);
  }
  world_obj = FCLCollisionObjectPtr(new fcl::CollisionObject<FCLFloat>(fcl_octree_ptr));
  return world_obj;
}

double Costmap3DQuery::calculateDistance(geometry_msgs::Pose pose,
                                         bool signed_distance,
                                         Costmap3DQuery::QueryRegion query_region,
                                         bool reuse_past_result)
{
  upgrade_lock upgrade_lock(upgrade_mutex_);
  std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
  queries_since_clear_.fetch_add(1, std::memory_order_relaxed);
  if (!robot_model_)
  {
    // We failed to create a robot model.
    // The failure would have been logged, so simply return collision.
    return -1.0;
  }
  checkCostmap(upgrade_lock);
  assert(octree_ptr_);

  // FCL does not correctly handle an empty octomap.
  if (octree_ptr_->size() == 0)
  {
    return std::numeric_limits<double>::max();
  }

  if (reuse_past_result)
  {
    // Reuse the past result (if there was one) and directly return the new distance.
    // In cases where the caller knows this is the correct behavior (such as
    // when making minor perturbations to estimate derivatives), this is
    // faster than having to calculate the hash and find the cache entry.
    if (tls_last_cache_entries_[query_region])
    {
      double distance = handleDistanceInteriorCollisions(
          tls_last_cache_entries_[query_region]->distanceToNewPose(pose, signed_distance),
          signed_distance,
          *tls_last_cache_entries_[query_region],
          pose);
      reuse_results_since_clear_.fetch_add(1, std::memory_order_relaxed);
      reuse_results_since_clear_us_.fetch_add(
          std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count(),
          std::memory_order_relaxed);
      return distance;
    }
  }

  DistanceCacheKey exact_cache_key(pose, query_region);
  auto exact_cache_entry = exact_distance_cache_.find(exact_cache_key);
  if (exact_cache_entry != exact_distance_cache_.end())
  {
    double distance = exact_cache_entry->second.distance;
    // Be sure to update the TLS last cache entry.
    // We do not need the write lock to update thread local storage.
    tls_last_cache_entries_[query_region] = &exact_cache_entry->second;
    exact_hits_since_clear_.fetch_add(1, std::memory_order_relaxed);
    exact_hits_since_clear_us_.fetch_add(
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count(),
        std::memory_order_relaxed);
    return distance;
  }

  FCLCollisionObjectPtr robot(getRobotCollisionObject(pose));
  FCLCollisionObjectPtr world(getWorldCollisionObject(pose, query_region));

  FCLFloat pose_distance = std::numeric_limits<FCLFloat>::max();
  fcl::DistanceRequest<FCLFloat> request;
  fcl::DistanceResult<FCLFloat> result;

  DistanceCacheKey milli_cache_key(pose, query_region, pose_milli_bins_per_meter_, pose_milli_bins_per_radian_);
  auto milli_cache_entry = milli_distance_cache_.find(milli_cache_key);
  bool milli_hit = false;
  if (milli_cache_entry != milli_distance_cache_.end())
  {
    if (milli_cache_entry->second.distance > milli_cache_threshold_)
    {
      double distance = handleDistanceInteriorCollisions(
          milli_cache_entry->second.distanceToNewPose(pose, signed_distance),
          signed_distance,
          milli_cache_entry->second,
          pose);
      // Be sure to update the TLS last cache entry.
      // We do not need the write lock to update thread local storage.
      tls_last_cache_entries_[query_region] = &milli_cache_entry->second;
      fast_milli_hits_since_clear_.fetch_add(1, std::memory_order_relaxed);
      fast_milli_hits_since_clear_us_.fetch_add(
          std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count(),
          std::memory_order_relaxed);
      return distance;
    }
    else
    {
      // we are too close to directly use the milli-cache, but use the
      // calculated distance as the pose_distance upper bound.
      // The upper-bound distance must not go through
      // handleDistanceInteriorCollisions, because that could artificially
      // prevent fcl::distance from getting the box nearest the mesh.
      double distance = milli_cache_entry->second.distanceToNewPose(pose, signed_distance);
      if (distance < pose_distance)
      {
        milli_hit = true;
        pose_distance = distance;
        milli_cache_entry->second.setupResult(&result);
      }
    }
  }

  DistanceCacheKey micro_cache_key(pose, query_region, pose_micro_bins_per_meter_, pose_micro_bins_per_radian_);
  auto micro_cache_entry = micro_distance_cache_.find(micro_cache_key);
  bool micro_hit = false;
  if (micro_cache_entry != micro_distance_cache_.end())
  {
    if (micro_cache_entry->second.distance > micro_cache_threshold_)
    {
      double distance = handleDistanceInteriorCollisions(
          micro_cache_entry->second.distanceToNewPose(pose, signed_distance),
          signed_distance,
          micro_cache_entry->second,
          pose);
      // Be sure to update the TLS last cache entry.
      // We do not need the write lock to update thread local storage.
      tls_last_cache_entries_[query_region] = &micro_cache_entry->second;
      fast_micro_hits_since_clear_.fetch_add(1, std::memory_order_relaxed);
      fast_micro_hits_since_clear_us_.fetch_add(
          std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count(),
          std::memory_order_relaxed);
      return distance;
    }
    else
    {
      // we are too close to directly use the micro-cache, but use the
      // calculated distance as the pose_distance upper bound.
      // The upper-bound distance must not go through
      // handleDistanceInteriorCollisions, because that could artificially
      // prevent fcl::distance from getting the box nearest the mesh.
      double distance = micro_cache_entry->second.distanceToNewPose(pose, signed_distance);
      if (distance < pose_distance)
      {
        micro_hit = true;
        pose_distance = distance;
        micro_cache_entry->second.setupResult(&result);
      }
    }
  }

  DistanceCacheKey cache_key(pose, query_region, pose_bins_per_meter_, pose_bins_per_radian_);
  bool cache_hit = false;
  // Assume a milli hit or micro hit is much better than the normal distance
  // cache at setting an upper bound, skip the normal cache lookup in such
  // cases.
  if (!milli_hit && !micro_hit)
  {
    auto cache_entry = distance_cache_.find(cache_key);
    if (cache_entry != distance_cache_.end())
    {
      cache_hit = true;
      // Cache hit, find the distance between the mesh triangle at the new pose
      // and the octomap box, and use this as our initial guess in the result.
      // This greatly prunes the search tree, yielding a big increase in runtime
      // performance.
      // This upper-bound distance must not go through
      // handleDistanceInteriorCollisions, because that could artificially
      // prevent fcl::distance from getting the box nearest the mesh.
      pose_distance = cache_entry->second.distanceToNewPose(pose, signed_distance);
      cache_entry->second.setupResult(&result);
    }
  }

  // Check the distance from the last cache entry too, and use it if it is a
  // better match. This is especialy useful if we miss every cache, but have a
  // last entry, and the queries are being done on a path.
  if (tls_last_cache_entries_[query_region])
  {
    double last_entry_distance = tls_last_cache_entries_[query_region]->distanceToNewPose(pose, signed_distance);
    if (last_entry_distance < pose_distance)
    {
      pose_distance = last_entry_distance;
      tls_last_cache_entries_[query_region]->setupResult(&result);
    }
  }

  // We could keep the read-lock held during the relatively long distance query
  // for two reasons:
  // 1) the world has a raw pointer to the octomap, so it must not be freed
  // 2) we do not want to be able to clear the cache during the distance
  //    query and then add an invalid cache entry below
  // However, it is not possible in practice for either of these things to
  // happen, as it is already necessary to either have the associated costmap
  // locked, or to have a copy (which by definition won't change). It allows
  // for much more parallelism to keep the read lock dropped during the
  // distance query.
  upgrade_lock.unlock();

  request.enable_nearest_points = true;
  // We will emulate signed distance ourselves, as FCL only emulates it anyway,
  // and for costmap query purposes, only getting one of the penetrations is
  // good enough (not the most maximal).
  request.enable_signed_distance = false;

  result.min_distance = pose_distance;

  double distance;

  if (pose_distance <= 0.0)
  {
    // The cached objects collided at the new pose.
    // FCL distance (signed or unsigned) between a mesh and octree
    // is not guaranteed to give the most negative distance in such cases,
    // as it stops as soon as a collision is found (due to the potentially very
    // large number of collisions). For the costmap use case, it isn't
    // super important that the most negative signed distance be found in the
    // signed case, as generally the use case for finding signed distance
    // is as an estimate of cost to move the robot to a pose, making it worse
    // to penetrate slightly more. In such cases, the main thing is that the
    // negative depth be one of the collisions, not necessarily the worst.
    // In the non-signed case, there is clearly nothing more to calculate, as
    // we already have a collision.
    distance = pose_distance;
  }
  else
  {
    distance = fcl::distance(world.get(), robot.get(), request, result);
  }

  // Note that it is possible for the result to be empty. The octomap might
  // only contain non-lethal leaves and we may have missed every cache.
  // Or the query region may be set to something other than ALL and there are
  // no map entries in the queried region.
  // If we get no result primitives, do not add null pointers to the cache!
  if (result.primitive1 && result.primitive2)
  {
    DistanceCacheEntry new_entry(result);

    // Emulate signed distance in a similar way as FCL. FCL re-does the whole
    // tree/BVH collision, but we already know the colliding primitives, so save
    // time by calculating their penetration depth directly.
    if (signed_distance && distance < 0.0)
    {
      distance = new_entry.distanceToNewPose(pose, signed_distance);
    }

    distance = handleDistanceInteriorCollisions(
        distance,
        signed_distance,
        new_entry,
        pose);

    new_entry.distance = distance;

    // Update distance caches.
    {
      // Get write access
      unique_lock write_lock(upgrade_mutex_);
      // While it may seem expensive to copy the cache entries into the
      // caches, it prevents cache aliasing and avoids dynamic memory.
      distance_cache_[cache_key] = new_entry;
      micro_distance_cache_[micro_cache_key] = new_entry;
      milli_distance_cache_[milli_cache_key] = new_entry;
      exact_distance_cache_[exact_cache_key] = new_entry;
      tls_last_cache_entries_[query_region] = &exact_distance_cache_[exact_cache_key];
    }
  }

  if (micro_hit)
  {
    slow_micro_hits_since_clear_.fetch_add(1, std::memory_order_relaxed);
    slow_micro_hits_since_clear_us_.fetch_add(
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count(),
        std::memory_order_relaxed);
    hit_fcl_bv_distance_calculations_.fetch_add(result.bv_distance_calculations);
    hit_fcl_primative_distance_calculations_.fetch_add(result.primative_distance_calculations);
  }
  else if (milli_hit)
  {
    slow_milli_hits_since_clear_.fetch_add(1, std::memory_order_relaxed);
    slow_milli_hits_since_clear_us_.fetch_add(
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count(),
        std::memory_order_relaxed);
    hit_fcl_bv_distance_calculations_.fetch_add(result.bv_distance_calculations);
    hit_fcl_primative_distance_calculations_.fetch_add(result.primative_distance_calculations);
  }
  else if (cache_hit)
  {
    hits_since_clear_.fetch_add(1, std::memory_order_relaxed);
    hits_since_clear_us_.fetch_add(
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count(),
        std::memory_order_relaxed);
    hit_fcl_bv_distance_calculations_.fetch_add(result.bv_distance_calculations);
    hit_fcl_primative_distance_calculations_.fetch_add(result.primative_distance_calculations);
  }
  else
  {
    misses_since_clear_us_.fetch_add(
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count(),
        std::memory_order_relaxed);
    miss_fcl_bv_distance_calculations_.fetch_add(result.bv_distance_calculations);
    miss_fcl_primative_distance_calculations_.fetch_add(result.primative_distance_calculations);
  }

  return distance;
}

double Costmap3DQuery::footprintDistance(geometry_msgs::Pose pose,
                                         Costmap3DQuery::QueryRegion query_region,
                                         bool reuse_past_result)
{
  return calculateDistance(pose, false, query_region, reuse_past_result);
}

double Costmap3DQuery::footprintSignedDistance(geometry_msgs::Pose pose,
                                               Costmap3DQuery::QueryRegion query_region,
                                               bool reuse_past_result)
{
  return calculateDistance(pose, true, query_region, reuse_past_result);
}

}  // namespace costmap_3d
