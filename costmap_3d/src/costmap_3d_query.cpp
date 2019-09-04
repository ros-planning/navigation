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

Costmap3DQuery::Costmap3DQuery(
    const LayeredCostmap3D* layered_costmap_3d,
    const std::string& mesh_resource,
    double padding,
    unsigned int pose_bins_per_meter,
    unsigned int pose_bins_per_radian,
    unsigned int pose_micro_bins_per_meter,
    unsigned int pose_micro_bins_per_radian)
  : layered_costmap_3d_(layered_costmap_3d),
    pose_bins_per_meter_(pose_bins_per_meter),
    pose_bins_per_radian_(pose_bins_per_radian),
    pose_micro_bins_per_meter_(pose_micro_bins_per_meter),
    pose_micro_bins_per_radian_(pose_micro_bins_per_radian)
{
  updateMeshResource(mesh_resource, padding);
}

Costmap3DQuery::Costmap3DQuery(const Costmap3DConstPtr& costmap_3d,
    const std::string& mesh_resource,
    double padding,
    unsigned int pose_bins_per_meter,
    unsigned int pose_bins_per_radian,
    unsigned int pose_micro_bins_per_meter,
    unsigned int pose_micro_bins_per_radian)
  : pose_bins_per_meter_(pose_bins_per_meter),
    pose_bins_per_radian_(pose_bins_per_radian),
    pose_micro_bins_per_meter_(pose_micro_bins_per_meter),
    pose_micro_bins_per_radian_(pose_micro_bins_per_radian)
{
  // Make a local copy of the costmap in question
  // It would be awesome if the Costmap3D had a way to snapshot
  // or copy-on-write. As it stands, for many scenarios involving
  // space-limited local costmaps, copying a 3D costmap will only take a
  // couple millseconds and is better than leaving the costmap locked for
  // an entire planning cycle.
  octree_ptr_.reset(new Costmap3D(*costmap_3d));
  std::shared_ptr<fcl::OcTree<FCLFloat>> fcl_octree_ptr;
  fcl_octree_ptr.reset(new fcl::OcTree<FCLFloat>(octree_ptr_));
  world_obj_ = FCLCollisionObjectPtr(new fcl::CollisionObject<FCLFloat>(fcl_octree_ptr));
  updateMeshResource(mesh_resource, padding);
}

// Be sure to deep-copy any object with state that we modify.
Costmap3DQuery::Costmap3DQuery(const Costmap3DQuery& rhs)
  : // Share a pointer to the layered_costmap_3d_
    layered_costmap_3d_(rhs.layered_costmap_3d_),
    // Copy the pose-bin constants
    pose_bins_per_meter_(rhs.pose_bins_per_meter_),
    pose_bins_per_radian_(rhs.pose_bins_per_radian_),
    pose_micro_bins_per_meter_(rhs.pose_micro_bins_per_meter_),
    pose_micro_bins_per_radian_(rhs.pose_micro_bins_per_radian_),
    // Share a pointer to the robot model
    robot_model_(rhs.robot_model_),
    // Create a new robot_obj_, as we need to modify the transform.
    robot_obj_(new FCLCollisionObject(robot_model_)),
    // Share a pointer to the octree (if there was one)
    octree_ptr_(rhs.octree_ptr_),
    // Start out sharing a pointer to the world object.
    // (We may make our own if the layered_costmap_3d_'s costmap is re-created)
    world_obj_(rhs.world_obj_)
    // Do NOT copy our caches as we do not synchrnoize.
    // The whole point of this copy constructor is so multiple planning
    // threads could be querying the same octomap with separate caches in
    // order to avoid locking.
{
}

Costmap3DQuery::~Costmap3DQuery()
{
}

void Costmap3DQuery::checkCostmap()
{
  if (layered_costmap_3d_)
  {
    if (layered_costmap_3d_->getCostmap3D() != octree_ptr_)
    {
      // The octomap has been reallocated, change where we are pointing.
      octree_ptr_ = layered_costmap_3d_->getCostmap3D();
      std::shared_ptr<fcl::OcTree<FCLFloat>> fcl_octree_ptr;
      fcl_octree_ptr.reset(new fcl::OcTree<FCLFloat>(octree_ptr_));
      world_obj_ = FCLCollisionObjectPtr(new fcl::CollisionObject<FCLFloat>(fcl_octree_ptr));
    }
    // The costmap has been updated since the last query, reset our caches
    if (last_layered_costmap_update_number_ != layered_costmap_3d_->getNumberOfUpdates())
    {
      // For simplicity, on every update, clear out the collision cache.
      // This is not strictly necessary. The mesh is stored in the cache and does
      // not change. The only thing that is changing is the costmap. We could go
      // through the delta map and only remove entries that have had the
      // corresponding octomap cell go away. This may be implemented as a future
      // improvement.
      distance_cache_.clear();
      micro_distance_cache_.clear();
    }
    last_layered_costmap_update_number_ = layered_costmap_3d_->getNumberOfUpdates();
  }
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
  pcl::PointCloud<pcl::PointXYZ> mesh_points;
  pcl::fromPCLPointCloud2(pcl_mesh.cloud, mesh_points);

  std::vector<fcl::Vector3<FCLFloat>> fcl_points;
  std::vector<fcl::Triangle> fcl_triangles;

  for (auto pcl_point : mesh_points)
  {
    fcl_points.push_back(padPCLPointToFCL(pcl_point, padding));
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
  pcl::PolygonMesh mesh;
  int pcl_rv = pcl::io::loadPolygonFileSTL(filename, mesh);
  if (pcl_rv < 0)
  {
    ROS_ERROR_STREAM("Costmap3DQuery: unable to load STL mesh file " << filename
                     << " query object will always return collision!");
    return;
  }
  robot_model_.reset(new FCLRobotModel());
  robot_model_->beginModel();
  addPCLPolygonMeshToRobotModel(mesh, padding, robot_model_.get());
  robot_model_->endModel();
  robot_obj_ = FCLCollisionObjectPtr(new FCLCollisionObject(robot_model_));
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

double Costmap3DQuery::footprintCost(geometry_msgs::Pose pose)
{
  // TODO: implement as cost query. For now, just translate a collision to cost
  return footprintCollision(pose) ? -1.0 : 0.0;
}

bool Costmap3DQuery::footprintCollision(geometry_msgs::Pose pose)
{
  if (!robot_obj_)
  {
    // We failed to create a robot model.
    // The failure would have been logged, so simply return collision.
    return true;
  }
  checkCostmap();
  assert(world_obj_);

  // FCL does not correctly handle an empty octomap.
  if (octree_ptr_->size() == 0)
  {
    // There is nothing to collide, so there will be no collision
    return false;
  }

  FCLCollisionObjectPtr robot(getRobotCollisionObject(pose));
  FCLCollisionObjectPtr world(getWorldCollisionObject());

  fcl::CollisionRequest<FCLFloat> request;
  fcl::CollisionResult<FCLFloat> result;

  fcl::collide(world.get(), robot.get(), request, result);

  return result.isCollision();
}

double Costmap3DQuery::footprintDistance(geometry_msgs::Pose pose)
{
  if (!robot_obj_)
  {
    // We failed to create a robot model.
    // The failure would have been logged, so simply return collision.
    return -1.0;
  }
  checkCostmap();
  assert(world_obj_);

  // FCL does not correctly handle an empty octomap.
  if (octree_ptr_->size() == 0)
  {
    return std::numeric_limits<double>::max();
  }

  FCLCollisionObjectPtr robot(getRobotCollisionObject(pose));
  FCLCollisionObjectPtr world(getWorldCollisionObject());

  FCLFloat pose_distance = std::numeric_limits<FCLFloat>::max();

  DistanceCacheKey micro_cache_key(pose, pose_micro_bins_per_meter_, pose_micro_bins_per_radian_);
  // if we hit the micro cache, use the result directly.
  auto micro_cache_entry = micro_distance_cache_.find(micro_cache_key);
  if (micro_cache_entry != micro_distance_cache_.end())
  {
    return micro_cache_entry->second.distanceToNewPose(pose);
  }

  fcl::DistanceRequest<FCLFloat> request;
  fcl::DistanceResult<FCLFloat> result;

  DistanceCacheKey cache_key(pose, pose_bins_per_meter_, pose_bins_per_radian_);
  auto cache_entry = distance_cache_.find(cache_key);
  if (cache_entry != distance_cache_.end())
  {
    // Cache hit, find the distance between the mesh triangle at the new pose
    // and the octomap box, and use this as our initial guess in the result.
    // This greatly prunes the search tree, yielding a big increase in runtime
    // performance.
    pose_distance = cache_entry->second.distanceToNewPose(pose);
    cache_entry->second.setupResult(&result);
  }
  else if(distance_cache_.size() > 0)
  {
    // Cache miss, use the beginning of the cache to set the initial guess.
    // This still prunes the search better than no initial guess in certain
    // circumstances and is relatively cheap to calculate. This guess is still
    // a correct upper bound, as the minimum distance must be less or equal to
    // the mesh triangle at the new pose and the old nearest octomap cell.
    auto begin = distance_cache_.begin();
    pose_distance = begin->second.distanceToNewPose(pose);
    begin->second.setupResult(&result);
  }

  request.enable_nearest_points = true;

  result.min_distance = pose_distance;

  double distance = fcl::distance(world.get(), robot.get(), request, result);

  // Update distance caches
  const DistanceCacheEntry& new_entry = DistanceCacheEntry(result);
  distance_cache_[cache_key] = new_entry;
  micro_distance_cache_[micro_cache_key] = new_entry;

  return distance;
}

double Costmap3DQuery::footprintSignedDistance(geometry_msgs::Pose pose)
{
  if (!robot_obj_)
  {
    // We failed to create a robot model.
    // The failure would have been logged, so simply return collision.
    return -1.0;
  }
  checkCostmap();
  // TODO: Figure out how to apply distance cache to signed distance.
  // TODO: Think about how to make this work w/ meshes representing a solid and octomaps
  assert(world_obj_);

  // FCL does not correctly handle an empty octomap.
  if (octree_ptr_->size() == 0)
  {
    return std::numeric_limits<double>::max();
  }

  FCLCollisionObjectPtr robot(getRobotCollisionObject(pose));
  FCLCollisionObjectPtr world(getWorldCollisionObject());

  fcl::DistanceRequest<FCLFloat> request;
  fcl::DistanceResult<FCLFloat> result;

  request.enable_signed_distance = true;

  return fcl::distance(world.get(), robot.get(), request, result);
}

}  // namespace costmap_3d
