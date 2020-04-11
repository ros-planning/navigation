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
#ifndef COSTMAP_3D_COSTMAP_3D_QUERY_H_
#define COSTMAP_3D_COSTMAP_3D_QUERY_H_

#include <string>
#include <memory>
#include <unordered_map>
#include <limits>
#include <atomic>
#include <boost/thread/shared_mutex.hpp>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/utility.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>
#include <costmap_3d/layered_costmap_3d.h>
#include <costmap_3d/crop_hull.h>
#include <costmap_3d/GetPlanCost3DService.h>

namespace costmap_3d
{

/** @brief Query a 3D Costmap. */
class Costmap3DQuery
{
public:
  /**
   * @brief  Construct a query object associated with a layered costmap 3D.
   * The query will always be performed on the current layered costmap 3D,
   * and the corresponding costmap 3D must be locked during queries.
   */
  Costmap3DQuery(
      const LayeredCostmap3D* layered_costmap_3d,
      const std::string& mesh_resource,
      double padding = 0.0,
      unsigned int pose_bins_per_meter = 4,
      unsigned int pose_bins_per_radian = 4,
      unsigned int pose_milli_bins_per_meter = 20,
      unsigned int pose_milli_bins_per_radian = 20,
      unsigned int pose_micro_bins_per_meter = 1024,
      unsigned int pose_micro_bins_per_radian = 1024);

  /**
   * @brief  Construct a query object on a particular costmap 3D.
   * This constructor creates an internal copy of the passed costmap
   * and all queries will be performed on that copy.
   * This is useful for doing a buffered query.
   * Note that the costmap in question should not update during the
   * constructor. If it is the underlying costmap in the LayeredCostmap3D,
   * be sure to lock the LayeredCostmap3D during construction.
   */
  Costmap3DQuery(
      const Costmap3DConstPtr& costmap_3d,
      const std::string& mesh_resource,
      double padding = 0.0,
      unsigned int pose_bins_per_meter = 4,
      unsigned int pose_bins_per_radian = 4,
      unsigned int pose_milli_bins_per_meter = 20,
      unsigned int pose_milli_bins_per_radian = 20,
      unsigned int pose_micro_bins_per_meter = 1024,
      unsigned int pose_micro_bins_per_radian = 1024);

  virtual ~Costmap3DQuery();

  /// Which region of the map to query at the query pose.
  using QueryRegion = uint8_t;
  static constexpr QueryRegion ALL = GetPlanCost3DService::Request::COST_QUERY_REGION_ALL;
  static constexpr QueryRegion LEFT = GetPlanCost3DService::Request::COST_QUERY_REGION_LEFT;
  static constexpr QueryRegion RIGHT = GetPlanCost3DService::Request::COST_QUERY_REGION_RIGHT;
  static constexpr QueryRegion MAX = RIGHT+1;

  /** @brief Get the cost to put the robot base at the given pose.
   *
   * The region of the map considered is limited by the query_region.
   *
   * It is assumed the pose is in the frame of the costmap.
   * return value represents the cost of the pose
   * negative is collision, zero is free.
   * For query objects which track the master layered costmap,
   * the caller must be holding the lock on the associated costmap. */
  virtual double footprintCost(geometry_msgs::Pose pose, QueryRegion query_region = ALL);

  /** @brief Return whether the given pose is in collision.
   *
   * The region of the map considered is limited by the query_region.
   *
   * It is assumed the pose is in the frame of the costmap.
   * For query objects which track the master layered costmap,
   * the caller must be holding the lock on the associated costmap.
   */
  virtual bool footprintCollision(geometry_msgs::Pose pose, QueryRegion query_region = ALL);

  /** @brief Return minimum distance to nearest costmap object.
   *
   * The region of the map considered is limited by the query_region.
   *
   * It is assumed the pose is in the frame of the costmap.
   * This returns the minimum distance, or negative for penetration.
   * Negative values are not based on penetration depth and may always be a
   * constant value like -1.0. If penetrations must be modeled use
   * footprintSignedDistance. A case where penetrations should be modeled would
   * be to approximate the motion necessary to avoid the penetrating obstacle.
   * For query objects which track the master layered costmap,
   * the caller must be holding the lock on the associated costmap.
   *
   * reuse_past_result can be used to force a subsequent query re-use a
   * past result at a new pose. Setting this to true will prevent the query
   * from attempting to find a better answer, so use with care, only when
   * altering the pose by a small amount.
   */
  virtual double footprintDistance(geometry_msgs::Pose pose,
                                   QueryRegion query_region = ALL,
                                   bool reuse_past_result = false);

  /** @brief Return minimum signed distance to nearest costmap object.
   *
   * The region of the map considered is limited by the query_region.
   *
   * It is assumed the pose is in the frame of the costmap.
   * This returns the signed distance. For non-pentration cases, the return
   * value is the same as footprintDistance. For penetrating collisions, an
   * approximation of one of the negative penetration depths will be returned.
   * This approximation has the property that for most cases the derivative
   * obtained by a small perturbation in pose will point away from the
   * penetration. However, if the pose is moved enough away from the
   * penetration, some other pentrating point may be choosen in the case of
   * multiple penetrations. This should normally result in a net negative
   * direction to the derivative, but the more penetrations the worse the
   * approximation becomes. Finding the absolute deepest penetration depth is
   * not worth the extra computational cost, and for navigation purposes is
   * rarely necessary.
   * For query objects which track the master layered costmap,
   * the caller must be holding the lock on the associated costmap.
   *
   * reuse_past_result can be used to force a subsequent query re-use a
   * past result at a new pose. Setting this to true will prevent the query
   * from attempting to find a better answer, so use with care, only when
   * altering the pose by a small amount.
   */
  virtual double footprintSignedDistance(geometry_msgs::Pose pose,
                                         QueryRegion query_region = ALL,
                                         bool reuse_past_result = false);

  /** @brief get a const reference to the padded robot mesh points being used */
  const pcl::PointCloud<pcl::PointXYZ>& getRobotMeshPoints() const {return *robot_mesh_points_;}

  /** @brief get a const reference to the mesh polygons being used */
  const std::vector<pcl::Vertices>& getRobotMeshPolygons() const {return robot_mesh_.polygons;}

protected:
  const LayeredCostmap3D* layered_costmap_3d_;

  using upgrade_mutex = boost::upgrade_mutex;
  using upgrade_lock = boost::upgrade_lock<upgrade_mutex>;
  using upgrade_to_unique_lock = boost::upgrade_to_unique_lock<upgrade_mutex>;
  using unique_lock = boost::unique_lock<upgrade_mutex>;

  /** @brief Ensure query map matches currently active costmap.
   * Note: must be called on every query to ensure that the correct Costmap3D is being queried.
   * The LayeredCostmap3D can reallocate the Costmap3D, such as when
   * the resolution changes.
   * Note: assumes the costmap is locked. The costmap should always be locked
   * during query calls when this is called.
   */
  virtual void checkCostmap(upgrade_lock& upgrade_lock);

  /** @brief Update the mesh to use for queries. */
  virtual void updateMeshResource(const std::string& mesh_resource, double padding = 0.0);

  /** @brief core of distance calculations */
  virtual double calculateDistance(geometry_msgs::Pose pose,
                                   bool signed_distance = false,
                                   QueryRegion query_region = ALL,
                                   bool reuse_past_result = false);

private:
  // Common initialization between all constrcutors
  void init();
  // synchronize this object for parallel queries
  upgrade_mutex upgrade_mutex_;
  // returns path to package file, or empty on error
  std::string getFileNameFromPackageURL(const std::string& url);

  // Save the PCL model of the mesh to use with crop hull
  pcl::PolygonMesh robot_mesh_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr robot_mesh_points_;

  // Use the costmap_3d version of CropHull that is thread safe
  CropHull<pcl::PointXYZ> crop_hull_;

  using FCLFloat = double;
  using FCLRobotModel = fcl::BVHModel<fcl::OBBRSS<FCLFloat>>;
  using FCLRobotModelPtr = std::shared_ptr<FCLRobotModel>;

  FCLRobotModelPtr robot_model_;

  using FCLCollisionObject = fcl::CollisionObject<FCLFloat>;
  using FCLCollisionObjectPtr = std::shared_ptr<FCLCollisionObject>;

  std::shared_ptr<const octomap::OcTree> octree_ptr_;
  inline const fcl::Transform3<FCLFloat> poseToFCLTransform(const geometry_msgs::Pose& pose) const
  {
    return fcl::Transform3<FCLFloat>(
        fcl::Translation3<FCLFloat>(pose.position.x, pose.position.y, pose.position.z) *
        fcl::Quaternion<FCLFloat>(pose.orientation.w,
                                  pose.orientation.x,
                                  pose.orientation.y,
                                  pose.orientation.z));
  }
  FCLCollisionObjectPtr getRobotCollisionObject(const geometry_msgs::Pose& pose) const;
  FCLCollisionObjectPtr getWorldCollisionObject(const geometry_msgs::Pose& pose,
                                                QueryRegion query_region) const;

  void padPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr points, float padding)
  {
    for (unsigned int index = 0; index < points->size(); index++)
    {
      float x = points->points[index].x;
      float y = points->points[index].y;
      float dist = sqrt(x * x + y * y);
      if (dist == 0.0)
      {
        // Do not pad the origin.
        // The code below will divide by zero, go to the next point
        continue;
      }
      float nx = x / dist;
      float ny = y / dist;
      points->points[index].x = nx * (dist + padding);
      points->points[index].y = ny * (dist + padding);
    }  // for point
  }

  inline fcl::Vector3<FCLFloat> convertPCLPointToFCL(const pcl::PointXYZ& p)
  {
    return fcl::Vector3<FCLFloat>(p.x, p.y, p.z);
  }

  inline pcl::PointXYZ convertFCLPointToPCL(const fcl::Vector3<FCLFloat>& p)
  {
    return pcl::PointXYZ(p.x(), p.y(), p.z());
  }

  // Add fcl triangles to the mesh vector for all triangles in a PCL polygon
  void addPCLPolygonToFCLTriangles(
      const pcl::Vertices& polygon,
      std::vector<fcl::Triangle>* fcl_triangles);

  // Add a padded PCL mesh into the given robot model
  void addPCLPolygonMeshToRobotModel(
      const pcl::PolygonMesh& pcl_mesh,
      double padding,
      FCLRobotModel* robot_model);

  class DistanceCacheKey
  {
  public:
    DistanceCacheKey(const geometry_msgs::Pose& pose, QueryRegion query_region, int bins_per_meter, int bins_per_radian)
    {
      binned_pose_ = binPose(pose, bins_per_meter, bins_per_radian);
      query_region_ = query_region;
    }
    // Constructor for the "exact" cache where the poses are not binned
    DistanceCacheKey(const geometry_msgs::Pose& pose, QueryRegion query_region)
    {
      binned_pose_ = pose;
      query_region_ = query_region;
    }

    size_t hash_value() const
    {
      size_t rv = 0;
      hash_combine(rv, binned_pose_.orientation.x);
      hash_combine(rv, binned_pose_.orientation.y);
      hash_combine(rv, binned_pose_.orientation.z);
      hash_combine(rv, binned_pose_.orientation.w);
      hash_combine(rv, binned_pose_.position.x);
      hash_combine(rv, binned_pose_.position.y);
      hash_combine(rv, binned_pose_.position.z);
      hash_combine(rv, query_region_);
      return rv;
    }

    bool operator==(const DistanceCacheKey& rhs) const
    {
      return binned_pose_.orientation.x == rhs.binned_pose_.orientation.x &&
             binned_pose_.orientation.y == rhs.binned_pose_.orientation.y &&
             binned_pose_.orientation.z == rhs.binned_pose_.orientation.z &&
             binned_pose_.orientation.w == rhs.binned_pose_.orientation.w &&
             binned_pose_.position.x == rhs.binned_pose_.position.x &&
             binned_pose_.position.y == rhs.binned_pose_.position.y &&
             binned_pose_.position.z == rhs.binned_pose_.position.z &&
             query_region_ == rhs.query_region_;
    }

  protected:
    geometry_msgs::Pose binned_pose_;
    QueryRegion query_region_;

    //! Borrow boost's hash_combine directly to avoid having to pull in boost
    template <class T>
    inline void hash_combine(std::size_t& seed, const T& v) const
    {
      std::hash<T> hasher;
      seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    }

    // Bin a pose.
    inline geometry_msgs::Pose binPose(const geometry_msgs::Pose& pose,
                                       int bins_per_meter,
                                       int bins_per_radian)
    {
      geometry_msgs::Pose rv;
      rv.position.x = std::round(pose.position.x * bins_per_meter) / bins_per_meter;
      rv.position.y = std::round(pose.position.y * bins_per_meter) / bins_per_meter;
      rv.position.z = std::round(pose.position.z * bins_per_meter) / bins_per_meter;
      // bin orientation by RPY angles.
      // another way of binning would be binning by axis/angle
      double yaw, pitch, roll;
      tf2::getEulerYPR<geometry_msgs::Quaternion>(pose.orientation, yaw, pitch, roll);
      yaw = std::round(yaw * bins_per_radian) / bins_per_radian;
      pitch = std::round(pitch * bins_per_radian) / bins_per_radian;
      roll = std::round(roll * bins_per_radian) / bins_per_radian;
      tf2::Quaternion binned_q;
      binned_q.setRPY(roll, pitch, yaw);
      rv.orientation = tf2::toMsg(binned_q);
      return rv;
    }
  };

  struct DistanceCacheKeyHash
  {
    size_t operator()(const DistanceCacheKey& key) const
    {
      return key.hash_value();
    }
  };

  struct DistanceCacheKeyEqual
  {
    bool operator()(const DistanceCacheKey& lhs, const DistanceCacheKey& rhs) const
    {
      return lhs == rhs;
    }
  };

  class DistanceCacheEntry
  {
  public:
    DistanceCacheEntry() {}
    DistanceCacheEntry(const DistanceCacheEntry& rhs)
        : distance(rhs.distance),
          octomap_box(rhs.octomap_box),
          octomap_box_tf(rhs.octomap_box_tf),
          mesh_triangle(rhs.mesh_triangle)
    {
    }
    const DistanceCacheEntry& operator=(const DistanceCacheEntry& rhs)
    {
      distance = rhs.distance;
      octomap_box = rhs.octomap_box;
      octomap_box_tf = rhs.octomap_box_tf;
      mesh_triangle = rhs.mesh_triangle;
      return *this;
    }
    DistanceCacheEntry(const fcl::DistanceResult<FCLFloat>& result)
    {
      assert(result.primitive1);
      assert(result.primitive2);
      distance = result.min_distance;
      octomap_box = std::dynamic_pointer_cast<fcl::Box<FCLFloat>>(result.primitive1);
      octomap_box_tf = result.tf1;
      mesh_triangle = std::dynamic_pointer_cast<fcl::TriangleP<FCLFloat>>(result.primitive2);
      assert(octomap_box);
      assert(mesh_triangle);
    }
    void setupResult(fcl::DistanceResult<FCLFloat>* result)
    {
      result->primitive1 = octomap_box;
      result->primitive2 = mesh_triangle;
      result->tf1 = octomap_box_tf;
    }
    FCLFloat distanceToNewPose(geometry_msgs::Pose pose, bool signed_distance=false) const
    {
      // Turn pose into tf
      fcl::Transform3<FCLFloat> new_tf(
          fcl::Translation3<FCLFloat>(pose.position.x, pose.position.y, pose.position.z) *
          fcl::Quaternion<FCLFloat>(pose.orientation.w,
                                    pose.orientation.x,
                                    pose.orientation.y,
                                    pose.orientation.z));

      FCLFloat dist;
      // As of the time this code was written, the normal FCL API does not
      // allow box/triangle distance or signed distance queries.
      // Yet FCL internally does such checks all the time, so use the
      // internal mechanism for now.
      fcl::detail::GJKSolver_libccd<FCLFloat> solver;
      solver.shapeTriangleDistance(*octomap_box, octomap_box_tf,
                                   mesh_triangle->a, mesh_triangle->b, mesh_triangle->c, new_tf,
                                   &dist);
      if (signed_distance && dist < 0.0)
      {
        // The objects collide, so use the penetration depth as signed distance.
        // We do not (yet) use the contact points or normal points
        // We must provide contact points and normal vector to get penetration
        // depth calculation.
        fcl::Vector3<FCLFloat> contact_points;
        fcl::Vector3<FCLFloat> normal_vector;
        solver.shapeTriangleIntersect(*octomap_box, octomap_box_tf,
                                      mesh_triangle->a, mesh_triangle->b, mesh_triangle->c, new_tf,
                                      &contact_points,
                                      &dist,
                                      &normal_vector);
        // Signed distance is the negative penetration depth
        dist = -dist;
        assert(dist <= 0.0);
      }
      return dist;
    }
    FCLFloat distance;
    std::shared_ptr<fcl::Box<FCLFloat>> octomap_box;
    fcl::Transform3<FCLFloat> octomap_box_tf;
    std::shared_ptr<fcl::TriangleP<FCLFloat>> mesh_triangle;
  };
  // used by distance calculation to find interior collisions
  double handleDistanceInteriorCollisions(
      double distance,
      bool signed_distance,
      const DistanceCacheEntry& cache_entry,
      const geometry_msgs::Pose& pose);
  using DistanceCache = std::unordered_map<DistanceCacheKey, DistanceCacheEntry, DistanceCacheKeyHash, DistanceCacheKeyEqual>;
  using ExactDistanceCache = std::unordered_map<DistanceCacheKey, DistanceCacheEntry, DistanceCacheKeyHash, DistanceCacheKeyEqual>;
  // Keep track of last update in this thread to correctly reset any thread
  // local caches
  static thread_local unsigned int tls_last_layered_costmap_update_number_;
  static thread_local Costmap3DQuery* tls_last_instance_;
  /// Indexed by QueryRegion
  static thread_local DistanceCacheEntry* tls_last_cache_entries_[MAX];
  /**
   * The distance cache allows us to find a very good distance guess quickly.
   * The cache memorizes to a hash table for a pose rounded to the number of
   * bins the mesh triangle and octomap box pair for the last distance query
   * in that pose bin. The distance is recalculated with the new pose, and
   * used as the starting guess to radically prune the search tree for
   * distance queries. This results in no loss of correctness and a huge
   * speed-up when distance queries are over the same space.
   */
  DistanceCache distance_cache_;
  /**
   * The milli-distance cache allows a very fast path when the nearest
   * obstacle is more than a threshold of distance away. This results in a
   * massive speed up for cases where the nearest obstacle is futher than the
   * milli_cache_threshold_
   */
  double milli_cache_threshold_;
  DistanceCache milli_distance_cache_;
  /**
   * The micro-distance cache allows a very fast path when the nearest
   * obstacle is more than a threshold of distance away. This results in a
   * massive speed up for cases where the nearest obstacle is futher than the
   * micro_cache_threshold_
   */
  double micro_cache_threshold_;
  DistanceCache micro_distance_cache_;
  // Immediately return the distance for an exact duplicate query
  // This avoid any calculation in the case that the calculation has been done
  // since the costmap was updated.
  ExactDistanceCache exact_distance_cache_;
  unsigned int last_layered_costmap_update_number_;
  //! Distance cache bins per meter for binning the pose's position
  unsigned int pose_bins_per_meter_;
  //! Distance cache bins per radian for binning the pose's orientation
  unsigned int pose_bins_per_radian_;
  //! Milli-distance cache bins per meter for binning the pose's position
  unsigned int pose_milli_bins_per_meter_;
  //! Milli-distance cache bins per radian for binning the pose's position
  unsigned int pose_milli_bins_per_radian_;
  //! Micro-distance cache bins per meter for binning the pose's position
  unsigned int pose_micro_bins_per_meter_;
  //! Micro-distance cache bins per radian for binning the pose's position
  unsigned int pose_micro_bins_per_radian_;
  // Statistics gathered between clearing cycles
  void printStatistics();
  void clearStatistics();
  // Use std::atomic so we can increment with only the read lock held.
  // The write lock will be held when resetting these so they all reset
  // atomically
  std::atomic<unsigned int> queries_since_clear_;
  std::atomic<unsigned int> hits_since_clear_;
  std::atomic<unsigned int> fast_milli_hits_since_clear_;
  std::atomic<unsigned int> slow_milli_hits_since_clear_;
  std::atomic<unsigned int> fast_micro_hits_since_clear_;
  std::atomic<unsigned int> slow_micro_hits_since_clear_;
  std::atomic<unsigned int> exact_hits_since_clear_;
  std::atomic<unsigned int> reuse_results_since_clear_;
  std::atomic<uint64_t> misses_since_clear_us_;
  std::atomic<uint64_t> hits_since_clear_us_;
  std::atomic<uint64_t> fast_milli_hits_since_clear_us_;
  std::atomic<uint64_t> slow_milli_hits_since_clear_us_;
  std::atomic<uint64_t> fast_micro_hits_since_clear_us_;
  std::atomic<uint64_t> slow_micro_hits_since_clear_us_;
  std::atomic<uint64_t> exact_hits_since_clear_us_;
  std::atomic<uint64_t> reuse_results_since_clear_us_;
  std::atomic<size_t> hit_fcl_bv_distance_calculations_;
  std::atomic<size_t> hit_fcl_primative_distance_calculations_;
  std::atomic<size_t> miss_fcl_bv_distance_calculations_;
  std::atomic<size_t> miss_fcl_primative_distance_calculations_;
};

using Costmap3DQueryPtr = std::shared_ptr<Costmap3DQuery>;
using Costmap3DQueryConstPtr = std::shared_ptr<const Costmap3DQuery>;

}  // namespace costmap_3d

#endif  // COSTMAP_3D_COSTMAP_3D_QUERY_H_
