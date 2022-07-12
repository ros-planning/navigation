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
#ifndef COSTMAP_3D_COSTMAP_3D_ROS_H_
#define COSTMAP_3D_COSTMAP_3D_ROS_H_

#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <tuple>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_3d/layered_costmap_3d.h>
#include <costmap_3d/costmap_3d_publisher.h>
#include <costmap_3d/costmap_3d_query.h>
#include <costmap_3d_msgs/GetPlanCost3DAction.h>
#include <costmap_3d_msgs/GetPlanCost3DService.h>
#include <costmap_3d_msgs/RayQuery3DAction.h>
#include <costmap_3d_msgs/RayQuery3DService.h>
#include <costmap_3d/Costmap3DConfig.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.hpp>
#include <geometry_msgs/Pose.h>

namespace costmap_3d
{

/** @brief A ROS wrapper for a 3D Costmap. */
class Costmap3DROS : public costmap_2d::Costmap2DROS
{
  using super = costmap_2d::Costmap2DROS;
public:
  /**
   * @brief  Constructor for the wrapper
   * @param name The name for this costmap
   * @param tf A reference to a TransformListener
   */
  Costmap3DROS(const std::string &name, tf2_ros::Buffer& tf);
  virtual ~Costmap3DROS();

  /**
   * @brief  Subscribes to sensor topics if necessary and starts costmap
   * updates, can be called to restart the costmap after calls to either
   * stop() or pause()
   */
  virtual void start();

  /**
   * @brief  Stops costmap updates and unsubscribes from sensor topics
   */
  virtual void stop();

  /** @brief Update the costmap from all layers, publishing as necessary. */
  virtual void updateMap();

  /**
   * @brief Reset all layers
   */
  virtual void resetLayers();

  /** @brief Returns true if all layers have current sensor data. */
  virtual bool isCurrent();

  /* Unlike Costmap2D, provide no interface to the 3D layered costmap internals.
   * Instead provide specific interfaces to clear 3D layers and get 3D collision
   * information from the map. Note that altering the included 2D master
   * costmap has no effect on the 3D data. The 3D costmap may be mirrored to
   * the 2D costmap using the costmap3DTo2D plugin. */

  /** @brief Get a buffered query object associated with a copy of our costmap.
   * Note: it is assumed that this object is not locked, as the lock is
   * acquired automatically.
   */
  virtual std::shared_ptr<Costmap3DQuery> getBufferedQuery(
          const std::string& footprint_mesh_resource = "",
          double padding = NAN);

  /** @brief update the given buffered query to hold a copy of the current costmap */
  virtual void updateBufferedQuery(std::shared_ptr<Costmap3DQuery> query);

  /** @brief Get a query object associated with our layered costmap.
   * Note: it is up to the caller to ensure the costmap is properly
   * locked while executing queries, or unpredictable behavior will occur!
   */
  virtual std::shared_ptr<Costmap3DQuery> getAssociatedQuery(
          const std::string& footprint_mesh_resource = "",
          double padding = NAN);

  /** @brief Get the names of the layers (both 2D and 3D) in the costmap. */
  virtual std::set<std::string> getLayerNames();

  // Be sure to have all versions of resetBoundingBox from parent referencable
  using super::resetBoundingBox;

  /** @brief Reset the costmap within the given axis-aligned bounding box in world coordinates for the given layers. */
  virtual void resetBoundingBox(geometry_msgs::Point min, geometry_msgs::Point max, const std::set<std::string>& layers);

  /** @brief Get the cost to put the robot base at the given pose in the 3D costmap.
   *
   * This call only checks the layered 3D costmap. Any 2D layers are ignored.
   *
   * It is assumed the pose is in the frame of the costmap, and the current
   * state of the costmap is queried at the given pose.
   * Currently the padding is applied in all directions.
   *
   * The caller must be holding the lock.
   *
   * @param pose                    pose to query
   * @param footprint_mesh_resource which footprint mesh to use, empty string means default
   * @param padding                 padding to add to the footprint. NAN means use default padding
   * @param query_region            region of map to query at given pose
   *
   * @returns negative for lethal, otherwise the cost of the pose */
  virtual double footprintCost(geometry_msgs::Pose pose,
                               const std::string& footprint_mesh_resource = "",
                               double padding = NAN,
                               Costmap3DQuery::QueryRegion query_region = Costmap3DQuery::ALL);

  virtual double footprintCost(double x, double y, double theta,
                               const std::string& footprint_mesh_resource = "",
                               double padding = NAN,
                               Costmap3DQuery::QueryRegion query_region = Costmap3DQuery::ALL)
  {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    pose.orientation = tf2::toMsg(q);
    return footprintCost(pose, footprint_mesh_resource, padding, query_region);
  }

  /** @brief Return whether the given pose is in collision in the 3D costmap.
   *
   * This call only checks the layered 3D costmap. Any 2D layers are ignored.
   *
   * The caller must be holding the lock.
   *
   * @param pose                    pose to query
   * @param footprint_mesh_resource which footprint mesh to use, empty string means default
   * @param padding                 padding to add to the footprint. NAN means use default padding
   *
   * @returns true if in collision, false otherwise */
  virtual bool footprintCollision(geometry_msgs::Pose pose,
                                  const std::string& footprint_mesh_resource = "",
                                  double padding = NAN,
                                  Costmap3DQuery::QueryRegion query_region = Costmap3DQuery::ALL);

  /** @brief Return minimum distance to the nearest lethal 3D costmap object.
   * This returns the minimum unsigned distance or negative on a collision.
   * Negative values are not exact minimum distances. If exact minimum is
   * required use footprintSignedDistance.
   *
   * This call only checks the layered 3D costmap. Any 2D layers are ignored.
   *
   * The caller must be holding the lock.
   *
   * @param pose                    pose to query
   * @param footprint_mesh_resource which footprint mesh to use, empty string means default
   * @param padding                 padding to add to the footprint. NAN means use default padding
   *
   * @returns negative on collision, otherwise distance to nearest obstacle */
  virtual double footprintDistance(geometry_msgs::Pose pose,
                                   const std::string& footprint_mesh_resource = "",
                                   double padding = NAN,
                                   Costmap3DQuery::QueryRegion query_region = Costmap3DQuery::ALL);

  /** @brief Return minimum signed distance to nearest 3D costmap object.
   * This returns the minimum signed distance. So, the deeper a pose goes into
   * obstacles, the more negative the return value becomes.
   * The caller must be holding the lock.
   *
   * This call only checks the layered 3D costmap. Any 2D layers are ignored.
   *
   * @param pose                    pose to query
   * @param footprint_mesh_resource which footprint mesh to use, empty string means default
   * @param padding                 padding to add to the footprint. NAN means use default padding
   *
   * @returns exact signed distance to nearest obstacle */
  virtual double footprintSignedDistance(geometry_msgs::Pose pose,
                                         const std::string& footprint_mesh_resource = "",
                                         double padding = NAN,
                                         Costmap3DQuery::QueryRegion query_region = Costmap3DQuery::ALL);

protected:
  ros::NodeHandle private_nh_;
  LayeredCostmap3D layered_costmap_3d_;

private:
  // Because the parent class starts a thread calling updateMap right away,
  // keep track if we are fully initialized to prevent doing anything with a
  // partially constructed object.
  std::mutex initialized_mutex_;
  bool initialized_;
  void reconfigureCB(costmap_3d::Costmap3DConfig &config, uint32_t level);
  pluginlib::ClassLoader<Layer3D> plugin_loader_;
  Costmap3DPublisher publisher_;
  ros::Publisher footprint_pub_;
  bool visualize_ray_query_miss_ = true;
  ros::Publisher ray_query_3d_visualization_pub_;
  dynamic_reconfigure::Server<costmap_3d::Costmap3DConfig> dsrv_;
  actionlib::SimpleActionServer<costmap_3d_msgs::GetPlanCost3DAction> get_plan_cost_action_srv_;
  ros::ServiceServer get_plan_cost_srv_;
  actionlib::SimpleActionServer<costmap_3d_msgs::RayQuery3DAction> ray_query_action_srv_;
  ros::ServiceServer ray_query_srv_;

  // Because there are two entry points to the services, and because the
  // services share buffered query maps, the octree being queried should
  // not change mid-query. Even though the queries themselves are thread
  // safe (for multi-threaded planners), to keep results consistent
  // for a buffered query to a single copy of the costmap, serialize
  // the ROS service and action server.
  std::mutex service_mutex_;
  void getPlanCost3DActionCallback(const actionlib::SimpleActionServer<costmap_3d_msgs::GetPlanCost3DAction>::GoalConstPtr& goal);
  bool getPlanCost3DServiceCallback(costmap_3d_msgs::GetPlanCost3DService::Request& request, costmap_3d_msgs::GetPlanCost3DService::Response& response);
  template <typename RequestType, typename ResponseType>
  void processPlanCost3D(RequestType& request, ResponseType& response);
  // Should be holding the service mutex, but not the costmap mutex
  virtual std::shared_ptr<Costmap3DQuery> getBufferedQueryForService(
          const std::string& footprint_mesh_resource = "",
          double padding = NAN);

  void rayQuery3DActionCallback(const actionlib::SimpleActionServer<costmap_3d_msgs::RayQuery3DAction>::GoalConstPtr& goal);
  bool rayQuery3DServiceCallback(costmap_3d_msgs::RayQuery3DService::Request& request, costmap_3d_msgs::RayQuery3DService::Response& response);
  template <typename RequestType, typename ResponseType>
  bool processRayQuery3D(RequestType& request, ResponseType& response);

  using QueryMapKey = std::tuple<std::string, double, Costmap3DQuery::QueryRegionScale>;
  // std::less does not work on Eigen3 vectors, so use this comparator.
  struct QueryMapKeyCompare
  {
    bool operator()(const QueryMapKey& lhs, const QueryMapKey& rhs) const
    {
      const auto& lhs0 = std::get<0>(lhs);
      const auto& lhs1 = std::get<1>(lhs);
      const auto& lhs2 = std::get<2>(lhs);
      const auto& rhs0 = std::get<0>(rhs);
      const auto& rhs1 = std::get<1>(rhs);
      const auto& rhs2 = std::get<2>(rhs);
      if (lhs0 < rhs0)
      {
        return true;
      }
      if (lhs0 > rhs0)
      {
        return false;
      }
      if (lhs1 < rhs1)
      {
        return true;
      }
      if (lhs1 > rhs1)
      {
        return false;
      }
      for (unsigned int i=0; i<3; ++i)
      {
        if (lhs2(i) < rhs2(i))
        {
          return true;
        }
        if (lhs2(i) > rhs2(i))
        {
          return false;
        }
      }
      return false;
    }
  };
  using QueryMap = std::map<QueryMapKey, std::shared_ptr<Costmap3DQuery>, QueryMapKeyCompare>;
  QueryMap query_map_;
  QueryMap service_buffered_query_map_;
  using upgrade_mutex = boost::upgrade_mutex;
  using upgrade_lock = boost::upgrade_lock<upgrade_mutex>;
  using upgrade_to_unique_lock = boost::upgrade_to_unique_lock<upgrade_mutex>;
  using unique_lock = boost::unique_lock<upgrade_mutex>;
  upgrade_mutex query_map_mutex_;
  const std::string& getFootprintMeshResource(const std::string& alt_mesh);
  double getFootprintPadding(double alt_padding);
  // assumes costmap is locked
  std::shared_ptr<Costmap3DQuery> getQuery(
      const std::string& footprint_mesh_resource,
      double padding,
      Costmap3DQuery::QueryRegionScale query_region_scale = Costmap3DQuery::QueryRegionScale::Zero(),
      unsigned int pose_bins_per_meter = 4,
      unsigned int pose_bins_per_radian = 4,
      unsigned int pose_milli_bins_per_meter = 20,
      unsigned int pose_milli_bins_per_radian = 20,
      unsigned int pose_micro_bins_per_meter = 1024,
      unsigned int pose_micro_bins_per_radian = 1024);
  void publishFootprint();
  void publishRegionOfInterest(std::shared_ptr<Costmap3DQuery> query, const geometry_msgs::PoseStamped& query_pose);

  std::string footprint_mesh_resource_;
  double footprint_3d_padding_;

};
// class Costmap3DROS
}  // namespace costmap_3d

#endif  // COSTMAP_3D_COSTMAP_3D_ROS_H_
