/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#ifndef ABSTRACT_LOCAL_PLANNER_ODOM_H_
#define ABSTRACT_LOCAL_PLANNER_ODOM_H_

#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <boost/thread.hpp>

#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <base_local_planner/LocalPlannerLimitsConfig.h>


namespace base_local_planner {

/**
 * @class AbstractLocalPlannerUtil
 * @brief Helper class implementing infrastructure code many local planner implementations may need.
 * Maintains odometry information from /odom topic.
 */
class LocalPlannerUtil {

private:
  // things we get from move_base
  std::string name_;

  costmap_2d::Costmap2DROS* costmap_ros_;
  tf::TransformListener* tf_;


  std::vector<geometry_msgs::PoseStamped> global_plan_;
  // for visualisation, publishers of global and local plan
  ros::Publisher g_plan_pub_, l_plan_pub_;


  dynamic_reconfigure::Server<LocalPlannerLimitsConfig> *dsrv_;
  boost::mutex limits_configuration_mutex_;
  bool setup_;
  LocalPlannerLimitsConfig default_limits_;
  LocalPlannerLimitsConfig limits_;
  bool initialized_;

public:

  /**
   * @brief  Callback to update the local planner's parameters based on dynamic reconfigure
   */
  void reconfigureCB(LocalPlannerLimitsConfig &config, uint32_t level);

  LocalPlannerUtil() : initialized_(false) {}

  /**
   * @brief  Destructs a trajectory controller
   */
  ~LocalPlannerUtil() {
    if (initialized_) {
      delete(dsrv_);
    }
  }

  void initialize(std::string name, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros);

  bool getRobotPose(tf::Stamped<tf::Pose>& global_pose);

  bool getGoal(tf::Stamped<tf::Pose>& goal_pose);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  bool getLocalPlan(tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan);

  void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

  void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

  costmap_2d::Costmap2DROS* getCostmapRos() {
    return costmap_ros_;
  }

  tf::TransformListener* getTfListener() {
    return tf_;
  }

  std::string getName() {
    return name_;
  }


  LocalPlannerLimitsConfig getCurrentLimits() {
    return limits_;
  }
};




};

#endif /* ABSTRACT_LOCAL_PLANNER_ODOM_H_ */
