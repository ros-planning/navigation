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

#include <base_local_planner/local_planner_util.h>

#include <base_local_planner/goal_functions.h>

namespace base_local_planner {

void LocalPlannerUtil::initialize(std::string name,
    tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* costmap_ros) {
  if(!initialized_) {
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    name_ = name;

    ros::NodeHandle pn("~/" + getName());
    g_plan_pub_ = pn.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = pn.advertise<nav_msgs::Path>("local_plan", 1);

    ros::NodeHandle pn2("~/localPlannerLimits");
    dsrv_ = new dynamic_reconfigure::Server<LocalPlannerLimitsConfig>(pn2);

    dynamic_reconfigure::Server<LocalPlannerLimitsConfig>::CallbackType cb = boost::bind(&LocalPlannerUtil::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    initialized_ = true;
  }
  else{
    ROS_WARN("Planner utils have already been initialized, doing nothing.");
  }
}

void LocalPlannerUtil::reconfigureCB(LocalPlannerLimitsConfig &config, uint32_t level)
{
  if(setup_ && config.restore_defaults) {
    config = default_limits_;
    config.restore_defaults = false;
  }

  if(!setup_) {
    default_limits_ = config;
    setup_ = true;
  }
  boost::mutex::scoped_lock l(limits_configuration_mutex_);
  limits_ = LocalPlannerLimitsConfig(config);
}




bool LocalPlannerUtil::getRobotPose(tf::Stamped<tf::Pose>& global_pose) {
  if(!costmap_ros_->getRobotPose(global_pose)) {
    return false;
  }
  return true;
}


bool LocalPlannerUtil::getGoal(tf::Stamped<tf::Pose>& goal_pose) {
  //we assume the global goal is the last point in the global plan
  return base_local_planner::getGoalPose(*tf_,
        global_plan_,
        costmap_ros_->getGlobalFrameID(),
        goal_pose);
}

bool LocalPlannerUtil::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if(!initialized_){
    ROS_ERROR("Planner utils have not been initialized, please call initialize() first");
    return false;
  }

  boost::mutex odom_mutex_;

  //reset the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  return true;
}

bool LocalPlannerUtil::getLocalPlan(tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan) {
  //get the global plan in our frame
  if(!base_local_planner::transformGlobalPlan(
      *getTfListener(),
      global_plan_,
      *getCostmapRos(),
      getCostmapRos()->getGlobalFrameID(),
      transformed_plan)) {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }

  //now we'll prune the plan based on the position of the robot
  if(limits_.prune_plan) {
    base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);
  }
  return true;
}


void LocalPlannerUtil::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
  base_local_planner::publishPlan(path, l_plan_pub_);
}


void LocalPlannerUtil::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
  base_local_planner::publishPlan(path, g_plan_pub_);
}

} // namespace
