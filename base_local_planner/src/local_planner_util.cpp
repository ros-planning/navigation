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

void LocalPlannerUtil::initialize(
    tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* costmap,
    std::string global_frame, std::string controller_name) {


      
  if(!initialized_) {
    tf_ = tf;
    costmap_ = costmap;
    global_frame_ = global_frame;
    speed_limit_manager_.initialize(costmap, controller_name);
    initialized_ = true;
  }
  else{
    ROS_WARN("Planner utils have already been initialized, doing nothing.");
  }
}

void LocalPlannerUtil::reconfigureCB(LocalPlannerLimits &config, bool restore_defaults)
{
  if(setup_ && restore_defaults) {
    config = default_limits_;
  }

  if(!setup_) {
    default_limits_ = config;
    setup_ = true;
  }
  {
    boost::mutex::scoped_lock l(limits_configuration_mutex_);
    nominal_limits_ = LocalPlannerLimits(config);
    ROS_INFO("Nominal limits tolerance: %f", nominal_limits_.xy_goal_tolerance);
  }
  //updateLimits();
}

costmap_2d::Costmap2D* LocalPlannerUtil::getCostmap() {
  return costmap_->getCostmap();
}

LocalPlannerLimits LocalPlannerUtil::getCurrentLimits() {
  boost::mutex::scoped_lock l(limits_configuration_mutex_);
  return active_limits_;
}


bool LocalPlannerUtil::getGoal(tf::Stamped<tf::Pose>& goal_pose) {
  //we assume the global goal is the last point in the global plan
  return base_local_planner::getGoalPose(*tf_,
        global_plan_,
        global_frame_,
        goal_pose);
}

bool LocalPlannerUtil::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if(!initialized_){
    ROS_ERROR("Planner utils have not been initialized, please call initialize() first");
    return false;
  }

  //reset the global plan
  global_plan_.clear();
  ROS_DEBUG_NAMED("local_planner_util", "Setting global plan with length %d", orig_global_plan.size());
  global_plan_ = orig_global_plan;

  return true;
}

double LocalPlannerUtil::distanceToPlanDivergence(const std::vector<geometry_msgs::PoseStamped>& new_plan)
{
  double EPSILON = 0.01;
  double distance = 0;
  size_t min_samples = std::min(new_plan.size(), global_plan_.size());
  bool foundDivergence = false;
  for (size_t k = 0; k < min_samples; ++k)
  {
    geometry_msgs::PoseStamped new_plan_pose = new_plan[k];
    geometry_msgs::PoseStamped global_plan_pose = global_plan_[k];

    if (std::fabs(new_plan_pose.pose.position.x - global_plan_pose.pose.position.x) > EPSILON
      || std::fabs(new_plan_pose.pose.position.y - global_plan_pose.pose.position.y) > EPSILON)
      {
        foundDivergence = true;
        break;
      }

    if (k > 0)
    {
      geometry_msgs::PoseStamped new_plan_pose_prev = new_plan[k - 1];
      double dx = new_plan_pose.pose.position.x - new_plan_pose_prev.pose.position.x;
      double dy = new_plan_pose.pose.position.y - new_plan_pose_prev.pose.position.y;
      distance += std::sqrt(dx * dx + dy * dy);
    }
  }
  if (!foundDivergence)
  {
    distance = -1.0;
  }
  return distance;
}

bool LocalPlannerUtil::getLocalPlan(tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan) {
  //get the global plan in our frame
  if(!base_local_planner::transformGlobalPlan(
      *tf_,
      global_plan_,
      global_pose,
      *(costmap_->getCostmap()),
      global_frame_,
      transformed_plan)) {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }

  //now we'll prune the plan based on the position of the robot
  if(active_limits_.prune_plan) {
    base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);
  }
  return true;
}


void LocalPlannerUtil::updateLimits() {

  tf::Stamped<tf::Pose> robot_pose;
  if (!costmap_->getRobotPose(robot_pose)) {
    ROS_WARN("Could not get robot pose to calculate speed limits");
  }

  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  if (!getLocalPlan(robot_pose, transformed_plan)) {
    ROS_WARN("No plan for speed limiters.");
  }
  speed_limit_manager_.setPlan(transformed_plan);

  double v_lim = 0, w_lim = 0;
  speed_limit_manager_.calculateLimits(v_lim, w_lim);


  boost::mutex::scoped_lock l(limits_configuration_mutex_);
  active_limits_ = nominal_limits_;

  // Make sure the limits are respected
  if (active_limits_.max_rot_vel > w_lim) {
    active_limits_.max_rot_vel = w_lim;
  }

  if (active_limits_.min_rot_vel > w_lim) {
    active_limits_.min_rot_vel = w_lim;
  }

  if (active_limits_.max_tip_vel > w_lim) {
    active_limits_.max_tip_vel = w_lim;
  }

  if (active_limits_.min_tip_vel > w_lim) {
    active_limits_.min_tip_vel = w_lim;
  }

  if (active_limits_.max_trans_vel > v_lim) {
    active_limits_.max_trans_vel = v_lim;
  }

  if (active_limits_.min_trans_vel > v_lim) {
    active_limits_.min_trans_vel = v_lim;
  }

  if (active_limits_.max_vel_x > v_lim) {
    active_limits_.max_vel_x = v_lim;
  }

  if (active_limits_.min_vel_x > v_lim) {
    active_limits_.min_vel_x = v_lim;
  }
}

} // namespace
