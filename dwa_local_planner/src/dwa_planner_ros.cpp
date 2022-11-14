/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#include <dwa_local_planner/dwa_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>
#include <numeric>

#include <nav_core/parameter_magic.h>

#include <mbf_msgs/ExePathAction.h>

// register this planner as a MBF's CostmapController plugin
PLUGINLIB_EXPORT_CLASS(dwa_local_planner::DWAPlannerROS, mbf_costmap_core::CostmapController)

namespace dwa_local_planner {

  void DWAPlannerROS::reconfigureCB(DWAPlannerConfig &config, uint32_t level) {
      const std::lock_guard<std::mutex> lock(config_mtx_);
      
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
      }

      // update generic local planner params
      base_local_planner::LocalPlannerLimits limits;
      limits.max_vel_trans = config.max_vel_trans;
      limits.min_vel_trans = config.min_vel_trans;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_vel_theta = config.max_vel_theta;
      limits.max_vel_theta_spin = config.max_vel_theta_spin;
      limits.min_vel_theta = config.min_vel_theta;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_lim_trans = config.acc_lim_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.inner_xy_goal_tolerance = config.inner_xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.theta_stopped_vel = config.theta_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // update dwa specific configuration
      dp_->reconfigure(config);
  }

  DWAPlannerROS::DWAPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false), prev_vel_dir_(0), oscillating_(false), latched_inner_goal_(false) {

  }

  void DWAPlannerROS::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if (! isInitialized()) {

      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      scaled_fp_pub_ = private_nh.advertise<visualization_msgs::Marker>("scaled_footprint", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }

      initialized_ = true;

      // Warn about deprecated parameters -- remove this block in N-turtle
      nav_core::warnRenamedParameter(private_nh, "max_vel_trans", "max_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_trans", "min_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "max_vel_theta", "max_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_theta", "min_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "acc_lim_trans", "acc_limit_trans");
      nav_core::warnRenamedParameter(private_nh, "theta_stopped_vel", "rot_stopped_vel");

      dsrv_ = new dynamic_reconfigure::Server<DWAPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb = boost::bind(&DWAPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }

  bool DWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    if (orig_global_plan.empty() || orig_global_plan.back().pose != current_goal_.pose
            || orig_global_plan.back().header.frame_id != current_goal_.header.frame_id) {
      // reset latching only if the goal changed
      latchedStopRotateController_.resetLatching();
      resetBestEffort();
    }

    current_goal_ = !orig_global_plan.empty() ? orig_global_plan.back() : geometry_msgs::PoseStamped();

    ROS_INFO("Got new plan");
    return dp_->setPlan(orig_global_plan);
  }

  bool DWAPlannerROS::isGoalReached(double, double) {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    const bool reached_outer_goal = latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_);
    if(reached_outer_goal && finishedBestEffort()) {
      ROS_INFO("Goal reached");
      // reset latching such that the latching doesn't apply even if the same goal is targeted again
      latchedStopRotateController_.resetLatching();
      resetBestEffort();
      return true;
    } else {
      return false;
    }
  }

  void DWAPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }


  void DWAPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  void DWAPlannerROS::publishScaledFootprint(const geometry_msgs::PoseStamped& pose, const base_local_planner::Trajectory &traj) const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration(2 * dp_->getSimPeriod()); // double the sim period to avoid flickering
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose = pose.pose;
    marker.scale.x = 0.01;
    marker.color.g = marker.color.a = 1.0; // green
    marker.points = dp_->getScaledFootprint(traj);
    marker.points.push_back(marker.points.front()); // close the polygon
    scaled_fp_pub_.publish(marker);
  }

  DWAPlannerROS::~DWAPlannerROS(){
    //make sure to clean things up
    delete dsrv_;
  }

  void DWAPlannerROS::resetBestEffort() {
    latched_inner_goal_ = false;
    prev_vel_dir_ = 0;
    oscillating_ = false;
    outer_goal_entry_.reset();
  }

  bool DWAPlannerROS::finishedBestEffort() {
    if (transformed_plan_.empty() || !outer_goal_entry_) {
      return false;
    }

    // latch inner tolerance
    const double goal_x = transformed_plan_.back().pose.position.x;
    const double goal_y = transformed_plan_.back().pose.position.y;
    const double inner_xy_goal_tolerance = planner_util_.getCurrentLimits().inner_xy_goal_tolerance;
    const double goal_dist = base_local_planner::getGoalPositionDistance(current_pose_, goal_x, goal_y);
    latched_inner_goal_ = latched_inner_goal_ || goal_dist <= inner_xy_goal_tolerance;

    // check if bypassed goal
    const std::vector<double> v1 = {
      goal_x - outer_goal_entry_->pose.position.x,
      goal_y - outer_goal_entry_->pose.position.y
    };
    const std::vector<double> v2 = {
      goal_x - current_pose_.pose.position.x,
      goal_y - current_pose_.pose.position.y
    };
    const bool bypassed_goal = std::inner_product(v1.begin(), v1.end(), v2.begin(), 0.0) < 0;

    return latched_inner_goal_ ||  bypassed_goal || oscillating_;
  }

  uint32_t DWAPlannerROS::dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose,
                                                     geometry_msgs::TwistStamped& cmd_vel, std::string& message) {
    // dynamic window sampling approach to get useful velocity commands
    if(! isInitialized()){
      message = "This planner has not been initialized, please call initialize() before using this planner";
      ROS_ERROR_STREAM_NAMED("dwa_local_planner", message);
      return mbf_msgs::ExePathResult::NOT_INITIALIZED;
    }

    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //compute what trajectory to drive along
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();

    // call with updated footprint
    std::lock_guard<std::mutex> lock(config_mtx_);
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
    //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.twist.linear.x = drive_cmds.pose.position.x;
    cmd_vel.twist.linear.y = drive_cmds.pose.position.y;
    cmd_vel.twist.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if(path.cost_ < 0) {
      message = "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot";
      ROS_DEBUG_STREAM_NAMED("dwa_local_planner", message);  //// TODO why is this DEBUG?
      local_plan.clear();
      publishLocalPlan(local_plan);
      return mbf_msgs::ExePathResult::NO_VALID_CMD;
    }

    ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
                    cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
    cmd_vel.header.stamp = ros::Time::now();

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      geometry_msgs::PoseStamped p;
      p.header.frame_id = costmap_ros_->getGlobalFrameID();
      p.header.stamp = ros::Time::now();
      p.pose.position.x = p_x;
      p.pose.position.y = p_y;
      p.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, p_th);
      tf2::convert(q, p.pose.orientation);
      local_plan.push_back(p);
    }

    //publish information to the visualizer
    publishScaledFootprint(global_pose, path);
    publishLocalPlan(local_plan);
    return mbf_msgs::ExePathResult::SUCCESS;
  }

  uint32_t DWAPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                  const geometry_msgs::TwistStamped& velocity,
                                                  geometry_msgs::TwistStamped& cmd_vel, std::string& message) {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      message = "Could not get robot pose";
      ROS_ERROR_STREAM_NAMED("dwa_local_planner", message);
      return mbf_msgs::ExePathResult::TF_ERROR;
    }

    if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan_)) {
      message = "Could not get local plan";
      ROS_ERROR_STREAM_NAMED("dwa_local_planner", message);
      return mbf_msgs::ExePathResult::TF_ERROR;
    }

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan_.empty()) {
      message = "Received an empty transformed plan";
      ROS_ERROR_STREAM_NAMED("dwa_local_planner", message);
      return mbf_msgs::ExePathResult::INVALID_PATH;
    }
    ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan_.size());

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan_, costmap_ros_->getRobotFootprint());

    // check if we reached outer tolerance
    const bool reached_outer_goal = latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_);
    if (reached_outer_goal) {
      if (!outer_goal_entry_) {
        outer_goal_entry_ = current_pose_;
      }
      
      // check if we reached inner tolerance
      if (finishedBestEffort()) {
        //publish an empty plan because we've reached our goal position
        std::vector<geometry_msgs::PoseStamped> local_plan;
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        publishGlobalPlan(transformed_plan);
        publishLocalPlan(local_plan);
        base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
        if (latchedStopRotateController_.computeVelocityCommandsStopRotate(
                cmd_vel.twist,
                limits.getAccLimits(),
                dp_->getSimPeriod(),
                &planner_util_,
                odom_helper_,
                current_pose_,
                boost::bind(&DWAPlanner::checkTrajectory, dp_, _1, _2, _3))) {
          cmd_vel.header.stamp = ros::Time::now();
          return mbf_msgs::ExePathResult::SUCCESS;
        }
        else {
            // reset latching since DWA planner can move forward / backwards
            latchedStopRotateController_.resetLatching();
            resetBestEffort();
            ROS_INFO_NAMED("dwa_local_planner", "can't rotate in place; fall back to DWA planner");
        }
      }
    }
    else {
      resetBestEffort();
    }

    uint32_t result = dwaComputeVelocityCommands(current_pose_, cmd_vel, message);

    // check for oscillations while approaching inner tolerance
    if (reached_outer_goal) {
      const int vel_dir = std::copysign(1, cmd_vel.twist.linear.x);
      oscillating_ = oscillating_
        || (prev_vel_dir_ != 0 && prev_vel_dir_ != vel_dir)
        || (cmd_vel.twist.linear.x == 0 && cmd_vel.twist.angular.z != 0);
      prev_vel_dir_ = vel_dir;
    }

    if (result == mbf_msgs::ExePathResult::SUCCESS) {
      publishGlobalPlan(transformed_plan_);
    } else {
      ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
      std::vector<geometry_msgs::PoseStamped> empty_plan;
      publishGlobalPlan(empty_plan);
    }
   return result;

  }


};
