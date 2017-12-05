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

#include <srslib_timing/ScopedTimingSampleRecorder.hpp>
#include <srslib_timing/ScopedRollingTimingStatistics.hpp>


//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_local_planner::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_local_planner {

  void DWAPlannerROS::reconfigureCB(DWAPlannerModeConfig &main_config, uint32_t level) {
      if (setup_ && main_config.restore_defaults) {
        main_config = default_config_;
        main_config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = main_config;
        setup_ = true;
      }

      // Check the mode
      int mode = main_config.mode;

      if (mode >= mode_configurations_.size())
      {
        ROS_WARN("Could not find mode %d in list of configurations", mode);
        return;
      }
      ROS_INFO("Switching dwa planner to mode %d", mode);

      DWAPlannerConfig config = mode_configurations_[mode]->getConfig();

      // update generic local planner params
      base_local_planner::LocalPlannerLimits limits;
      limits.max_trans_vel = config.max_trans_vel;
      limits.min_trans_vel = config.min_trans_vel;
      limits.max_vel_x = std::min(main_config.max_vel_x, config.max_vel_x);
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_rot_vel = config.max_rot_vel;
      limits.min_rot_vel = config.min_rot_vel;
      limits.max_tip_vel = config.max_tip_vel;
      limits.min_tip_vel = config.min_tip_vel;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_limit_trans = config.acc_limit_trans;
      limits.acc_limit_tip = config.acc_limit_tip;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.rot_stopped_vel = config.rot_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      base_local_planner::SpeedLimiterParams sp_params;
      sp_params.max_linear_velocity_ = config.max_vel_x;
      sp_params.nominal_linear_velocity_ = config.obstacle_speed_limit_nominal_linear_velocity;
      sp_params.min_linear_velocity_ = config.obstacle_speed_limit_min_linear_velocity;
      sp_params.max_range_ = config.obstacle_speed_limit_max_range;
      sp_params.nominal_range_max_ = config.obstacle_speed_limit_nominal_range_max;
      sp_params.nominal_range_min_ = config.obstacle_speed_limit_nominal_range_min;
      sp_params.min_range_ = config.obstacle_speed_limit_min_range;
      sp_params.x_buffer_ = config.obstacle_speed_limit_x_buffer;
      sp_params.y_buffer_ = config.obstacle_speed_limit_y_buffer;
      sp_params.half_angle_ = config.obstacle_speed_limit_half_angle;

      sp_params.min_angular_velocity_effect_distance_ = config.obstacle_speed_limit_min_angular_vel_effect_dist;
      sp_params.max_angular_velocity_effect_distance_ = config.obstacle_speed_limit_max_angular_vel_effect_dist;
      sp_params.min_angular_velocity_ = config.obstacle_speed_limit_min_angular_velocity;
      sp_params.max_angular_velocity_ = config.max_rot_vel;
      planner_util_.setSpeedLimiterParams(sp_params);

      base_local_planner::ShadowSpeedLimiterParams sh_params;
      sh_params.max_linear_velocity_ = config.max_vel_x;
      sh_params.nominal_linear_velocity_ = config.shadow_speed_limit_nominal_linear_velocity;      
      sh_params.min_linear_velocity_ = config.shadow_speed_limit_min_linear_velocity;
      sh_params.max_range_ = config.shadow_speed_limit_max_range;
      sh_params.nominal_range_max_ = config.shadow_speed_limit_nominal_range_max;
      sh_params.nominal_range_min_ = config.shadow_speed_limit_nominal_range_min;
      sh_params.min_range_ = config.shadow_speed_limit_min_range;
      sh_params.forward_offset_ = config.shadow_speed_limit_forward_offset;
      planner_util_.setShadowSpeedLimiterParams(sh_params);


      base_local_planner::PathSpeedLimiterParams path_params;
      path_params.max_linear_velocity_ = config.max_vel_x;
      path_params.min_linear_velocity_ = config.path_speed_limit_min_linear_velocity;
      path_params.min_lookahead_distance_ = config.path_speed_limit_min_lookahead_distance;
      path_params.max_lookahead_distance_ = config.path_speed_limit_max_lookahead_distance;
      path_params.max_heading_difference_ = config.path_speed_limit_max_heading_difference;
      path_params.min_heading_difference_ = config.path_speed_limit_min_heading_difference;
      path_params.max_angular_velocity_ = config.max_rot_vel;
      planner_util_.setPathSpeedLimiterParams(path_params);


      odom_helper_.setAccelerationRates(config.acc_lim_x, config.acc_lim_theta);
      // odom_helper_.setWheelbase(config.wheelbase);

      // update dwa specific configuration
      dp_->reconfigure(config);
  }

  DWAPlannerROS::DWAPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false),
      tdr_("DWAPlannerROS")
      {

  }

  void DWAPlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if (! isInitialized()) {

      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      planner_util_.initialize(tf, costmap_ros_, costmap_ros_->getGlobalFrameID());

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));

      // set up footprint when initialization
      dp_->setFootprintSpec(costmap_ros->getRobotFootprint());

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }

      /////// Set up the different modes.
      if (!private_nh.hasParam("modes"))
      {
        ROS_ERROR("DWAPlannerROS cannot be initialized because no modes were provided.");
        return;
      }

      std::vector<std::string> mode_names;
      private_nh.getParam("modes", mode_names);
      for (auto mode_name : mode_names)
      {
        // Create a config object and store it in the map.
        auto configuration = std::make_shared<DWAPlannerConfiguration>(name + "/" + mode_name);
        mode_configurations_.push_back(configuration);
      }
      /////// End mode setup.

      initialized_ = true;

      dsrv_ = new dynamic_reconfigure::Server<DWAPlannerModeConfig>(private_nh);
      dynamic_reconfigure::Server<DWAPlannerModeConfig>::CallbackType cb = boost::bind(&DWAPlannerROS::reconfigureCB, this, _1, _2);
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
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();

    ROS_INFO("Got new plan");
    return dp_->setPlan(orig_global_plan);
  }

  bool DWAPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      ROS_INFO("Goal reached");
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

  DWAPlannerROS::~DWAPlannerROS(){
    //make sure to clean things up
    delete dsrv_;
  }

  bool DWAPlannerROS::dwaComputeVelocityCommands(tf::Stamped<tf::Pose> &global_pose, tf::Stamped<tf::Pose>& robot_vel,
    geometry_msgs::Twist& cmd_vel) {
    // dynamic window sampling approach to get useful velocity commands
    if(! isInitialized()){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //compute what trajectory to drive along
    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = costmap_ros_->getBaseFrameID();

    // call with updated footprint
    srs::ScopedTimingSampleRecorder stsr(tdr_.getRecorder("-PlanCall"));
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
    stsr.stopSample();

    //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

    odom_helper_.setCmdVel(cmd_vel);

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if(path.cost_ < 0) {
      ROS_DEBUG_NAMED("dwa_local_planner",
          "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      publishLocalPlan(local_plan);

      return false;
    }

    ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      tf::Stamped<tf::Pose> p =
              tf::Stamped<tf::Pose>(tf::Pose(
                      tf::createQuaternionFromYaw(p_th),
                      tf::Point(p_x, p_y, 0.0)),
                      ros::Time::now(),
                      costmap_ros_->getGlobalFrameID());
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    //publish information to the visualizer

    publishLocalPlan(local_plan);
    return true;
  }

  bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
    if (loopTimingStatistics_.isValid())
    {
      odom_helper_.setForwardEstimationTime(loopTimingStatistics_.getMedian());
      ROS_DEBUG("Loop timing stats:  mean: %f, min: %f, median: %f, max: %f",
        loopTimingStatistics_.getMean(),loopTimingStatistics_.getMin(),
        loopTimingStatistics_.getMedian(),loopTimingStatistics_.getMax());
    }
    else
    {
      odom_helper_.setForwardEstimationTime(0.0);
    }
    odom_helper_.getEstimatedOdomPose(current_pose_);

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
      ROS_ERROR("Could not get local plan");
      return false;
    }
    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) {
      ROS_WARN_NAMED("dwa_local_planner", "Received an empty transformed plan.");
      return false;
    }
    ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getEstimatedRobotVel(robot_vel);

    // Update the speed limits
    planner_util_.updateLimits();

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    srs::ScopedTimingSampleRecorder stsr(tdr_.getRecorder("-UpdatePlanCosts"));
    dp_->updatePlanAndLocalCosts(current_pose_, robot_vel, transformed_plan);
    stsr.stopSample();

    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
      //publish an empty plan because we've reached our goal position
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      publishGlobalPlan(transformed_plan);
      publishLocalPlan(local_plan);
      base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      return latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,
          limits.getAccLimits(),
          dp_->getSimPeriod(),
          &planner_util_,
          odom_helper_,
          current_pose_,
          boost::bind(&DWAPlanner::checkTrajectory, dp_, _1, _2, _3));
    } else {
      srs::ScopedRollingTimingStatistics scoped_timer(&loopTimingStatistics_);

      srs::ScopedTimingSampleRecorder stsr2(tdr_.getRecorder("-dwaComputeVelCommands"));
      bool isOk = dwaComputeVelocityCommands(current_pose_, robot_vel, cmd_vel);
      stsr2.stopSample();
      if (isOk) {
        publishGlobalPlan(transformed_plan);
      } else {
        ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }
      return isOk;
    }
  }
};
