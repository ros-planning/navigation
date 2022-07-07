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
#include <tf2/utils.h>

#include <nav_core/parameter_magic.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_local_planner::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_local_planner {

  void DWAPlannerROS::reconfigureCB(DWAPlannerConfig &config, uint32_t level) {
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
      limits.min_vel_theta = config.min_vel_theta;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_lim_trans = config.acc_lim_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.spin_turn_tolerance = config.spin_turn_tolerance;
      limits.spin_turn_vel_theta = config.spin_turn_vel_theta;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.theta_stopped_vel = config.theta_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // update dwa specific configuration
      dp_->reconfigure(config);
  }

  DWAPlannerROS::DWAPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false) {

  }

  void DWAPlannerROS::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if (! isInitialized()) {

      ros::NodeHandle nh;
      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      vel_cmd_mode_pub_ = private_nh.advertise<std_msgs::UInt8>("vel_cmd_mode", 1);
      vel_cmd_marker_pub_ = private_nh.advertise<visualization_msgs::Marker>("vel_cmd_marker", 1);

      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      actuator_position_sub_ = nh.subscribe("actuator_position", 10, &DWAPlannerROS::actuator_position_callback, this);
      nomotion_update_client_ = nh.serviceClient<std_srvs::Empty>("request_nomotion_update");
      nomotion_update_timer_ = nh.createTimer(ros::Duration(1.0 / 10.0), &DWAPlannerROS::call_nomotion_update_callback, this);

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

      private_nh.param("rotate_target_distance", this->rotate_target_distance_, 1.0);
      private_nh.param("rotate_start_threshold_vel", this->rotate_start_threshold_vel_, 0.1);
      private_nh.param("rotate_start_threshold_angular_vel", this->rotate_start_threshold_angular_vel_, 0.05);
      
      private_nh.param("use_rotate_first_actuator_connect", this->use_rotate_first_actuator_connect_, false);
      private_nh.param("use_rotate_first_actuator_disconnect", this->use_rotate_first_actuator_disconnect_, true);
      private_nh.param("kpre", this->kpre_, 0.65);

      private_nh.param("latch_unlock_distance", this->latch_unlock_distance_, 1.0);

      this->is_actuator_connect_ = false;
      this->rotate_to_goal_ = false;
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
    geometry_msgs::PoseStamped new_goal = orig_global_plan.back();
    double x = new_goal.pose.position.x - current_pose_.pose.position.x;
    double y = new_goal.pose.position.y - current_pose_.pose.position.y;
    double d = std::sqrt(x * x + y * y);
    if (this->latch_unlock_distance_ < d)
    {
      //when we get a new plan, we also want to clear any latch we may have on goal tolerances
      latchedStopRotateController_.resetLatching();
    }
    if ((this->is_actuator_connect_ && this->use_rotate_first_actuator_connect_) ||
        (!this->is_actuator_connect_ && this->use_rotate_first_actuator_disconnect_))
    {
      geometry_msgs::PoseStamped robot_vel;
      odom_helper_.getRobotVel(robot_vel);
      if (std::abs(robot_vel.pose.position.x) < this->rotate_start_threshold_vel_ &&
          std::abs(tf2::getYaw(robot_vel.pose.orientation)) < this->rotate_start_threshold_angular_vel_)
      {
        this->rotate_to_goal_ = true;
      }
    }

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
      std_srvs::Empty empty_srvs;
      this->nomotion_update_client_.call(empty_srvs);
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

  double DWAPlannerROS::lowPassFilter(double kpre, double& pre_val, double cur_val) {
    if (kpre < 0.0 || kpre > 1.0) {
      kpre = kpre_default_;
    }
    double filtered_val = (1.0 - kpre) * cur_val + kpre * pre_val;
    pre_val = filtered_val;

    return filtered_val;
  }

  bool DWAPlannerROS::dwaComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist& cmd_vel) {
    // dynamic window sampling approach to get useful velocity commands
    if(! isInitialized()){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
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
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    // Low pass filter
    cmd_vel.angular.z = lowPassFilter(kpre_, pre_cmd_vel_angular_z, cmd_vel.angular.z);
    cmd_vel.linear.x  = lowPassFilter(kpre_, pre_cmd_vel_linear_x,  cmd_vel.linear.x);

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

    publishLocalPlan(local_plan);
    return true;
  }

  void DWAPlannerROS::updateRotateToGoal()
  {
    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    if (robot_vel.pose.position.x < -0.1)
    {
      double yaw = tf2::getYaw(current_pose_.pose.orientation);
      double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);

      bool is_rotate = dp_->checkTrajectory(Eigen::Vector3f(current_pose_.pose.position.x, current_pose_.pose.position.y, yaw),
        Eigen::Vector3f(robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw),
        Eigen::Vector3f(0.0, 0.0, M_PI));

      if (is_rotate)
      {
        this->rotate_to_goal_ = true;
      }
    }
  }

  bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
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

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

    this->updateRotateToGoal();

    vel_cmd_mode_marker_msg_.header.frame_id = "base_link";
    vel_cmd_mode_marker_msg_.header.stamp = ros::Time();
    vel_cmd_mode_marker_msg_.ns = "vel_cmd_mode";
    vel_cmd_mode_marker_msg_.id = 0;
    vel_cmd_mode_marker_msg_.type = visualization_msgs::Marker::SPHERE;
    vel_cmd_mode_marker_msg_.action = visualization_msgs::Marker::ADD;
    vel_cmd_mode_marker_msg_.pose.position.x = 0.0;
    vel_cmd_mode_marker_msg_.pose.position.y = 0.0;
    vel_cmd_mode_marker_msg_.pose.position.z = 0.0;
    vel_cmd_mode_marker_msg_.pose.orientation.x = 0.0;
    vel_cmd_mode_marker_msg_.pose.orientation.y = 0.0;
    vel_cmd_mode_marker_msg_.pose.orientation.z = 0.0;
    vel_cmd_mode_marker_msg_.pose.orientation.w = 1.0;
    vel_cmd_mode_marker_msg_.scale.x = 0.3;
    vel_cmd_mode_marker_msg_.scale.y = 0.3;
    vel_cmd_mode_marker_msg_.scale.z = 0.3;
    vel_cmd_mode_marker_msg_.color.a = 1.0;

    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_))
    {
      vel_cmd_mode_msg_.data = 1;
      vel_cmd_mode_marker_msg_.color.r = 1.0;
      vel_cmd_mode_marker_msg_.color.g = 0.0;
      vel_cmd_mode_marker_msg_.color.b = 0.0;
      vel_cmd_mode_pub_.publish(vel_cmd_mode_msg_);
      vel_cmd_marker_pub_.publish(vel_cmd_mode_marker_msg_);
      
      this->is_force_update_ = true;
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
    }
    else if (this->rotate_to_goal_)
    {
      vel_cmd_mode_msg_.data = 0;
      vel_cmd_mode_marker_msg_.color.r = 0.0;
      vel_cmd_mode_marker_msg_.color.g = 1.0;
      vel_cmd_mode_marker_msg_.color.b = 0.0;
      vel_cmd_mode_pub_.publish(vel_cmd_mode_msg_);
      vel_cmd_marker_pub_.publish(vel_cmd_mode_marker_msg_);

      geometry_msgs::PoseStamped robot_vel;
      odom_helper_.getRobotVel(robot_vel);
      geometry_msgs::PoseStamped turn_target_pose;

      double current_x = current_pose_.pose.position.x;
      double current_y = current_pose_.pose.position.y;

      // calc closest waypoint
      auto closest_waypoint_iter = transformed_plan.begin();
      double min_distance = std::numeric_limits<double>::max();
      for (auto iter = transformed_plan.begin(); iter != transformed_plan.end(); iter++)
      {
        double tmp_x = iter->pose.position.x;
        double tmp_y = iter->pose.position.y;
        double distance = std::sqrt(std::pow(current_x - tmp_x, 2) + std::pow(current_y - tmp_y, 2));

        if (distance < min_distance)
        {
          min_distance = distance;
          closest_waypoint_iter = iter;
        }
      }

      // transformed_plan is not empty
      for (auto iter = closest_waypoint_iter; iter != transformed_plan.end(); iter++)
      {
        turn_target_pose = *iter;
        double tmp_x = iter->pose.position.x;
        double tmp_y = iter->pose.position.y;
        double distance = std::sqrt(std::pow(current_x - tmp_x, 2) + std::pow(current_y - tmp_y, 2));
        if (this->rotate_target_distance_ < distance)
        {
          break;
        }
      }
      
      double turn_turget_x = turn_target_pose.pose.position.x;
      double turn_turget_y = turn_target_pose.pose.position.y;

      double turn_turget_th = std::atan2(turn_turget_y - current_y, turn_turget_x - current_x);
      base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();

      bool was_rotate = latchedStopRotateController_.rotateToGoal(
        current_pose_,
        robot_vel,
        turn_turget_th,
        cmd_vel,
        limits.getAccLimits(),
        dp_->getSimPeriod(),
        limits,
        boost::bind(&DWAPlanner::checkTrajectory, dp_, _1, _2, _3));

      double current_th = tf2::getYaw(this->current_pose_.pose.orientation);
      if (!was_rotate || std::abs(current_th - turn_turget_th) < (5.0 * M_PI / 180.0))
      {
        this->rotate_to_goal_ = false;
      }

      return true;
    }
    else
    {
      vel_cmd_mode_msg_.data = 2;
      vel_cmd_mode_marker_msg_.color.r = 0.0;
      vel_cmd_mode_marker_msg_.color.g = 0.0;
      vel_cmd_mode_marker_msg_.color.b = 1.0;
      vel_cmd_mode_pub_.publish(vel_cmd_mode_msg_);
      vel_cmd_marker_pub_.publish(vel_cmd_mode_marker_msg_);
      bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
      if (isOk) {
        publishGlobalPlan(transformed_plan);
        this->latchedStopRotateController_.resetLatching();
      } else {
        ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }
      return isOk;
    }
  }

  void DWAPlannerROS::actuator_position_callback(const lexxauto_msgs::ActuatorStatus::ConstPtr& msg)
  {
    this->is_actuator_connect_ = msg->connect;
  }

  void DWAPlannerROS::call_nomotion_update_callback(const ros::TimerEvent& event)
  {
    if (this->is_force_update_)
    {
      std_srvs::Empty empty_srvs;
      this->nomotion_update_client_.call(empty_srvs);
      this->is_force_update_ = false;
    }
  }
};
