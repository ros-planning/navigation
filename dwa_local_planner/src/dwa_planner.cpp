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
#include <dwa_local_planner/dwa_planner.h>
#include <base_local_planner/goal_functions.h>
#include <cmath>

//for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>
#include <tf2/utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace dwa_local_planner {
  void DWAPlanner::reconfigure(DWAPlannerConfig &config)
  {

    boost::mutex::scoped_lock l(configuration_mutex_);

    generator_.setParameters(
        config.sim_time,
        config.sim_granularity,
        config.angular_sim_granularity,
        config.use_dwa,
        config.sim_period);
    sim_time_ = config.sim_time;
    sim_period_ = config.sim_period;

    // Configure the switches
    switch_yaw_error_ = config.switch_yaw_error;
    switch_goal_distance_ = config.switch_goal_distance;
    switch_plan_distance_ = config.switch_plan_distance;

    ROS_INFO_STREAM("Switches configured:\n"
                    << "    - Yaw error : " << config.switch_yaw_error << " [rad]\n"
                    << "    - Goal distance : " << config.switch_goal_distance << " [m]\n"
                    << "    - Plan distance : " << config.switch_plan_distance << " [m]\n");

    // Set not moving thresholds
    not_moving_distance_ = config.not_moving_distance;
    not_moving_time_window_ = config.not_moving_time_window;
    not_moving_minimal_duration_ = config.not_moving_minimal_duration;

    ROS_INFO_STREAM("notMoving configured:\n"
                    << "    - Not moving reset distance : " << config.not_moving_distance << " [m]\n"
                    << "    - Not moving evaluation time window : " << config.not_moving_time_window << " [s]\n"
                    << "    - Not moving minimal duration : " << config.not_moving_minimal_duration << " [s]\n");

    // Set scales
    align_align_scale_ = config.align_align_scale;
    align_plan_scale_ = config.align_plan_scale;
    align_goal_scale_ = config.align_goal_scale;
    align_cmd_scale_  = config.align_cmd_scale;

    default_align_scale_ = config.default_align_scale;
    default_plan_scale_ = config.default_plan_scale;
    default_goal_scale_ = config.default_goal_scale;
    default_cmd_scale_  = config.default_cmd_scale;

    arrive_align_scale_ = config.arrive_align_scale;
    arrive_plan_scale_ = config.arrive_plan_scale;
    arrive_goal_scale_ = config.arrive_goal_scale;
    arrive_cmd_scale_  = config.arrive_cmd_scale;


    ROS_INFO_STREAM("Scales configured:\n"
                    << "    - Align:\n"
                    << "        - align scale : " << config.align_align_scale << " [-]\n"
                    << "        - plan scale : " << config.align_plan_scale << " [-]\n"
                    << "        - goal scale : " << config.align_goal_scale << " [-]\n"
                    << "        - obstacle scale : " << config.align_obstacle_scale << " [-]\n"
                    << "    - Default:\n"
                    << "        - align scale : " << config.default_align_scale << " [-]\n"
                    << "        - plan scale : " << config.default_plan_scale << " [-]\n"
                    << "        - goal scale : " << config.default_goal_scale << " [-]\n"
                    << "        - obstacle scale : " << config.default_obstacle_scale << " [-]\n"
                    << "    - Arrive:\n"
                    << "        - align scale : " << config.arrive_align_scale << " [-]\n"
                    << "        - plan scale : " << config.arrive_plan_scale << " [-]\n"
                    << "        - goal scale : " << config.arrive_goal_scale << " [-]\n"
                    << "        - obstacle scale : " << config.arrive_obstacle_scale << " [-]\n"
                    );

    // Set cmd_vel costs
    align_cmd_px_  = config.align_cmd_px;
    align_cmd_nx_  = config.align_cmd_nx;
    align_cmd_py_  = config.align_cmd_py;
    align_cmd_ny_  = config.align_cmd_ny;
    align_cmd_pth_ = config.align_cmd_pth;
    align_cmd_nth_ = config.align_cmd_nth;

    default_cmd_px_  = config.default_cmd_px;
    default_cmd_nx_  = config.default_cmd_nx;
    default_cmd_py_  = config.default_cmd_py;
    default_cmd_ny_  = config.default_cmd_ny;
    default_cmd_pth_ = config.default_cmd_pth;
    default_cmd_nth_ = config.default_cmd_nth;

    arrive_cmd_px_  = config.arrive_cmd_px;
    arrive_cmd_nx_  = config.arrive_cmd_nx;
    arrive_cmd_py_  = config.arrive_cmd_py;
    arrive_cmd_ny_  = config.arrive_cmd_ny;
    arrive_cmd_pth_ = config.arrive_cmd_pth;
    arrive_cmd_nth_ = config.arrive_cmd_nth;

    // Set obstacle costs
    align_obstacle_scale_ = config.align_obstacle_scale;
    default_obstacle_scale_ = config.default_obstacle_scale;
    arrive_obstacle_scale_ = config.arrive_obstacle_scale;

    // obstacle costs can vary due to scaling footprint feature
    obstacle_costs_.setParams(config.max_vel_trans, config.max_scaling_factor, config.scaling_speed);

    // Sums scores by default
    obstacle_costs_.setSumScores(false);

    // Set parameters for occupancy velocity costfunction
    occ_vel_costs_.setParams(config.max_vel_trans);

    int vx_samp, vy_samp, vth_samp;
    vx_samp = config.vx_samples;
    vy_samp = config.vy_samples;
    vth_samp = config.vth_samples;

    if (vx_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
      vx_samp = 1;
      config.vx_samples = vx_samp;
    }

    if (vy_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
      vy_samp = 1;
      config.vy_samples = vy_samp;
    }

    if (vth_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to sample one value... so we're going to set vth_samples to 1 instead");
      vth_samp = 1;
      config.vth_samples = vth_samp;
    }

    vsamples_[0] = vx_samp;
    vsamples_[1] = vy_samp;
    vsamples_[2] = vth_samp;


  }

  DWAPlanner::DWAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util) :
      planner_util_(planner_util),
      occ_vel_costs_(planner_util->getCostmap()),
      plan_costs_(planner_util->getCostmap()),
      goal_costs_(planner_util->getCostmap()),
      obstacle_costs_(planner_util->getCostmap()),
      vis_(planner_util->getCostmap(), goal_costs_, plan_costs_, planner_util->getGlobalFrame())
  {
    // set up all the cost functions that will be applied in order
    // (any function returning negative values will abort scoring, so the order can improve performance)
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    critics.push_back(&goal_costs_);
    critics.push_back(&occ_vel_costs_);
    critics.push_back(&plan_costs_);
    critics.push_back(&alignment_costs_);
    critics.push_back(&cmd_vel_costs_);
    critics.push_back(&obstacle_costs_);

    // trajectory generators
    std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&generator_);

    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);
  }

  void DWAPlanner::updatePlanAndLocalCosts(
      const geometry_msgs::PoseStamped& robot_pose,
      const std::vector<geometry_msgs::PoseStamped>& local_plan,
      double lookahead,
      const std::vector<geometry_msgs::Point>& footprint_spec) {

    obstacle_costs_.setFootprint(footprint_spec);

    // Determine the errors
    double yaw_error = base_local_planner::getGoalOrientationAngleDifference(robot_pose, tf2::getYaw(local_plan.front().pose.orientation));
    double plan_distance = base_local_planner::getGoalPositionDistance(robot_pose, local_plan.front().pose.position.x, local_plan.front().pose.position.y);
    double goal_distance = base_local_planner::getGoalPositionDistance(robot_pose, local_plan.back().pose.position.x, local_plan.back().pose.position.y);

    // Determine state of the controller
    LocalPlannerState state = determineState(robot_pose, yaw_error, plan_distance, goal_distance);

    // Update the cost functions depending on the state we are in
    switch (state)
    {
    case NotMoving:
      // Almost similar to Align (see below). Difference in the desired orientation
      alignment_costs_.setScale(align_align_scale_);
      plan_costs_.setScale(align_plan_scale_);
      goal_costs_.setScale(align_goal_scale_);

      alignment_costs_.setDesiredOrientation(tf2::getYaw(local_plan.back().pose.orientation));

      cmd_vel_costs_.setCoefficients(align_cmd_px_, align_cmd_nx_, align_cmd_py_, align_cmd_ny_, align_cmd_pth_, align_cmd_nth_);

      obstacle_costs_.setScale(align_obstacle_scale_);

      break;

    case Align:
      alignment_costs_.setScale(align_align_scale_);
      plan_costs_.setScale(align_plan_scale_);
      goal_costs_.setScale(align_goal_scale_);

      alignment_costs_.setDesiredOrientation(tf2::getYaw(local_plan.front().pose.orientation));

      cmd_vel_costs_.setCoefficients(align_cmd_px_, align_cmd_nx_, align_cmd_py_, align_cmd_ny_, align_cmd_pth_, align_cmd_nth_);

      obstacle_costs_.setScale(align_obstacle_scale_);

      break;

    case Default:
      alignment_costs_.setScale(default_align_scale_);
      plan_costs_.setScale(default_plan_scale_);
      goal_costs_.setScale(default_goal_scale_);

      alignment_costs_.setDesiredOrientation(tf2::getYaw(local_plan.front().pose.orientation));

      cmd_vel_costs_.setCoefficients(default_cmd_px_, default_cmd_nx_, default_cmd_py_, default_cmd_ny_, default_cmd_pth_, default_cmd_nth_);

      obstacle_costs_.setScale(default_obstacle_scale_);

      break;

    case Arrive:
      alignment_costs_.setScale(arrive_align_scale_);
      plan_costs_.setScale(arrive_plan_scale_);
      goal_costs_.setScale(arrive_goal_scale_);

      alignment_costs_.setDesiredOrientation(tf2::getYaw(local_plan.back().pose.orientation));

      cmd_vel_costs_.setCoefficients(arrive_cmd_px_, arrive_cmd_nx_, arrive_cmd_py_, arrive_cmd_ny_, arrive_cmd_pth_, arrive_cmd_nth_);

      obstacle_costs_.setScale(arrive_obstacle_scale_);

      break;
    }

    // Optimization data (Set local plan)
    std::vector<geometry_msgs::PoseStamped> local_plan_from_lookahead;
    base_local_planner::planFromLookahead(local_plan, lookahead, local_plan_from_lookahead);

    goal_costs_.setTargetPoses(local_plan_from_lookahead);
    plan_costs_.setTargetPoses(local_plan);

    // Update footprint if changed
    occ_vel_costs_.setFootprint(footprint_spec);
  }


  /*
   * given the current state of the robot, find a good trajectory
   */
  base_local_planner::Trajectory DWAPlanner::findBestPath(
      const geometry_msgs::PoseStamped& global_pose,
      const geometry_msgs::PoseStamped& global_vel,
      const geometry_msgs::PoseStamped& goal_pose) {

    //make sure that our configuration doesn't change mid-run
    boost::mutex::scoped_lock l(configuration_mutex_);

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    // prepare cost functions and generators for this run
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);

    base_local_planner::Trajectory result_traj;
    result_traj.cost_ = -7;
    // find best trajectory by sampling and scoring the samples
    std::vector<base_local_planner::Trajectory> all_explored;
    scored_sampling_planner_.findBestTrajectory(result_traj, &all_explored);

    vis_.publishDesiredOrientation(alignment_costs_.getDesiredOrientation(), global_pose);
    vis_.publishCostGrid();
    vis_.publishTrajectoryCloud(all_explored);

    return result_traj;
  }

  LocalPlannerState DWAPlanner::determineState(const geometry_msgs::PoseStamped& robot_pose, double yaw_error, double /*plan_distance*/, double goal_distance) {
    static LocalPlannerState prev_state = None;
    LocalPlannerState state = Default;

    // todo: optional path_distance state

    // ToDo: can we make this more generic???
    if (!isMoving(robot_pose)) // ToDo: prevent switching behavior???
      state = NotMoving;
    else if (goal_distance < switch_goal_distance_)
      state = Arrive;
    else if (fabs(yaw_error) > switch_yaw_error_ || ( prev_state == Align && fabs(yaw_error) > (switch_yaw_error_/2) ) )
      state = Align;
    else
      state = Default;

    // To print state
    if (prev_state != state)
    {
      ROS_INFO_STREAM("State = " << StateName[state]);
      prev_state = state;
    }

    return state;
  }

  bool DWAPlanner::isMoving(const geometry_msgs::PoseStamped& robot_pose) {
    static double x_saved = 0.0;
    static double y_saved = 0.0;
    static bool prev_is_moving = true;
    static ros::Time stamp_not_moving = ros::Time::now();  // Time stamp to indicate when the robot went to 'not moving'

    double x = robot_pose.pose.position.x;
    double y = robot_pose.pose.position.y;

    double dist_sq = (x_saved - x) * (x_saved - x) + (y_saved - y) * (y_saved - y);

    double time_since_move = (ros::Time::now() - stamp_last_motion_).toSec();
    double time_since_stop = (ros::Time::now() - stamp_not_moving).toSec();

    bool is_moving = false;

    // Check whether we're actually moving
    if (dist_sq > not_moving_distance_) {
      resetMotionStamp();
      x_saved = x;
      y_saved = y;
      is_moving = true;
    }

    if (!prev_is_moving && time_since_stop < not_moving_minimal_duration_) {
      // Configure the minimal 'notMoving' duration to avoid switching behavior
      return false;
    }
    else if (is_moving) {
      prev_is_moving = true;
      return true;
    }
    else if ( time_since_move > not_moving_time_window_) {
      if (prev_is_moving)
      {
        ROS_WARN("Robot has not moved significantly for more than %f seconds", not_moving_time_window_);
        stamp_not_moving = ros::Time::now();
        prev_is_moving = false;
      }
      return false;
    }
    else {
      return true;
    }
  }
};
