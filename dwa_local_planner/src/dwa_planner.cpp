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
#include <cmath>

//for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>

namespace dwa_local_planner {
  void DWAPlanner::reconfigureCB(DWAPlannerConfig &config, uint32_t level)
  {
    if (setup_ && config.restore_defaults) {
      config = default_config_;
      config.restore_defaults = false;
    }

    if ( ! setup_) {
      default_config_ = config;
      setup_ = true;
    }
    boost::mutex::scoped_lock l(configuration_mutex_);
 
 
    sim_time_ = config.sim_time;
    sim_granularity_ = config.sim_granularity;
    generator_.setParameters(sim_time_, sim_granularity_);

    double resolution = costmap_.getResolution();
    pdist_scale_ = config.path_distance_bias;
    path_costs_.setScale(resolution * pdist_scale_ * 0.5);
    alignment_costs_.setScale(resolution * pdist_scale_ * 0.5);

    gdist_scale_ = config.goal_distance_bias;
    goal_costs_.setScale(resolution * gdist_scale_ * 0.5);
    goal_front_costs_.setScale(resolution * gdist_scale_ * 0.5);

    occdist_scale_ = config.occdist_scale;
    obstacle_costs_.setScale(resolution * occdist_scale_);
//    alignment_costs_.setScale(resolution * adist_scale_);//TODO

    stop_time_buffer_ = config.stop_time_buffer;
    oscillation_costs_.setOscillationResetDist(config.oscillation_reset_dist);
    forward_point_distance_ = config.forward_point_distance;
    goal_front_costs_.setXShift(forward_point_distance_);
    alignment_costs_.setXShift(forward_point_distance_);
 
    scaling_speed_ = config.scaling_speed;
    max_scaling_factor_ = config.max_scaling_factor;

    backward_motion_penalty_ = 100.0; // TODO: Make dynamic parameter
 
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
 
    penalize_negative_x_ = config.penalize_negative_x;
  }

  // used for visualization only, total_costs are not really total costs
  bool DWAPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {

    path_cost = path_costs_.getCellCosts(cx, cy);
    goal_cost = goal_costs_.getCellCosts(cx, cy);
    occ_cost = costmap_.getCost(cx, cy);
    if (path_cost == path_costs_.obstacleCosts() ||
        path_cost == path_costs_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }

    double resolution = costmap_.getResolution();
    total_cost =
        pdist_scale_ * resolution * path_cost +
        gdist_scale_ * resolution * goal_cost +
        occdist_scale_ * occ_cost;
    return true;
  }

  bool DWAPlanner::checkTrajectory(Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      const base_local_planner::LocalPlannerLimitsConfig& limits){
    oscillation_costs_.resetOscillationFlags();
    base_local_planner::Trajectory traj;
    generator_.generateTrajectory(pos, vel, &limits, sim_time_, sim_granularity_, traj);
    double cost = scored_sampling_planner_.scoreTrajectory(traj);
    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel[0], vel[1], vel[2], cost);

    //otherwise the check fails
    return false;
  }

  DWAPlanner::DWAPlanner(std::string name, const costmap_2d::Costmap2DROS* costmap_ros) :
      dsrv_(ros::NodeHandle("~/" + name)),
      setup_(false),
      penalize_negative_x_(true),
      obstacle_costs_(costmap_ros),
      prefer_forward_costs_(0.0),
      path_costs_(costmap_ros),
      goal_costs_(costmap_ros, 0.0, 0.0, true),
      goal_front_costs_(costmap_ros, 0.0, 0.0, true),
      alignment_costs_(costmap_ros)
  {
    costmap_ros_ = costmap_ros;
    costmap_ros_->getCostmapCopy(costmap_);

    ros::NodeHandle pn("~/" + name);

    double acc_lim_x, acc_lim_y, acc_lim_th;
    pn.param("acc_lim_x", acc_lim_x, 2.5);
    pn.param("acc_lim_y", acc_lim_y, 2.5);
    pn.param("acc_lim_th", acc_lim_th, 3.2);

    //Assuming this planner is being run within the navigation stack, we can
    //just do an upward search for the frequency at which its being run. This
    //also allows the frequency to be overwritten locally.
    std::string controller_frequency_param_name;
    if(!pn.searchParam("controller_frequency", controller_frequency_param_name)) {
      sim_period_ = 0.05;
    } else {
      double controller_frequency = 0;
      pn.param(controller_frequency_param_name, controller_frequency, 20.0);
      if(controller_frequency > 0) {
        sim_period_ = 1.0 / controller_frequency;
      } else {
        ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
        sim_period_ = 0.05;
      }
    }
    ROS_INFO("Sim period is set to %.2f", sim_period_);

    acc_lim_[0] = acc_lim_x;
    acc_lim_[1] = acc_lim_y;
    acc_lim_[2] = acc_lim_th;

    dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb = boost::bind(&DWAPlanner::reconfigureCB, this, _1, _2);
    dsrv_.setCallback(cb);


    oscillation_costs_.resetOscillationFlags();

    pn.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
    map_viz_.initialize(name, boost::bind(&DWAPlanner::getCellCosts, this, _1, _2, _3, _4, _5, _6));

    std::string frame_id;
    pn.param("global_frame_id", frame_id, std::string("odom"));

    traj_cloud_.header.frame_id = frame_id;
    traj_cloud_pub_.advertise(pn, "trajectory_cloud", 1);

    // set up all the cost functions that will be applied in order (any returning negative values will abort scoring)
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    critics.push_back(&oscillation_costs_); // discards oscillating motions (assisgns cost -1)
    critics.push_back(&obstacle_costs_); // discards trajectories that move into obstacles
    critics.push_back(&path_costs_); // prefers trajectories on global path
    critics.push_back(&goal_costs_); // prefers trajectories that go towards (local) goal, based on wave propagation
    critics.push_back(&goal_front_costs_); // prefers trajectories that make the nose go towards (local) nose goal
    critics.push_back(&alignment_costs_); // prefers trajectories that keep the robot nose on nose path
    critics.push_back(&prefer_forward_costs_); // prefers moving forward to moving backwards
    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(&generator_, critics);
  }



  void DWAPlanner::publishTrajectories() {
    traj_cloud_pub_.publish(traj_cloud_);
  }



  void DWAPlanner::updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan){
    global_plan_.resize(new_plan.size());
    for (unsigned int i = 0; i < new_plan.size(); ++i) {
      global_plan_[i] = new_plan[i];
    }
  }

  //given the current state of the robot, find a good trajectory
  base_local_planner::Trajectory DWAPlanner::findBestPath(
      tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      tf::Stamped<tf::Pose>& drive_velocities,
      const base_local_planner::LocalPlannerLimitsConfig& limits) {

    // update costmap member
    costmap_ros_->getCostmapCopy(costmap_);

    //make sure that our configuration doesn't change mid-run
    boost::mutex::scoped_lock l(configuration_mutex_);

    Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
    Eigen::Vector3f vel(global_vel.getOrigin().getX(), global_vel.getOrigin().getY(), tf::getYaw(global_vel.getRotation()));

    // prepare cost functions and generator for this run
    generator_.initialise(pos, vel, &limits, sim_period_, acc_lim_, vsamples_);

    // obstacle costs can vary due to scaling footprint feature
    obstacle_costs_.setParams(limits.max_trans_vel, max_scaling_factor_, scaling_speed_);

    // costs for going away from path
    path_costs_.setTargetPoses(global_plan_);

    // costs for not going towards the local goal as much as possible
    goal_costs_.setTargetPoses(global_plan_);

    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    double sq_dist =
        (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
        (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);

    // keeping the nose on the target
    if (sq_dist > forward_point_distance_ * forward_point_distance_ * 2) {
      // costs for robot being aligned with path (nose on path, not just center on path)
      alignment_costs_.setScale(1.0);
      goal_front_costs_.setScale(1.0);
      alignment_costs_.setTargetPoses(global_plan_);
      goal_front_costs_.setTargetPoses(global_plan_);
    } else {
      // once we are close to base, trying to keep the nose close to anything destabilizes behavior.
      alignment_costs_.setScale(0.0);
      goal_front_costs_.setScale(0.0);
    }


    prefer_forward_costs_.setPenalty(backward_motion_penalty_); // TODO make param

    // find best trajectory by sampling and scoring the samples
    scored_sampling_planner_.findBestTrajectory(result_traj_, NULL);

    // verbose publishing of point clouds
    if (publish_cost_grid_pc_) {
      //we'll publish the visualization of the costs to rviz before returning our best trajectory
      map_viz_.publishCostCloud(costmap_);
    }

    // debrief stateful scoring functions
    oscillation_costs_.updateOscillationFlags(pos, &result_traj_, limits.min_trans_vel);

    //if we don't have a legal trajectory, we'll just command zero
    if (result_traj_.cost_ < 0) {
      drive_velocities.setIdentity();
    } else {
      tf::Vector3 start(result_traj_.xv_, result_traj_.yv_, 0);
      drive_velocities.setOrigin(start);
      tf::Matrix3x3 matrix;
      matrix.setRotation(tf::createQuaternionFromYaw(result_traj_.thetav_));
      drive_velocities.setBasis(matrix);
    }

    return result_traj_;
  }
};
