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

#include <srslib_timing/ScopedTimingSampleRecorder.hpp>

#include <dwa_local_planner/dwa_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/map_grid_cost_point.h>
#include <base_local_planner/geometry_math_helpers.h>
#include <cmath>

//for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>

namespace dwa_local_planner {
  void DWAPlanner::reconfigure(DWAPlannerConfig &config)
  {

    boost::mutex::scoped_lock l(configuration_mutex_);

    generator_.setParameters(
        config.sim_time,
        config.sim_granularity,
        config.angular_sim_granularity,
        config.use_dwa,
        sim_period_);

    follower_generator_.setParameters(
        config.sim_time,
        config.sim_granularity,
        config.follower_generator_kp_theta,
        config.follower_generator_lookahead_distance,
        config.follower_generator_num_trajectories
      );

    cw_tip_generator_.setParameters(
      config.sim_time,
      config.sim_granularity,
      config.tip_generator_num_trajectories
    );

    ccw_tip_generator_.setParameters(
      config.tip_sim_time,
      config.sim_granularity,
      config.tip_generator_num_trajectories
    );

    generator_sim_time_ = config.sim_time;
    double resolution = planner_util_->getCostmap()->getResolution();
    pdist_scale_ = config.path_distance_bias;
    // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
    path_costs_.setScale(resolution * pdist_scale_ * 0.5);
    alignment_costs_.setScale(resolution * pdist_scale_ * 0.5);

    gdist_scale_ = config.goal_distance_bias;
    goal_costs_.setScale(resolution * gdist_scale_ * 0.5);
    goal_front_costs_.setScale(resolution * gdist_scale_ * 0.5);

    occdist_scale_ = config.occdist_scale;
    obstacle_costs_.setScale(resolution * occdist_scale_);

    velocity_costs_.setScale(config.velocity_scale);
    velocity_costs_.setMaxVelocity(config.max_vel_x);
    velocity_costs_.setMinVelocity(config.min_vel_x);

    stop_time_buffer_ = config.stop_time_buffer;
    oscillation_costs_.setOscillationResetDist(config.oscillation_reset_dist, config.oscillation_reset_angle);
    forward_point_distance_ = config.forward_point_distance;
    goal_front_costs_.setXShift(forward_point_distance_);
    alignment_costs_.setXShift(forward_point_distance_);

    jerk_costs_.setScale(config.jerk_scale);

    euclidean_distance_costs_.setScale(config.euclidean_distance_scale);
    euclidean_distance_scale_ = config.euclidean_distance_scale;
    always_use_euclidean_goal_distance_ = config.always_use_euclidean_goal_distance;

    heading_costs_.setPoseCaptureMinRadius(config.heading_critic_capture_min_radius);
    heading_costs_.setPoseCaptureMaxRadius(config.heading_critic_capture_max_radius);
    heading_costs_.setRejectionHalfAngle(config.heading_critic_half_angle);

    global_plan_distance_costs_.setMaxAllowedDistanceFromPlan(config.max_allowed_distance_from_global_plan);

    minimum_simulation_time_factor_ = config.minimum_simulation_time_factor;

    // obstacle costs can vary due to scaling footprint feature
    obstacle_costs_.setParams(config.max_trans_vel, config.max_scaling_factor, config.scaling_speed);

    oscillation_reset_plan_divergence_distance_ = config.oscillation_reset_plan_divergence_distance;

    close_to_goal_range_ = config.close_to_goal_range;

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

    generator_.enable(config.enable_simple_trajectory_generator);
    follower_generator_.enable(config.enable_follower_trajectory_generator);
    stationary_generator_.enable(config.enable_stationary_trajectory_generator);
    cw_tip_generator_.enable(config.enable_tip_trajectory_generator);
    ccw_tip_generator_.enable(config.enable_tip_trajectory_generator);

    publish_traj_pc_ = config.publish_traj_pc;
  }

  DWAPlanner::DWAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util) :
      planner_util_(planner_util),
      obstacle_costs_(planner_util->getCostmap()),
      path_costs_(planner_util->getCostmap()),
      goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      alignment_costs_(planner_util->getCostmap()), tdr_("DWAPlannerROS")

  {
    ros::NodeHandle private_nh("~/" + name);

    goal_costs_.setStopOnFailure( false );
    goal_front_costs_.setStopOnFailure( false );
    alignment_costs_.setStopOnFailure( false );

    //Assuming this planner is being run within the navigation stack, we can
    //just do an upward search for the frequency at which its being run. This
    //also allows the frequency to be overwritten locally.
    std::string controller_frequency_param_name;
    if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name)) {
      sim_period_ = 0.05;
    } else {
      double controller_frequency = 0;
      private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
      if(controller_frequency > 0) {
        sim_period_ = 1.0 / controller_frequency;
      } else {
        ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
        sim_period_ = 0.05;
      }
    }
    ROS_INFO("Sim period is set to %.2f", sim_period_);

    oscillation_costs_.resetOscillationFlags();

    bool sum_scores;
    private_nh.param("sum_scores", sum_scores, false);
    obstacle_costs_.setSumScores(sum_scores);


    private_nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
    map_viz_.initialize(name, planner_util->getGlobalFrame(), boost::bind(&DWAPlanner::getCellCosts, this, _1, _2, _3, _4, _5, _6));

    std::string frame_id;
    private_nh.param("global_frame_id", frame_id, std::string("odom"));

    traj_cloud_ = new pcl::PointCloud<base_local_planner::MapGridCostPoint>;
    traj_cloud_->header.frame_id = frame_id;
    traj_cloud_pub_.advertise(private_nh, "trajectory_cloud", 1);
    private_nh.param("publish_traj_pc", publish_traj_pc_, false);

    // set up all the cost functions that will be applied in order
    // (any function returning negative values will abort scoring, so the order can improve performance)
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    ROS_ERROR("NOT using global plan distance costs");
    //critics.push_back(&global_plan_distance_costs_); // discards trajectories that are moving if the global plan is too far from the robot
    critics.push_back(&heading_costs_); // discards trajectories that are not turn in place if path is behind robot
    critics.push_back(&oscillation_costs_); // discards oscillating motions (assisgns cost -1)
    critics.push_back(&velocity_costs_); // scales cost based on velocity
    critics.push_back(&obstacle_costs_); // discards trajectories that move into obstacles
    critics.push_back(&goal_front_costs_); // prefers trajectories that make the nose go towards (local) nose goal
    critics.push_back(&alignment_costs_); // prefers trajectories that keep the robot nose on nose path
    critics.push_back(&path_costs_); // prefers trajectories on global path
    critics.push_back(&goal_costs_); // prefers trajectories that go towards (local) goal, based on wave propagation
    critics.push_back(&jerk_costs_); // prefers trajectories that have the same acceleration as the previous trajectory
    critics.push_back(&euclidean_distance_costs_); // costs trajectories based on distance to goal

    // trajectory generators
    std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&generator_);
    generator_list.push_back(&stationary_generator_);
    generator_list.push_back(&follower_generator_);
    generator_list.push_back(&cw_tip_generator_);
    generator_list.push_back(&ccw_tip_generator_);


    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);
  }

  void DWAPlanner::setFootprintSpec(const std::vector<geometry_msgs::Point>& footprint_spec)
  {
    ROS_INFO("Get footprint from DWAPlannerROS");
    robot_footprint_ = footprint_spec;
    obstacle_costs_.setFootprint(robot_footprint_);
  }

  // used for visualization only, total_costs are not really total costs
  bool DWAPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {

    path_cost = path_costs_.getCellCosts(cx, cy);
    goal_cost = goal_costs_.getCellCosts(cx, cy);
    occ_cost = planner_util_->getCostmap()->getCost(cx, cy);
    if (path_cost == path_costs_.obstacleCosts() ||
        path_cost == path_costs_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }

    double resolution = planner_util_->getCostmap()->getResolution();
    total_cost =
        pdist_scale_ * resolution * path_cost +
        gdist_scale_ * resolution * goal_cost +
        occdist_scale_ * occ_cost;
    return true;
  }

  bool DWAPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    double divergenceDistance = planner_util_->distanceToPlanDivergence(orig_global_plan);
    if (divergenceDistance >= 0 && divergenceDistance < oscillation_reset_plan_divergence_distance_)
    {
      ROS_DEBUG("flag reset due to set plan at range %f", divergenceDistance);
      oscillation_costs_.resetOscillationFlags();
    }
    return planner_util_->setPlan(orig_global_plan);
  }

  /**
   * This function is used when other strategies are to be applied,
   * but the cost functions for obstacles are to be reused.
   */
  bool DWAPlanner::checkTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f vel_samples){

    // set footprint
    ROS_DEBUG_NAMED("dwaPlanner", "checkTrajectory() sets footprint with size %zu", robot_footprint_.size());
    obstacle_costs_.setFootprint(robot_footprint_);
    oscillation_costs_.resetOscillationFlags();
    base_local_planner::Trajectory traj;
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_,
        true);
    if (!generator_.generateTrajectory(pos, vel, vel_samples, traj, true))
    {
      ROS_WARN("Could not generate trajectory %f, %f, %f", vel_samples[0], vel_samples[1], vel_samples[2]);
      return false;
    }
    double cost = scored_sampling_planner_.scoreTrajectory(traj, -1, nullptr);
    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

    //otherwise the check fails
    return false;
  }


  void DWAPlanner::updatePlanAndLocalCosts(
      tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      const std::vector<geometry_msgs::PoseStamped>& new_plan) {
    global_plan_.resize(new_plan.size());
    for (unsigned int i = 0; i < new_plan.size(); ++i) {
      global_plan_[i] = new_plan[i];
    }

    // costs for going away from path
    path_costs_.setTargetPoses(global_plan_);

    // costs for not going towards the local goal as much as possible
    goal_costs_.setTargetPoses(global_plan_);

    euclidean_distance_costs_.setTargetPoses(global_plan_);

    // alignment costs
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();

    Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
    double sq_dist =
        (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
        (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);

    // we want the robot nose to be drawn to its final position
    // (before robot turns towards goal orientation), not the end of the
    // path for the robot center. Choosing the final position after
    // turning towards goal orientation causes instability when the
    // robot needs to make a 180 degree turn at the end
    std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
    double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
    front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
      forward_point_distance_ * cos(angle_to_goal);
    front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + forward_point_distance_ *
      sin(angle_to_goal);

    goal_front_costs_.setTargetPoses(front_global_plan);
    // costs for robot being aligned with path (nose on path)
    alignment_costs_.setTargetPoses(global_plan_);


    // keeping the nose on the path
    if (sq_dist > close_to_goal_range_ * close_to_goal_range_) {
      double resolution = planner_util_->getCostmap()->getResolution();
      alignment_costs_.setScale(resolution * pdist_scale_ * 0.5);

      goal_front_costs_.setScale(resolution * gdist_scale_ * 0.5);
      goal_costs_.setScale(resolution * gdist_scale_ * 0.5);

      // costs for going fast near obstacles
      obstacle_costs_.setIgnoreSpeedCost(false);
      if (!always_use_euclidean_goal_distance_)
    {
        euclidean_distance_costs_.setScale(0.0);
      }
      else
      {
        euclidean_distance_costs_.setScale(euclidean_distance_scale_);
      }
    } else {
      // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
      alignment_costs_.setScale(0.0);

      goal_front_costs_.setScale(0.0);
      goal_costs_.setScale(0.0);

      // costs for going fast near obstacles
      obstacle_costs_.setIgnoreSpeedCost(true);

      euclidean_distance_costs_.setScale(euclidean_distance_scale_);
    }

    // Change the sim time depending on distance from goal.
    double max_vel_x = planner_util_->getCurrentLimits().max_vel_x;

    double sq_max_vel_sim_distance = (generator_sim_time_ * max_vel_x) * (generator_sim_time_ * max_vel_x);
    if (sq_dist > sq_max_vel_sim_distance)
    {
      generator_.setSimTime(generator_sim_time_);
    }
    else
    {
      double max_linear_accel = planner_util_->getCurrentLimits().getAccLimits()[0];
      if (max_vel_x > 0 && max_linear_accel > 0)
      {
        // Shorten the sim time.
        double time_to_goal_at_max = std::sqrt(sq_dist) / max_vel_x;
        // We don't want to overshoot the goal either -> leads to spinning around.
        double time_to_decel_from_max = max_vel_x / max_linear_accel;
        double time_to_decel_from_current = global_vel.getOrigin().getX() / max_linear_accel;
        double sim_time = std::max(std::max(time_to_goal_at_max,
                                            time_to_decel_from_current),
                                            minimum_simulation_time_factor_ * generator_sim_time_);
        generator_.setSimTime(sim_time);
      }
    }

    // costs for having the wrong heading
    geometry_msgs::PoseStamped global_pose_as_pose;
    tf::poseStampedTFToMsg(global_pose, global_pose_as_pose);
    heading_costs_.setCurrentPose(global_pose_as_pose);
    heading_costs_.setTargetPoses(global_plan_);
    heading_costs_.setGoalDistanceSquared(sq_dist);

    global_plan_distance_costs_.setCurrentPose(global_pose_as_pose);
    global_plan_distance_costs_.setTargetPoses(global_plan_);

    velocity_costs_.setGoalDistanceSquared(sq_dist);
  }

  bool DWAPlanner::getDistanceAndTimeEstimates(const tf::Stamped<tf::Pose>& poseTf, double& distance, double& time)
  {
    // Get the current plan and pose.
    if (global_plan_.empty())
    {
      distance = 0;
      time = 0;
      return true;
    }

    geometry_msgs::PoseStamped pose;
    tf::poseStampedTFToMsg(poseTf, pose);

    Eigen::Vector2f pos2 = base_local_planner::poseStampedToVector(pose);
    // Vectors for storage
    Eigen::Vector2f p0 = Eigen::Vector2f::Zero();
    Eigen::Vector2f p1 = Eigen::Vector2f::Zero();

    // Other storage
    double distance_from_robot = -1;
    double minimum_distance = 0.5; // it needs to be at least this close

    for (size_t k = 0; k < global_plan_.size() - 1; ++k)
    {
      // Pull out the datas
      p0 = base_local_planner::poseStampedToVector(global_plan_[k]);
      p1 = base_local_planner::poseStampedToVector(global_plan_[k + 1]);
      double segment_length = (p1 - p0).norm();
      double dist_from_path = base_local_planner::distanceToLineSegment(pos2, p0, p1);

      if (dist_from_path < minimum_distance)
      {
        minimum_distance = dist_from_path;
        // Reset distance
        distance_from_robot = std::max(0.0, segment_length - base_local_planner::distanceAlongLineSegment(pos2, p0, p1));
      }
      else
      {
        distance_from_robot += segment_length;
      }
    }
    if (distance_from_robot < 0)
    {
      // If the distance wasn't set, the robot is too far from the path.
      // Use euclidean distance.
      distance_from_robot = (pos2 - p1).norm();
    }

    distance = distance_from_robot;
    time = distance_from_robot * planner_util_->getCurrentLimits().max_vel_x;
    return true;
  }


  /*
   * given the current state of the robot, find a good trajectory
   */
  base_local_planner::Trajectory DWAPlanner::findBestPath(
      tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      tf::Stamped<tf::Pose>& drive_velocities) {

    ROS_DEBUG_NAMED("dwaPlanner", "findBestPath() sets footprint with size %zu", robot_footprint_.size());
    obstacle_costs_.setFootprint(robot_footprint_);

    //make sure that our configuration doesn't change mid-run
    boost::mutex::scoped_lock l(configuration_mutex_);

    Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
    Eigen::Vector3f vel(global_vel.getOrigin().getX(), global_vel.getOrigin().getY(), tf::getYaw(global_vel.getRotation()));
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    // prepare cost functions and generators for this run
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_,
        true);
    follower_generator_.setGlobalPlan(global_plan_);
    follower_generator_.initialise(pos,
        vel,
        &limits);
    stationary_generator_.initialise(pos,
        vel,
        &limits);
    cw_tip_generator_.initialise(pos,
        vel,
        &limits,
        true);
    ccw_tip_generator_.initialise(pos,
        vel,
        &limits,
        false);
    jerk_costs_.setCurrentVelocity(vel);

    result_traj_.cost_ = -7;
    // find best trajectory by sampling and scoring the samples
    std::vector<base_local_planner::Trajectory> all_explored;
    srs::ScopedTimingSampleRecorder stsr(tdr_.getRecorder("-findBest"));
    scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);
    stsr.stopSample();

    if(publish_traj_pc_)
    {
        base_local_planner::MapGridCostPoint pt;
        traj_cloud_->points.clear();
        traj_cloud_->width = 0;
        traj_cloud_->height = 0;
        std_msgs::Header header;
        pcl_conversions::fromPCL(traj_cloud_->header, header);
        header.stamp = ros::Time::now();
        traj_cloud_->header = pcl_conversions::toPCL(header);
        std::stringstream ss;
        ss << "Costs: ";
        for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
        {
            if(t->cost_<0)
                continue;
            // Fill out the plan
            for(unsigned int i = 0; i < t->getPointsSize(); ++i) {
                double p_x, p_y, p_th;
                t->getPoint(i, p_x, p_y, p_th);
                pt.x=p_x;
                pt.y=p_y;
                pt.z=0;
                pt.path_cost=p_th;
                pt.total_cost=t->cost_;
                traj_cloud_->push_back(pt);
                ss << t->cost_ << ", ";
            }
        }
        traj_cloud_pub_.publish(*traj_cloud_);
        ROS_DEBUG_STREAM(ss.str());
    }

    // verbose publishing of point clouds
    if (publish_cost_grid_pc_) {
      //we'll publish the visualization of the costs to rviz before returning our best trajectory
      map_viz_.publishCostCloud(planner_util_->getCostmap());
    }

    // debrief stateful scoring functions
    oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_trans_vel);

    //if we don't have a legal trajectory, we'll just command zero
    if (result_traj_.cost_ < 0) {
      drive_velocities.setIdentity();
    } else {
      // add it to the jerk costs
      jerk_costs_.setPreviousTrajectoryAndVelocity(result_traj_, vel);

      tf::Vector3 start(result_traj_.xv_, result_traj_.yv_, 0);
      drive_velocities.setOrigin(start);
      tf::Matrix3x3 matrix;
      matrix.setRotation(tf::createQuaternionFromYaw(result_traj_.thetav_));
      drive_velocities.setBasis(matrix);
    }

    return result_traj_;
  }
};
