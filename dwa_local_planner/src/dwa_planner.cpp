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

#include <mbf_msgs/ExePathResult.h>
#include <mbf_utility/navigation_utility.h>

namespace dwa_local_planner {
  void DWAPlanner::reconfigure(DWAPlannerConfig &config)
  {
    generator_.setParameters(
        config.sim_time,
        config.sim_granularity,
        config.angular_sim_granularity,
        config.use_dwa,
        sim_period_);

    double resolution = planner_util_->getCostmap()->getResolution();
    path_distance_bias_ = resolution * config.path_distance_bias;
    // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
    path_costs_.setScale(path_distance_bias_);
    alignment_costs_.setScale(path_distance_bias_);

    goal_distance_bias_ = resolution * config.goal_distance_bias;
    goal_costs_.setScale(goal_distance_bias_);
    goal_front_costs_.setScale(goal_distance_bias_);

    occdist_scale_ = config.occdist_scale;
    obstacle_costs_.setScale(occdist_scale_);

    stop_time_buffer_ = config.stop_time_buffer;
    oscillation_costs_.setOscillationResetDist(config.oscillation_reset_dist, config.oscillation_reset_angle);
    forward_point_distance_ = config.forward_point_distance;
    goal_front_costs_.setXShift(forward_point_distance_);
    alignment_costs_.setXShift(forward_point_distance_);

    // obstacle costs can vary due to scaling footprint feature
    obstacle_costs_.setParams(config.max_vel_trans, config.max_forward_inflation, config.max_sideward_inflation, config.scaling_speed, config.occdist_use_footprint);

    twirling_costs_.setScale(config.twirling_scale);

    backwardvel_scale_ = 1.2 * config.sim_time * (config.path_distance_bias + config.goal_distance_bias);
    prefer_forward_costs_.setScale(backwardvel_scale_);

    max_backward_sq_dist_ = config.max_backward_dist * config.max_backward_dist;

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
      obstacle_costs_(planner_util->getCostmap()),
      path_costs_(planner_util->getCostmap()),
      goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      alignment_costs_(planner_util->getCostmap())
  {
    ros::NodeHandle private_nh("~/" + name);

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

    private_nh.param("global_frame_id", frame_id_, std::string("odom"));

    traj_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("trajectory_cloud", 1);
    private_nh.param("publish_traj_pc", publish_traj_pc_, false);

    // set up all the cost functions that will be applied in order
    // (any function returning negative values will abort scoring, so the order can improve performance)
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    critics.push_back(&path_align_costs_);
    critics.push_back(&oscillation_costs_); // discards oscillating motions (assisgns cost -1)
    critics.push_back(&obstacle_costs_); // discards trajectories that move into obstacles
    critics.push_back(&goal_front_costs_); // prefers trajectories that make the nose go towards (local) nose goal
    critics.push_back(&alignment_costs_); // prefers trajectories that keep the robot nose on nose path
    critics.push_back(&path_costs_); // prefers trajectories on global path
    critics.push_back(&goal_costs_); // prefers trajectories that go towards (local) goal, based on wave propagation
    critics.push_back(&prefer_forward_costs_); // prefer trajectories that don't go backwards
    critics.push_back(&twirling_costs_); // optionally prefer trajectories that don't spin

    // trajectory generators
    std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&generator_);

    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

    private_nh.param("cheat_factor", cheat_factor_, 1.0);

    if (planner_util->getCostmap() != NULL) {
      world_model_ = new base_local_planner::CostmapModel(*planner_util->getCostmap());
    }
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

    total_cost =
        path_distance_bias_ * path_cost +
        goal_distance_bias_ * goal_cost +
        occdist_scale_ * occ_cost;
    return true;
  }

  inline bool is2DPoseEqual(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b) {
    return a.pose.position.x == b.pose.position.x && a.pose.position.y == b.pose.position.y &&
        a.pose.orientation.z * b.pose.orientation.w == a.pose.orientation.w * b.pose.orientation.z;
  }

  bool DWAPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    const std::vector<geometry_msgs::PoseStamped>& prevPlan = planner_util_->getGlobalPlan();
    // conservatively reset oscillation flags: reset only for plan towards different goal
    if (orig_global_plan.empty() || prevPlan.empty() || !is2DPoseEqual(prevPlan.back(), orig_global_plan.back())) {
      oscillation_costs_.resetOscillationFlags();
    }
    return planner_util_->setPlan(orig_global_plan);
  }

  std::vector<geometry_msgs::Point> DWAPlanner::getScaledFootprint(const base_local_planner::Trajectory &traj) const {
    return obstacle_costs_.getScaledFootprint(traj);
  }

  /**
   * This function is used when other strategies are to be applied,
   * but the cost functions for obstacles are to be reused.
   */
  bool DWAPlanner::checkTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f vel_samples){
    oscillation_costs_.resetOscillationFlags();
    base_local_planner::Trajectory traj;
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);
    generator_.generateTrajectory(pos, vel, vel_samples, traj);
    double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

    //otherwise the check fails
    return false;
  }

  uint32_t DWAPlanner::cropPlanToReachableGoal(
    const geometry_msgs::PoseStamped& global_pose,
    const std::vector<geometry_msgs::Point>& footprint_spec,
    std::vector<geometry_msgs::PoseStamped>& plan) {
    // search the path for a reachable goal
    const double dist_to_goal = mbf_utility::distance(global_pose, plan.back());

    // find furthest point along the path of which the corresponding cell is in free space
    const costmap_2d::Costmap2D* const costmap = planner_util_->getCostmap();
    int last_point_on_the_map = -1;
    int last_point_in_free_space = -1;
    for (unsigned int i = 0; i < plan.size(); ++i) {
      double g_x = plan[i].pose.position.x;
      double g_y = plan[i].pose.position.y;
      unsigned int map_x, map_y;
      if (!costmap->worldToMap(g_x, g_y, map_x, map_y)) {
        // give up once a point within the map had been found
        if (last_point_on_the_map >= 0) {
          break;
        }
        continue;
      }

      last_point_on_the_map = i;
      if (costmap->getCost(map_x, map_y) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        last_point_in_free_space = i;
      }
    }

    // handle general errors
    if (last_point_on_the_map == -1) {
      ROS_ERROR("None of the points of the global plan were in the local costmap, global plan points too far from robot");
      return mbf_msgs::ExePathResult::OUT_OF_MAP;
    }

    if (last_point_in_free_space == -1) {
      ROS_ERROR("None of the points of the global plan were in free space");
      return mbf_msgs::ExePathResult::BLOCKED_PATH;
    }

    const auto dist_to_last_free_point = mbf_utility::distance(global_pose, plan[last_point_in_free_space]);
    const auto limits = planner_util_->getCurrentLimits();
    if (dist_to_last_free_point > limits.goal_obstacle_approach_distance) {
      // If the obstacle blocking the goal is further than goal_obstacle_approach_distance,
      // just target the last free point instead, by cropping the path to that point.

      // The reasoning for going towards the goal even though it is blocked by an obstacle according to the costmap
      // is to ensure that the goal is actually still blocked (i.e. not because of some left-over obstacle from some earlier time)
      plan.resize(last_point_in_free_space+1);
      return mbf_msgs::ExePathResult::SUCCESS;
    }

    // goal / obstacle blocking the goal is within goal_obstacle_approach_distance
    // -> check if any path point close to the goal is reachable (i.e. can fit robot footprint without colliding)
    const double goal_yaw = tf2::getYaw(plan.back().pose.orientation);
    const double max_goal_deviation = std::max(0.0, limits.xy_goal_tolerance - limits.xy_min_goal_tolerance);
    bool reachable_goal_found = false;
    auto i = last_point_in_free_space;
    for (; i >= 0; --i) {
      const auto dist_to_goal = mbf_utility::distance(plan[i], plan.back());
      if (dist_to_goal > max_goal_deviation) {
        break;
      }
      const auto footprint_cost = world_model_->footprintCost(plan[i].pose.position.x, plan[i].pose.position.y, goal_yaw, footprint_spec);
      reachable_goal_found = footprint_cost >= 0;
      if (reachable_goal_found) {
        break;
      }
    }

    if (!reachable_goal_found) {
      ROS_WARN_STREAM("The footprint is in collision for all path points within " << max_goal_deviation << "[m] of the goal");
      return mbf_msgs::ExePathResult::BLOCKED_GOAL;
    }

    if (i < 0) {
      // this seems unlikely, but needs to be handled
      ROS_WARN("The footprint is in collision for all path points");
      return mbf_msgs::ExePathResult::BLOCKED_PATH;
    }

    plan.resize(i+1);
    return mbf_msgs::ExePathResult::SUCCESS;
  }

  uint32_t DWAPlanner::updatePlanAndLocalCosts(
      const geometry_msgs::PoseStamped& global_pose,
      const std::vector<geometry_msgs::PoseStamped>& new_plan,
      const std::vector<geometry_msgs::Point>& footprint_spec) {
    global_plan_.resize(new_plan.size());
    for (unsigned int i = 0; i < new_plan.size(); ++i) {
      global_plan_[i] = new_plan[i];
    }

    std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
    const auto result = cropPlanToReachableGoal(global_pose, footprint_spec, front_global_plan);
    if (result != mbf_msgs::ExePathResult::SUCCESS) {
      return result;
    }

    const geometry_msgs::PoseStamped goal_pose = front_global_plan.back();
    const double dist_to_goal = mbf_utility::distance(global_pose, goal_pose);

    obstacle_costs_.setFootprint(footprint_spec);

    path_align_costs_.setTargetPoses(global_plan_, global_pose);

    // costs for going away from path
    path_costs_.setTargetPoses(front_global_plan);
    alignment_costs_.setTargetPoses(front_global_plan);

    // costs for not going towards the local goal as much as possible
    goal_costs_.setTargetPoses(front_global_plan);

    // alignment costs

    // we want the robot nose to be drawn to its final position
    // (before robot turns towards goal orientation), not the end of the
    // path for the robot center. Choosing the final position after
    // turning towards goal orientation causes instability when the
    // robot needs to make a 180 degree turn at the end
    double angle_to_goal = atan2(goal_pose.pose.position.y - global_pose.pose.position.y, goal_pose.pose.position.x - global_pose.pose.position.x);

    double forward_point_distance = forward_point_distance_;
    const auto cos_angle_to_goal = cos(angle_to_goal);
    const auto sin_angle_to_goal = sin(angle_to_goal);
    const auto sq_dist = dist_to_goal * dist_to_goal;
    if (sq_dist < MIN_GOAL_DIST_SQ) {
      // when close to the goal, reduce forward_point_distance such that the robot can reach the goal pose
      // without its nose entering space considered as occupied by obstacles
      forward_point_distance = 0;
      const float num_steps = std::ceil(forward_point_distance_ / planner_util_->getCostmap()->getResolution());
      for (int i = 1; i <= num_steps; ++i) {
          const double new_forward_point_distance = (i / num_steps) * forward_point_distance_;
          const double x = front_global_plan.back().pose.position.x +
              std::abs(new_forward_point_distance) * cos_angle_to_goal;
          const double y = front_global_plan.back().pose.position.y +
              std::abs(new_forward_point_distance) * sin_angle_to_goal;

          unsigned cx, cy;
          if (!planner_util_->getCostmap()->worldToMap(x, y, cx, cy)) {
              break;
          }
          const auto cost = planner_util_->getCostmap()->getCost(cx, cy);
          // same cost check as in MapGrid::updatePathCell()
          if (cost == costmap_2d::LETHAL_OBSTACLE ||
                cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
                cost == costmap_2d::NO_INFORMATION) {
              break;
          }
          forward_point_distance = new_forward_point_distance;
      }
    }

    front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
        std::abs(forward_point_distance) * cos_angle_to_goal;
    front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y +
        std::abs(forward_point_distance) * sin_angle_to_goal;

    goal_front_costs_.setTargetPoses(front_global_plan);

    if (sq_dist > max_backward_sq_dist_ && path_align_costs_.isTurningRequired()) {
      // enable turning penalty
      path_align_costs_.setScale(1.0);
      // disable goal cost during turning because turning won't move the robot closer to the goal
      goal_costs_.setScale(0.0);
      // disable alignment cost during turning because turning will inadvertedly move the nose off the path
      alignment_costs_.setScale(0.0);

      // retract nose
      goal_front_costs_.setXShift(0.0);
      alignment_costs_.setXShift(0.0);

    } else {
      // disable turning penalty close to goal
      path_align_costs_.setScale(0.0);

      // revert settings made for when turning was required
      goal_costs_.setScale(goal_distance_bias_);

      if (sq_dist > forward_point_distance * forward_point_distance * cheat_factor_) {
        alignment_costs_.setScale(path_distance_bias_);
      } else {
        // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
        alignment_costs_.setScale(0.0);
      }

      // reset to forward_point_distance_
      goal_front_costs_.setXShift(forward_point_distance);
      alignment_costs_.setXShift(forward_point_distance);
    }

    // disable cost for backwards motion close to the goal
    if (sq_dist > max_backward_sq_dist_) {
        prefer_forward_costs_.setScale(backwardvel_scale_);
    } else {
        prefer_forward_costs_.setScale(0.0);
    }
    return mbf_msgs::ExePathResult::SUCCESS;
  }

  /*
   * given the current state of the robot, find a good trajectory
   */
  std::pair<base_local_planner::Trajectory, ExePathOutcome> DWAPlanner::findBestPath(
      const geometry_msgs::PoseStamped& global_pose,
      const geometry_msgs::PoseStamped& global_vel,
      geometry_msgs::PoseStamped& drive_velocities) {

    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    // prepare cost functions and generators for this run
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);

    result_traj_.cost_ = -7;
    // find best trajectory by sampling and scoring the samples
    std::vector<base_local_planner::Trajectory> all_explored;
    const auto outcome = scored_sampling_planner_.findBestTrajectory(global_pose, result_traj_, &all_explored);

    if(publish_traj_pc_)
    {
        sensor_msgs::PointCloud2 traj_cloud;
        traj_cloud.header.frame_id = frame_id_;
        traj_cloud.header.stamp = ros::Time::now();

        sensor_msgs::PointCloud2Modifier cloud_mod(traj_cloud);
        cloud_mod.setPointCloud2Fields(5, "x", 1, sensor_msgs::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::PointField::FLOAT32,
                                          "theta", 1, sensor_msgs::PointField::FLOAT32,
                                          "cost", 1, sensor_msgs::PointField::FLOAT32);

        unsigned int num_points = 0;
        for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
        {
            if (t->cost_<0)
              continue;
            num_points += t->getPointsSize();
        }

        cloud_mod.resize(num_points);
        sensor_msgs::PointCloud2Iterator<float> iter_x(traj_cloud, "x");
        for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
        {
            if(t->cost_<0)
                continue;
            // Fill out the plan
            for(unsigned int i = 0; i < t->getPointsSize(); ++i) {
                double p_x, p_y, p_th;
                t->getPoint(i, p_x, p_y, p_th);
                iter_x[0] = p_x;
                iter_x[1] = p_y;
                iter_x[2] = 0.0;
                iter_x[3] = p_th;
                iter_x[4] = t->cost_;
                ++iter_x;
            }
        }
        traj_cloud_pub_.publish(traj_cloud);
    }

    // verbose publishing of point clouds
    if (publish_cost_grid_pc_) {
      //we'll publish the visualization of the costs to rviz before returning our best trajectory
      map_viz_.publishCostCloud(planner_util_->getCostmap());
    }

    // debrief stateful scoring functions
    oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_vel_trans);

    //if we don't have a legal trajectory, we'll just command zero
    if (result_traj_.cost_ < 0) {
      drive_velocities.pose.position.x = 0;
      drive_velocities.pose.position.y = 0;
      drive_velocities.pose.position.z = 0;
      drive_velocities.pose.orientation.w = 1;
      drive_velocities.pose.orientation.x = 0;
      drive_velocities.pose.orientation.y = 0;
      drive_velocities.pose.orientation.z = 0;
    } else {
      drive_velocities.pose.position.x = result_traj_.xv_;
      drive_velocities.pose.position.y = result_traj_.yv_;
      drive_velocities.pose.position.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, result_traj_.thetav_);
      tf2::convert(q, drive_velocities.pose.orientation);
    }

    return std::make_pair(result_traj_, outcome);
  }
};
