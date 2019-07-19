/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, 6 River Systems
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
 * Author: DGrieneisen
 *********************************************************************/

#include <base_local_planner/point_and_shoot_trajectory_generator.h>
#include <ros/ros.h>
#include <cmath>
#include <angles/angles.h>

#include <base_local_planner/velocity_iterator.h>
#include <base_local_planner/geometry_math_helpers.h>

namespace base_local_planner {

void PointAndShootTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    base_local_planner::LocalPlannerLimits* limits) {

  // Store the current values.
  pos_ = pos;
  vel_ = vel;
  limits_ = limits;

  // We will generate a series of trajectories where each is the previous plus a litle more time
  // Reset stored information to prepare for next generation
  next_sample_index_ = 0;

  stored_trajectory_end_time_ = -1.0;
  stored_trajectory_.resetPoints();

  end_times_.clear();
  if (num_trajectories_ > 0)
  {
    float step_t = sim_time_ / num_trajectories_;
    for (size_t k = 1; k <= num_trajectories_; ++k)
    {
      end_times_.push_back(k * step_t);
    }
  }
  new_try_ = true;
  ROS_DEBUG("PASTG intialized with %d times and end time %f", num_trajectories_, sim_time_);
}

void PointAndShootTrajectoryGenerator::setParameters(
    double sim_time,
    double sim_granularity,
    double num_trajectories,
    double kp_angular,
    double kp_linear) {

  sim_time_ = sim_time;
  sim_granularity_ = sim_granularity;
  num_trajectories_ = num_trajectories;
  kp_angular_ = kp_angular,
  kp_linear_ = kp_linear;
}

/**
 * Whether this generator can create more trajectories
 */
bool PointAndShootTrajectoryGenerator::hasMoreTrajectories() {
  ROS_DEBUG("PASTG check for more.  next: %d, end_times size %zu",
    next_sample_index_, end_times_.size());
  return enabled_ && next_sample_index_ < end_times_.size();
}

/**
 * Create and return the next sample trajectory
 */
bool PointAndShootTrajectoryGenerator::nextTrajectory(Trajectory &comp_traj) {
  bool result = false;
  if (hasMoreTrajectories()) {
    if (generateTrajectory(
        pos_,
        vel_,
        end_times_[next_sample_index_],
        comp_traj)) {
      result = true;
    }
  }
  next_sample_index_++;
  return result;
}

/**
 * @param pos current position of robot
 * @param vel desired velocity for sampling
 */
bool PointAndShootTrajectoryGenerator::generateTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      double max_sim_time,
      base_local_planner::Trajectory& traj) {
  double eps = 1e-4;
  traj.cost_   = 0.0; // placed here in case we return early
  double simulation_time = max_sim_time;

  // This extends the previously generated trajectory (if it exists)
  Eigen::Vector3f loop_vel;
  if (stored_trajectory_end_time_ > 0.0)
  {
    traj = stored_trajectory_;

    // Calculate the end time
    simulation_time = max_sim_time - stored_trajectory_end_time_;

    double x, y, th;
    stored_trajectory_.getEndpoint(x, y, th);
    pos[0] = x;
    pos[1] = y;
    pos[2] = th;
    double vx, vy, vth;
    stored_trajectory_.getEndVelocity(vx, vy, vth);
    vel[0] = vx;
    vel[1] = vy;
    vel[2] = vth;
    loop_vel = vel;
  }
  else
  {
    traj.resetPoints();
    loop_vel = computeNewVelocities(pos, vel, limits_->max_vel_x,
      limits_->acc_lim_x, limits_->max_tip_vel,
      limits_->acc_limit_tip, sim_granularity_);
    traj.xv_     = loop_vel[0];
    traj.yv_     = loop_vel[1];
    traj.thetav_ = loop_vel[2];
    traj.time_delta_ = sim_granularity_;
    stored_trajectory_end_time_ = 0.0;
  }

  int num_steps = ceil(simulation_time / sim_granularity_);

  //compute a timestep
  double dt = sim_granularity_;

  ROS_DEBUG("Forward simulating with %d steps for %f, starting at %f to %f",
    num_steps, simulation_time, stored_trajectory_end_time_, max_sim_time);
  //simulate the trajectory and check for collisions, updating costs along the way
  for (int i = 0; i < num_steps; ++i) {
    stored_trajectory_end_time_ += dt;
    //add the point to the trajectory so we can draw it later if we want
    traj.addPoint(pos[0], pos[1], pos[2], loop_vel[0], loop_vel[1], loop_vel[2]);

    //calculate velocities
    unsigned int found_idx = 0;
    loop_vel = computeNewVelocities(pos, loop_vel, limits_->max_vel_x,
      limits_->acc_lim_x, limits_->max_tip_vel,
      limits_->acc_limit_tip, dt);

    //update the position of the robot using the velocities passed in
    pos = computeNewPositions(pos, loop_vel, dt);

  } // end for simulation steps

  // Limit the command out to less than the allowed max.
  limits_->applyToTrajectory(traj);

  stored_trajectory_ = traj;

  return num_steps > 0; // true if trajectory has at least one point
}

Eigen::Vector3f PointAndShootTrajectoryGenerator::computeNewPositions(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double dt) {
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

/**
 * cheange vel using acceleration limits to converge towards sample_target-vel
 */
Eigen::Vector3f PointAndShootTrajectoryGenerator::computeNewVelocities(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double max_lin_vel, double lin_accel,
    double max_tip_vel, double tip_accel, double dt) {
  Eigen::Vector3f new_vel = Eigen::Vector3f::Zero();

  if (global_plan_.empty()) {
    ROS_WARN_NAMED("PASTG", "Cannot compute new velocities because there is no global plan");
    return new_vel;
  }

  // Calculate the range and bearing to the final point.
  double goal_x = global_plan_.back().pose.position.x;
  double goal_y = global_plan_.back().pose.position.y;
  double dx = goal_x - pos[0];
  double dy = goal_y - pos[1];

  double range = std::sqrt(dx * dx + dy * dy);
  double bearing = angles::normalize_angle(std::atan2(dy, dx) - pos[2]);

  // Simple proportional control with limits
  double desired_angular_velocity = 0.0;
  if( std::fabs(bearing) > 0.000001)
  {
    desired_angular_velocity = bearing / std::fabs(bearing)
    * std::min(kp_angular_ * std::fabs(bearing), max_tip_vel);
  }

  // Add accel limit
  if (vel[2] < desired_angular_velocity) {
    new_vel[2] = std::min(desired_angular_velocity, vel[2] + tip_accel * dt);
  } else {
    new_vel[2] = std::max(desired_angular_velocity, vel[2] - tip_accel * dt);
  }

  double lin_bear_factor = std::max(1.0 - std::fabs(bearing), 0.0);
  double lin_factor = std::min(kp_linear_ * range, max_lin_vel);
  double desired_linear_velocity = lin_bear_factor * lin_factor;
  // Add accel limit
  if (vel[0] < desired_linear_velocity) {
    new_vel[0] = std::min(desired_linear_velocity, vel[0] + lin_accel * dt);
  } else {
    new_vel[0] = std::max(desired_linear_velocity, vel[0] - lin_accel * dt);
  }
  if (new_try_) {
    ROS_DEBUG_NAMED("PASTG", "range: %f, bearing: %f, des v,w: %f,%f, (lin bear: %f, lin: %f)",
      range, bearing, desired_linear_velocity, desired_angular_velocity,
      lin_bear_factor, lin_factor);
    new_try_ = false;
  }
  return new_vel;
}


} /* namespace base_local_planner */

