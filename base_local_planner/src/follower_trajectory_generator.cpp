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

#include <base_local_planner/follower_trajectory_generator.h>
#include <ros/ros.h>
#include <cmath>

#include <base_local_planner/velocity_iterator.h>

namespace base_local_planner {

void FollowerTrajectoryGenerator::initialise(
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
  stored_idx_ = 0;

  end_times_.clear();
  if (num_trajectories_ > 0)
  {
    float step_t = sim_time_ / num_trajectories_;
    for (size_t k = 1; k <= num_trajectories_; ++k)
    {
      end_times_.push_back(k * step_t);
    }
  }
  ROS_DEBUG("FTG intialized with %d times and end time %f", num_trajectories_, sim_time_);
}

void FollowerTrajectoryGenerator::setParameters(
    double sim_time,
    double sim_granularity,
    double kp_theta,
    double lookahead_distance,
    double num_trajectories) {

  sim_time_ = sim_time;
  sim_granularity_ = sim_granularity;
  kp_theta_ = kp_theta;
  lookahead_distance_ = lookahead_distance;
  num_trajectories_ = num_trajectories;
}

/**
 * Whether this generator can create more trajectories
 */
bool FollowerTrajectoryGenerator::hasMoreTrajectories() {
  ROS_DEBUG("FTG check for more.  next: %d, end_times size %zu, plan empty: %d",
    next_sample_index_, end_times_.size(), global_plan_.empty());
  return enabled_ && next_sample_index_ < end_times_.size() && !global_plan_.empty();
}

/**
 * Create and return the next sample trajectory
 */
bool FollowerTrajectoryGenerator::nextTrajectory(Trajectory &comp_traj) {
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
bool FollowerTrajectoryGenerator::generateTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      double max_sim_time,
      base_local_planner::Trajectory& traj) {
  double eps = 1e-4;
  traj.cost_   = 0.0; // placed here in case we return early
  double simulation_time = max_sim_time;
  unsigned int start_idx = 0;

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
    start_idx = stored_idx_;
    loop_vel = vel;
  }
  else
  {
    traj.resetPoints();
    unsigned int dummy;
    loop_vel = computeNewVelocities(pos, vel, limits_->getAccLimits(), sim_granularity_, start_idx, dummy);
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
    loop_vel = computeNewVelocities(pos, loop_vel, limits_->getAccLimits(), dt,
      start_idx, found_idx);
    start_idx = found_idx;
    //update the position of the robot using the velocities passed in
    pos = computeNewPositions(pos, loop_vel, dt);

  } // end for simulation steps
  stored_trajectory_ = traj;
  stored_idx_ = start_idx;

  return num_steps > 0; // true if trajectory has at least one point
}

Eigen::Vector3f FollowerTrajectoryGenerator::computeNewPositions(const Eigen::Vector3f& pos,
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
Eigen::Vector3f FollowerTrajectoryGenerator::computeNewVelocities(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, Eigen::Vector3f acclimits, double dt,
    unsigned int start_idx, unsigned int& found_idx) {
  Eigen::Vector3f new_vel = Eigen::Vector3f::Zero();

  // get the desired heading.
  double desired_heading, distance_to_goal;
  getDesiredHeadingAndGoalDistance(pos, start_idx, found_idx, desired_heading, distance_to_goal);

  if (start_idx == found_idx && found_idx == global_plan_.size() -1 )
  {
    ROS_DEBUG("At the goal");
    desired_heading = pos[2];
    distance_to_goal = 0;
  }

  if (distance_to_goal < 0.05)
  {
    desired_heading = pos[2];
    ROS_DEBUG("Close to goal: start %d found %d size: %zu dist %f",
      start_idx, found_idx, global_plan_.size(), distance_to_goal);
  }

  // Simple proportional control with limits
  double heading_error = desired_heading - pos[2];
  while (heading_error > M_PI)
  {
    heading_error -= 2 * M_PI;
  }
  while (heading_error < -M_PI)
  {
    heading_error += 2 * M_PI;
  }

  double desired_angular_velocity = heading_error * kp_theta_;
  // limit it
  if (desired_angular_velocity > 0)
  {
    desired_angular_velocity = std::min(desired_angular_velocity, limits_->max_rot_vel);
  }
  else
  {
    desired_angular_velocity = std::max(desired_angular_velocity, -limits_->max_rot_vel);
  }

  // Scale desired linear velocity by heading error and by distance from goal
  double max_linear_vel = 0;
  if (distance_to_goal > 0.0)
  {
    max_linear_vel = std::min(limits_->max_vel_x, 0.55 * std::sqrt(2 * distance_to_goal * acclimits[0]));
  }
  double desired_linear_velocity = std::max(0.0, max_linear_vel - 0.25 * std::fabs(heading_error) - 0.25 * std::fabs(vel[2]));

  // Now scale to within the limits
  if (vel[0] < desired_linear_velocity) {
    new_vel[0] = std::min(double(desired_linear_velocity), vel[0] + acclimits[0] * dt);
  } else {
    new_vel[0] = std::max(double(desired_linear_velocity), vel[0] - acclimits[0] * dt);
  }

  if (vel[2] < desired_angular_velocity) {
    new_vel[2] = std::min(double(desired_angular_velocity), vel[2] + acclimits[2] * dt);
  } else {
    new_vel[2] = std::max(double(desired_angular_velocity), vel[2] - acclimits[2] * dt);
  }

  ROS_DEBUG("FTG: des h: %f, curr h: %f, h err: %f, w: %f, v: %f, dist: %f",
    desired_heading, pos[2], heading_error, desired_angular_velocity, desired_linear_velocity, distance_to_goal);
  return new_vel;
}


void FollowerTrajectoryGenerator::getDesiredHeadingAndGoalDistance(const Eigen::Vector3f& pos,
  unsigned int start_idx, unsigned int& closest_idx,
  double& desired_heading, double& distance_to_goal)
{
  closest_idx = start_idx;
  // Iterate over the entire trajectory and find the closest point.
  // As it goes, keep track of information which is reset if a point is closer.
  double heading_lookahead_distance = lookahead_distance_;

  // Store in small vector
  Eigen::Vector2f pos2 = Eigen::Vector2f::Zero();
  pos2[0] = pos[0];
  pos2[1] = pos[1];
  // Vectors for storage
  Eigen::Vector2f p0 = Eigen::Vector2f::Zero();
  Eigen::Vector2f p1 = Eigen::Vector2f::Zero();

  // Other storage
  distance_to_goal = 0;
  double minimum_distance = std::numeric_limits<double>::max();
  Eigen::Vector2f pose_of_heading = Eigen::Vector2f::Zero();

  for (size_t k = start_idx; k < global_plan_.size() - 1; ++k)
  {
    ROS_DEBUG("Searching along plan at k=%zu", k);
    // Pull out the datas
    p0 = poseStampedToVector(global_plan_[k]);
    p1 = poseStampedToVector(global_plan_[k + 1]);
    double segment_length = (p1 - p0).norm();
    double dist_from_path = distanceToLineSegment(pos2, p0, p1);
    if (dist_from_path < minimum_distance)
    {
      minimum_distance = dist_from_path;
      // Reset distance
      distance_to_goal = std::max(0.0, segment_length - distanceAlongLineSegment(pos2, p0, p1));
      pose_of_heading = p1;
      closest_idx = k;
      ROS_DEBUG("New closest at idx %zu, from path %f to goal %f",
                k, dist_from_path, segment_length - distanceAlongLineSegment(pos2, p0, p1));
    }
    else
    {
      if (distance_to_goal < heading_lookahead_distance)
      {
        if (distance_to_goal + segment_length > heading_lookahead_distance)
        {
          pose_of_heading = poseAtDistanceAlongLineSegment(heading_lookahead_distance - distance_to_goal, p0, p1);
        }
        else
        {
          pose_of_heading = p1;
        }
      }
      distance_to_goal += segment_length;
    }
  }
  // Calculate the desired heading.
  desired_heading = std::atan2(pose_of_heading[1] - pos2[1], pose_of_heading[0] - pos2[0]);
  ROS_DEBUG("Pos: [%f, %f], pose_of_heading: [%f, %f], des: %f",
    pos2[0], pos2[1], pose_of_heading[0], pose_of_heading[1], desired_heading);
}

double FollowerTrajectoryGenerator::distanceToLineSegment(const Eigen::Vector2f& pos,
  const Eigen::Vector2f& p0, const Eigen::Vector2f& p1)
{
  double l2 = (p1 - p0).squaredNorm();
  if (l2 == 0.0)
  {
    ROS_DEBUG("dtLS early.p0 %f,%f p1 %f, %f, pos %f, %f",
      p0[0], p0[1], p1[0], p1[1], pos[0], pos[1]);
    return (pos - p1).norm();
  }
  double t = std::max(0.0, std::min(1.0, (pos - p0).dot(p1 - p0) / l2));

  Eigen::Vector2f projection = p0 + t * (p1 - p0);

  ROS_DEBUG("dtLS: p0 %f,%f p1 %f, %f, pos %f, %f, t %f, proje %f %f",
    p0[0], p0[1], p1[0], p1[1], pos[0], pos[1], t, projection[0], projection[1]);
  return (pos - projection).norm();
}

double FollowerTrajectoryGenerator::distanceAlongLineSegment(const Eigen::Vector2f& pos,
  const Eigen::Vector2f& p0, const Eigen::Vector2f& p1)
{
  double l = (p1 - p0).norm();
  if (l == 0.0)
  {
    return 0.0;
  }
  return (pos - p0).dot(p1 - p0) / l;
}

Eigen::Vector2f FollowerTrajectoryGenerator::poseAtDistanceAlongLineSegment(double distance,
  const Eigen::Vector2f& p0, const Eigen::Vector2f& p1)
{
  double l2 = (p1 - p0).squaredNorm();
  if (l2 == 0.0)
  {
    return p1;
  }

  double t = distance / l2;

  Eigen::Vector2f projection = p0 + t * (p1 - p0);

  ROS_DEBUG("paDtLS: p0 %f,%f p1 %f, %f, dist %f, t %f, proje %f %f",
    p0[0], p0[1], p1[0], p1[1], distance, t, projection[0], projection[1]);
  return projection;
}

Eigen::Vector2f FollowerTrajectoryGenerator::poseStampedToVector(geometry_msgs::PoseStamped pose)
{
  Eigen::Vector2f p = Eigen::Vector2f::Zero();
  p[0] = pose.pose.position.x;
  p[1] = pose.pose.position.y;
  return p;
}

} /* namespace base_local_planner */

