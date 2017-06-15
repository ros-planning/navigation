/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, 6 River Systems
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
 * Author: Daniel Grieneisen
 *********************************************************************/

#include <base_local_planner/critics/jerk_cost_function.h>
#include <tf/transform_datatypes.h>

namespace base_local_planner {

JerkCostFunction::JerkCostFunction() : old_linear_accel_(0.0), old_angular_accel_(0.0),
  EPSILON(0.01), current_vel_(0.0, 0.0, 0.0)
{}

void JerkCostFunction::setPreviousTrajectoryAndVelocity(const Trajectory& traj, const Eigen::Vector3f& vel)
{
  // Calculate the linear and angular velocity
  calculateAccelerations(traj, vel, "prev", old_linear_accel_, old_angular_accel_);
}

bool JerkCostFunction::prepare()
{
  // do nothing
  return true;
}

double JerkCostFunction::scoreTrajectory(Trajectory &traj)
{
  // ROS_WARN("Scoring trajectory");
  // Don't add any cost if the robot isn't moving yet.
  if (std::fabs(current_vel_[0]) < EPSILON && std::fabs(current_vel_[2]) < EPSILON)
  {
    return 0;
  }

  // Don't add any cost if the linear velocity is below a certain amount
  if (std::fabs(current_vel_[0] < 0.2))
  {
    return 0;
  }

  // Get the accelerations
  double new_lin_accel = 0;
  double new_angular_accel = 0;
  calculateAccelerations(traj, current_vel_, "scoreTraj", new_lin_accel, new_angular_accel);

  if (old_linear_accel_ < 0)
  {
    // Don't discourage accelerating if the robot is slowing.
    return 0;
  }

  // Compare to the old accelerations
  double dLinearAccel = std::fabs(new_lin_accel - old_linear_accel_);
  double dAngularAccel = std::fabs(new_angular_accel - old_angular_accel_);

  // Map differences into cost.
  double cost = (dLinearAccel + 0.1 * dAngularAccel);

  return cost;
}

void JerkCostFunction::calculateAccelerations(const Trajectory& traj, Eigen::Vector3f vel,
  std::string msg, double& linear_accel, double& angular_accel)
{
  if (traj.time_delta_ > 0)
  {
    linear_accel = (traj.xv_ - vel[0]) / traj.time_delta_;
    angular_accel = (traj.thetav_ - vel[2]) / traj.time_delta_;
  }
  else if (traj.time_delta_ < 0)
  {
    ROS_DEBUG("Negative time delta in jerk cost function. %f.  %s", traj.time_delta_, msg.c_str());
  }
}

} /* namespace base_local_planner */
