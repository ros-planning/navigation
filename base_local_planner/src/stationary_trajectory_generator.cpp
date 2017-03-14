/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017 6 River Systems.
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
 * Author: dgrieneisen
 *********************************************************************/

#include <base_local_planner/stationary_trajectory_generator.h>
#include <ros/ros.h>
#include <cmath>

#include <base_local_planner/velocity_iterator.h>

namespace base_local_planner {

void StationaryTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    base_local_planner::LocalPlannerLimits* limits) {

  // Stored
  pos_ = pos;
  vel_ = vel;
  limits_ = limits;

  trajectory_generated_ = false;
}

/**
 * Whether this generator can create more trajectories
 */
bool StationaryTrajectoryGenerator::hasMoreTrajectories() {
  return enabled_ && !trajectory_generated_;
}

/**
 * Create and return the next sample trajectory
 */
bool StationaryTrajectoryGenerator::nextTrajectory(Trajectory &comp_traj) {
  bool result = false;
  if (hasMoreTrajectories()) {
    if (generateTrajectory(
        pos_,
        vel_,
        comp_traj)) {
      result = true;
    }
  }
  return result;
}

/**
 * @param pos current position of robot
 * @param vel desired velocity for sampling
 */
bool StationaryTrajectoryGenerator::generateTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      base_local_planner::Trajectory& traj) {

  trajectory_generated_ = true;
  traj.cost_   = 100.0; // placed here in case we return early

  double epsilon = 0.01;
  if (std::fabs(vel[0]) < epsilon &&
      std::fabs(vel[1]) < epsilon &&
      std::fabs(vel[2]) < epsilon)
  {
    ROS_DEBUG("Generating trajectory at %f, %f, %f.", pos[0], pos[1], pos[2]);
    traj.resetPoints();
    traj.addPoint(pos[0], pos[1], pos[2], 0.0, 0.0, 0.0);
    traj.xv_ = 0.0;
    traj.yv_ = 0.0;
    traj.thetav_ = 0.0;
    traj.time_delta_ = 0.1;
    return true;
  }

  return false;
}

} /* namespace base_local_planner */

