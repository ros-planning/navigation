/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/velocity_cost_function.h>

namespace base_local_planner {

VelocityCostFunction::VelocityCostFunction() :
    goal_distance_squared_(16.0),
    min_goal_distance_squared_(1.0),
    max_linear_velocity_(0.0),
    min_linear_velocity_(0.0),
    EPSILON(0.001) {}

bool VelocityCostFunction::prepare() {
  // Don't prepare anything.
  return true;
}

double VelocityCostFunction::scoreTrajectory(Trajectory &traj) {
  if (goal_distance_squared_ < min_goal_distance_squared_)
  {
    return 0.0;
  }

  if (std::fabs(max_linear_velocity_) < EPSILON)
  {
    return 0.0;
  }

  double numerator = numerator = std::fabs(traj.xv_);
  // if (traj.xv_ < min_linear_velocity_)
  // {
  //   numerator = std::fabs(traj.xv_);
  // }
  // else
  // {
  //   numerator = std::fabs(traj.xv_) + std::fabs(traj.thetav_);
  // }

  double ratio =  numerator / max_linear_velocity_;

  if (ratio < 1)
  {
    return 1.0 - ratio;
  }

  return 0.0;
}

} /* namespace base_local_planner */
