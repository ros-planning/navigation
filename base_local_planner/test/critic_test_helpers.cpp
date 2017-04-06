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
#include "critic_test_helpers.h"

geometry_msgs::PoseStamped createPoseStamped(float x, float y, float yaw)
{
  geometry_msgs::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return p;
}

std::vector<geometry_msgs::PoseStamped> createGlobalPlan()
{
  // Create a global plan
  std::vector<geometry_msgs::PoseStamped> out;

  // Create a straight line down the x axis
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  for (x = 0.0; x <= 5.0; x += 0.1)
  {
    out.push_back(createPoseStamped(x, y, yaw));
  }
  return out;
}

base_local_planner::Trajectory createTrajectory(double v, double w)
{
  return base_local_planner::Trajectory(v, 0, w, 0.1, 1);
}

Eigen::Vector3f createVector(float x, float y, float yaw)
{
  Eigen::Vector3f out = Eigen::Vector3f::Zero();
  out[0] = x;
  out[1] = y;
  out[2] = yaw;
  return out;
}

Eigen::Vector2f create2DVector(float x, float y)
{
  Eigen::Vector2f out = Eigen::Vector2f::Zero();
  out[0] = x;
  out[1] = y;
  return out;
}

bool vector2DEqual(Eigen::Vector2f a, Eigen::Vector2f b)
{
  double epsilon = 0.0001;
  return (std::fabs(a[0] - b[0]) < epsilon && std::fabs(a[0] - b[0]) < epsilon);
}

void printTrajectory(const base_local_planner::Trajectory& traj)
{
  std::stringstream ss;

  ss << "Print trajectory" << std::endl
    << " xv: " << traj.xv_ << ", thetav: " << traj.thetav_ << std::endl
    << " cost: " << traj.cost_ << ", td: " << traj.time_delta_ << ", samples: " << traj.getPointsSize() << std::endl;

  for (unsigned int k = 0; k < traj.getPointsSize(); ++k)
  {
    double x, y, th;
    traj.getPoint(k, x, y, th);
    double vx, vy, vth;
    traj.getVelocity(k, vx, vy, vth);
    ss << "  " << k << ": [" << x << ", " << y << ", " << th << "] ["
      << vx << ", " << vy << ", " << vth << "]" << std::endl;
  }
  std::cerr << ss.str();
}
