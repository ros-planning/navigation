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
#include <base_local_planner/geometry_math_helpers.h>

namespace base_local_planner
{

geometry_msgs::Pose createPose(float x, float y, float yaw)
{
  geometry_msgs::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return p;
}


geometry_msgs::PoseWithCovariance createPoseWithCovariance(float x, float y, float yaw)
{
  geometry_msgs::PoseWithCovariance p;
  p.pose = createPose(x, y, yaw);
  return p;
}

geometry_msgs::PoseStamped createPoseStamped(float x, float y, float yaw)
{
  geometry_msgs::PoseStamped p;
  p.pose = createPose(x, y, yaw);
  return p;
}

geometry_msgs::Twist createTwist(float v, float w)
{
  geometry_msgs::Twist t;
  t.linear.x = v;
  t.angular.z = w;
  return t;
}

geometry_msgs::TwistStamped createTwistStamped(float v, float w)
{
  geometry_msgs::TwistStamped t;
  t.twist = createTwist(v, w);
  return t;
}

geometry_msgs::TwistWithCovariance createTwistWithCovariance(float v, float w)
{
  geometry_msgs::TwistWithCovariance t;
  t.twist = createTwist(v, w);
  return t;
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

nav_msgs::Odometry createOdometry(double x, double y, double yaw, double v, double w)
{
  nav_msgs::Odometry odom;
  odom.pose = createPoseWithCovariance(x, y, yaw);
  odom.twist = createTwistWithCovariance(v, w);
  return odom;
}

bool vector2DEqual(Eigen::Vector2f a, Eigen::Vector2f b)
{
  double epsilon = 0.0001;
  return (std::fabs(a[0] - b[0]) < epsilon
    && std::fabs(a[1] - b[1]) < epsilon);
}

bool vector3fEqual(Eigen::Vector3f a, Eigen::Vector3f b)
{
  std::cerr << "Comparing " << printVector3f(a) << " and " << printVector3f(b) << std::endl;;
  double epsilon = 0.0001;
  return (std::fabs(a[0] - b[0]) < epsilon
    && std::fabs(a[1] - b[1]) < epsilon
    && std::fabs(a[2] - b[2]) < epsilon);
}

bool poseEqual(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  return vector3fEqual(poseToVector3f(a), poseToVector3f(b));
}

bool twistEqual(geometry_msgs::Twist a, geometry_msgs::Twist b)
{
  return vector3fEqual(twistToVector3f(a), twistToVector3f(b));
}

bool odometryEqual(nav_msgs::Odometry a, nav_msgs::Odometry b)
{
  std::cerr << "Comparing " << printOdometry(a) << " and " << printOdometry(b) << std::endl;
  return (poseEqual(a.pose.pose, b.pose.pose)
    && twistEqual(a.twist.twist, b.twist.twist));
}

std::string printVector3f(Eigen::Vector3f a)
{
  std::stringstream ss;
  ss << "[" << a[0] << ", " << a[1] << ", " << a[2] << "]";
  return ss.str();
}

std::string printOdometry(nav_msgs::Odometry odom)
{
  std::stringstream ss;
  ss << "[Odom: pose: " << printVector3f(poseToVector3f(odom.pose.pose))
    << ", vel: " << printVector3f(twistToVector3f(odom.twist.twist))
    << "]";

  return ss.str();
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


}
