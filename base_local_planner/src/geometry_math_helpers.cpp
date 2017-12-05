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

#include <base_local_planner/geometry_math_helpers.h>
#include <ros/ros.h>

namespace base_local_planner {

double distanceToLineSegment(const Eigen::Vector2f& pos,
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

double distanceAlongLineSegment(const Eigen::Vector2f& pos,
  const Eigen::Vector2f& p0, const Eigen::Vector2f& p1)
{
  double l = (p1 - p0).norm();
  if (l == 0.0)
  {
    return 0.0;
  }
  return (pos - p0).dot(p1 - p0) / l;
}

Eigen::Vector2f poseAtDistanceAlongLineSegment(double distance,
  const Eigen::Vector2f& p0, const Eigen::Vector2f& p1)
{
  double l = (p1 - p0).norm();
  if (l == 0.0)
  {
    return p1;
  }

  double t = distance / l;

  Eigen::Vector2f projection = p0 + t * (p1 - p0);

  ROS_DEBUG("paDtLS: p0 %f,%f p1 %f, %f, dist %f, t %f, proje %f %f",
    p0[0], p0[1], p1[0], p1[1], distance, t, projection[0], projection[1]);
  return projection;
}

Eigen::Vector2f poseStampedToVector(geometry_msgs::PoseStamped pose)
{
  Eigen::Vector2f p = Eigen::Vector2f::Zero();
  p[0] = pose.pose.position.x;
  p[1] = pose.pose.position.y;
  return p;
}

double angleMinusPiToPi(double val)
{
  double output = val;
  while (output > M_PI)
  {
    output -= 2 * M_PI;
  }
  while (output < -M_PI)
  {
    output += 2 * M_PI;
  }
  return output;
}

Eigen::Vector3f poseToVector3f(const geometry_msgs::Pose& pose)
{
  Eigen::Vector3f p = Eigen::Vector3f::Zero();
  p[0] = pose.position.x;
  p[1] = pose.position.y;
  p[2] = tf::getYaw(pose.orientation);
  return p;
}

Eigen::Vector3f twistToVector3f(const geometry_msgs::Twist& t)
{
  Eigen::Vector3f v = Eigen::Vector3f::Zero();
  v[0] = t.linear.x;
  v[1] = t.linear.y;
  v[2] = t.angular.z;
  return v;
}


geometry_msgs::Pose vector3fToPose(const Eigen::Vector3f& vec)
{
  geometry_msgs::Pose p;
  p.position.x = vec[0];
  p.position.y = vec[1];
  p.orientation = tf::createQuaternionMsgFromYaw(vec[2]);
  return p;
}

geometry_msgs::Twist vector3fToTwist(const Eigen::Vector3f& vec)
{
  geometry_msgs::Twist t;
  t.linear.x = vec[0];
  t.linear.y = vec[1];
  t.angular.z = vec[2];
  return t;
}


double lerp(double value, double min_value, double max_value, double min_output, double max_output)
{
    double ratio = (value - min_value) / (max_value - min_value);
    return ratio * max_output + (1.0 - ratio) * min_output;
}

double twoLevelInterpolation(double value, 
  double min_value, double max_value,
  double min_output, double max_output)
{
  // Turn this into easier to understand logic with 2 plateaus.
  if (value < min_value)
  {
    return min_output;
  }
  else if (value >= min_value && value < max_value)
  {
    return lerp(value, min_value, max_value, min_output, max_output);
  }
  else
  {
    return max_output;
  }
}


double threeLevelInterpolation(double value, 
  double min_value, double nominal_value_low, 
  double nominal_value_high, double max_value,
  double min_output, double nominal_output, double max_output)
{
  // Turn this into easier to understand logic with 3 plateaus.
  if (value < min_value)
  {
    return min_output;
  }
  else if (value >= min_value && value < nominal_value_low)
  {
    return lerp(value, min_value, nominal_value_low, min_output, nominal_output);
  }
  else if (value >= nominal_value_low && value < nominal_value_high)
  {
    return nominal_output;
  }
  else if (value >= nominal_value_high && value < max_value)
  {
    return lerp(value, nominal_value_high, max_value, nominal_output, max_output);
  }
  else
  {
    return max_output;
  }
}

}