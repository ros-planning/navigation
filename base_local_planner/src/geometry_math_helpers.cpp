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

Eigen::Vector2f poseStampedToVector(geometry_msgs::PoseStamped pose)
{
  Eigen::Vector2f p = Eigen::Vector2f::Zero();
  p[0] = pose.pose.position.x;
  p[1] = pose.pose.position.y;
  return p;
}

double angleMinusPiToPi(double val)
{
  while (val > M_PI)
  {
    val -= 2 * M_PI;
  }
  while (val < -M_PI)
  {
    val += 2 * M_PI;
  }
}

}