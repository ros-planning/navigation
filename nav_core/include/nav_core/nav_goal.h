/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Clearpath Robotics, Inc.
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
* Author: Jason Mercer
*********************************************************************/

#ifndef NAV_NAV_CORE_NAV_GOAL_H_
#define NAV_NAV_CORE_NAV_GOAL_H_

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <angles/angles.h>

namespace nav_core
{

class NavGoal
{
public:
  typedef size_t id_type;

  NavGoal()
    : id_(0)
  {
  }

  NavGoal(geometry_msgs::PoseStamped p)
    : id_(0)
  {
    pose_ = p;
  }

  bool equalPose(const NavGoal& g) const
  {
    return similarPose(g, 0, 0);
  }

  bool similarPose(const NavGoal& g, float dist_tol, float angle_tol_rad) const
  {
    const geometry_msgs::Pose& p1 = pose_.pose;
    const geometry_msgs::Pose& p2 = g.pose_.pose;

    const float xx = square(p1.position.x - p2.position.x);
    const float yy = square(p1.position.y - p2.position.y);
    const float zz = square(p1.position.z - p2.position.z);
    const float rr = xx + yy + zz;

    if (rr > square(dist_tol))
    {
      return false;
    }

    return angles::shortest_angular_distance(tf::getYaw(p2.orientation), tf::getYaw(p1.orientation)) <= angle_tol_rad;
  }

  geometry_msgs::PoseStamped pose_;
  id_type id_;

private:
  inline float square(float x) const
  {
    return x*x;
  }
};


}  // namespace nav_core

#endif  // NAV_NAV_CORE_NAV_GOAL_H_
