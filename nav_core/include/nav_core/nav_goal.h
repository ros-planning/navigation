/**
Software License Agreement (proprietary)

\file      nav_goal.h
\authors   Jason Mercer <jmercer@clearpathrobotics.com>
\copyright Copyright (c) 2016, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/

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

    return angles::normalize_angle(tf::getYaw(p2.orientation) - tf::getYaw(p1.orientation)) <= angle_tol_rad;
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
