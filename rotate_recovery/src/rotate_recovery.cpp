/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <rotate_recovery/rotate_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>


// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(rotate_recovery::RotateRecovery, nav_core::RecoveryBehavior)

namespace rotate_recovery
{
RotateRecovery::RotateRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void RotateRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name);

    // we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);
    private_nh.param("acc_lim_theta", acc_lim_th_, 3.2);
    private_nh.param("max_vel_theta", max_rotational_vel_, 1.0);
    private_nh.param("min_in_place_vel_theta", min_rotational_vel_, 0.4);
    private_nh.param("yaw_goal_tolerance", tolerance_, 0.10);
    private_nh.param("rotation_angle", rotation_angle_, 2 * M_PI);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

RotateRecovery::~RotateRecovery()
{
  delete world_model_;
}

void RotateRecovery::runBehavior()
{
  runBehavior(rotation_angle_);
}

bool RotateRecovery::runBehavior(const double rotation_angle)
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return false;
  }

  if (local_costmap_ == NULL)
  {
    ROS_ERROR("The costmap passed to the RotateRecovery object cannot be NULL. Doing nothing.");
    return false;
  }
  ROS_WARN("Rotate recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  geometry_msgs::PoseStamped global_pose;
  local_costmap_->getRobotPose(global_pose);

  double rotated_angle = 0.0;

  double prev_yaw = tf2::getYaw(global_pose.pose.orientation);
  const double dir = rotation_angle > 0 ? 1.0 : -1.0;
  const double max_stopping_distance =
      acc_lim_th_ > 0 ? max_rotational_vel_ * max_rotational_vel_ / (2.0 * acc_lim_th_) : M_PI_4;

  while(n.ok())
  {
    local_costmap_->getRobotPose(global_pose);

    const double current_yaw = tf2::getYaw(global_pose.pose.orientation);
    rotated_angle += dir * angles::normalize_angle(current_yaw - prev_yaw);
    prev_yaw = current_yaw;

    // compute the distance left to rotate
    double dist_left = std::abs(rotation_angle) - rotated_angle;

    const double x = global_pose.pose.position.x, y = global_pose.pose.position.y;

    // check if that velocity is legal by forward simulating
    double sim_angle = 0.0;
    while (sim_angle < dist_left)
    {
      const double theta = current_yaw + dir * sim_angle;

      // make sure that the point is legal, if it isn't... we'll abort
      double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
      if(footprint_cost < 0.0)
      {
        // only allow to rotate to the obstacle, minus some buffer
        dist_left = sim_angle - 2 * max_stopping_distance;
        if (dist_left <= 0) {
          ROS_ERROR("Rotate recovery stops rotating in place because there is a potential collision. Cost: %.2f", footprint_cost);
          geometry_msgs::Twist cmd_vel;
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;

           vel_pub.publish(cmd_vel);
          return false;
        }
        ROS_DEBUG("Rotate recovery slowing down because there is a potential collision %.2f [rad] away", sim_angle);
        break;
      }

      sim_angle += sim_granularity_;
    }

    // compute the velocity that will let us stop by the time we reach the goal
    double vel = dist_left > 0 ? sqrt(2 * acc_lim_th_ * dist_left) : 0;

    // make sure that this velocity falls within the specified limits
    vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = dir * vel;

    vel_pub.publish(cmd_vel);

    // if we're done with our in-place rotation... then return
    if(rotated_angle >= std::abs(rotation_angle) - tolerance_) {
      cmd_vel.angular.z = 0.0;
      vel_pub.publish(cmd_vel);
      return true;
    }

    r.sleep();
  }
  return false;
}
};  // namespace rotate_recovery
