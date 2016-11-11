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
#include <base_local_planner/odometry_helper_ros.h>

namespace base_local_planner {

OdometryHelperRos::OdometryHelperRos(std::string odom_topic) {
  setOdomTopic( odom_topic );
}

OdometryHelperRos::OdometryHelperRos(std::string odom_topic, std::string cmd_vel_topic) {
  setOdomTopic( odom_topic );
  setCmdVelTopic( cmd_vel_topic );
}

void OdometryHelperRos::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO_ONCE("odom received!");

  //we assume that the odometry is published in the frame of the base
  boost::mutex::scoped_lock lock(odom_mutex_);
  base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
  base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
  base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
  base_odom_.child_frame_id = msg->child_frame_id;
  base_odom_.header = msg->header;
//  ROS_DEBUG_NAMED("dwa_local_planner", "In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
//      base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
}

void OdometryHelperRos::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO_ONCE("cmd vel received!");

  boost::mutex::scoped_lock lock(odom_mutex_);
  cmd_vel_ = *msg;
  cmd_vel_time_ = ros::Time::now().toSec();  // Record whatever time it is.
}

void OdometryHelperRos::setCmdVel(const geometry_msgs::Twist& msg) {
  boost::mutex::scoped_lock lock(odom_mutex_);
  cmd_vel_ = msg;
  cmd_vel_time_ = ros::Time::now().toSec();  // Record whatever time it is.
}

//copy over the odometry information
void OdometryHelperRos::getOdom(nav_msgs::Odometry& base_odom) {
  boost::mutex::scoped_lock lock(odom_mutex_);
  base_odom = base_odom_;
}

void OdometryHelperRos::getRobotVel(tf::Stamped<tf::Pose>& robot_vel) {
  // Set current velocities from odometry
  geometry_msgs::Twist global_vel;
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    global_vel.linear.x = base_odom_.twist.twist.linear.x;
    global_vel.linear.y = base_odom_.twist.twist.linear.y;
    global_vel.angular.z = base_odom_.twist.twist.angular.z;

    robot_vel.frame_id_ = base_odom_.child_frame_id;
  }
  robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
  robot_vel.stamp_ = ros::Time();
}

void OdometryHelperRos::getEstimatedRobotVel(tf::Stamped<tf::Pose>& robot_vel) {
  // Estimate current velocities from odometry and command velocities.
  // Copy out data
  geometry_msgs::Twist reported_vel;
  double reported_vel_time = 0;
  geometry_msgs::Twist cmd_vel;
  double cmd_vel_time = 0;
  std::string frame_id = "";
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    reported_vel.linear.x = base_odom_.twist.twist.linear.x;
    reported_vel.linear.y = base_odom_.twist.twist.linear.y;
    reported_vel.angular.z = base_odom_.twist.twist.angular.z;
    reported_vel_time = base_odom_.header.stamp.sec + 1e-9 * base_odom_.header.stamp.nsec;

    cmd_vel = cmd_vel_;
    cmd_vel_time = cmd_vel_time_;

    frame_id = base_odom_.child_frame_id;
  }
  // Get the current time
  double current_time = ros::Time::now().toSec();

  // Hard-coded values (for now)
  double cmd_vel_timeout = 0.5;
  double linear_acceleration_rate = 0.7; // m/s^2 (from the firmware)
  double angular_acceleration_rate = 2.63; // rad/s^2 (from the firmware)
  double forward_estimation_time = 0.05; // The control cycle takes time.  Estimate what the velocity will be when the command is output.
  double velocity_response_delay = 0.1;  // Time lag in getting back velocities (from measurements)

  // Estimate
  geometry_msgs::Twist estimated_vel;
  if ((current_time - cmd_vel_time) > cmd_vel_timeout) {
    // It's been too long since a cmd vel was sent.  Use the reported velocity.
    ROS_DEBUG("cmd vel timeout.");
    estimated_vel = reported_vel;
  } else {
    double estimate_dt = (current_time + forward_estimation_time) - (reported_vel_time - velocity_response_delay);
    estimated_vel.linear.x = estimateVelocity(reported_vel.linear.x, cmd_vel.linear.x, linear_acceleration_rate, estimate_dt);
    estimated_vel.linear.y = estimateVelocity(reported_vel.linear.y, cmd_vel.linear.y, linear_acceleration_rate, estimate_dt);
    estimated_vel.angular.z = estimateVelocity(reported_vel.angular.z, cmd_vel.angular.z, angular_acceleration_rate, estimate_dt);
  }

  robot_vel.frame_id_ = frame_id;
  robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(estimated_vel.angular.z), tf::Vector3(estimated_vel.linear.x, estimated_vel.linear.y, 0)));
  robot_vel.stamp_ = ros::Time();
}

double OdometryHelperRos::estimateVelocity(double old, double cmd, double accel, double dt) {
  // Make sure that signs work correctly.
  double res = 0;
  if (old > cmd) {
    res = std::max(old - accel*dt, cmd);
  } else {
    res = std::min(old + accel*dt, cmd);
  }
  return res;
}

void OdometryHelperRos::setOdomTopic(std::string odom_topic)
{
  if( odom_topic != odom_topic_ )
  {
    odom_topic_ = odom_topic;

    if( odom_topic_ != "" )
    {
      ros::NodeHandle gn;
      odom_sub_ = gn.subscribe<nav_msgs::Odometry>( odom_topic_, 1, boost::bind( &OdometryHelperRos::odomCallback, this, _1 ));
    }
    else
    {
      odom_sub_.shutdown();
    }
  }
}

void OdometryHelperRos::setCmdVelTopic(std::string cmd_vel_topic)
{
  if( cmd_vel_topic != cmd_vel_topic_ )
  {
    cmd_vel_topic_ = cmd_vel_topic;

    if( cmd_vel_topic_ != "" )
    {
      ros::NodeHandle gn;
      cmd_vel_sub_ = gn.subscribe<geometry_msgs::Twist>( cmd_vel_topic_, 1, boost::bind( &OdometryHelperRos::cmdVelCallback, this, _1 ));
    }
    else
    {
      cmd_vel_sub_.shutdown();
    }
  }
}
} /* namespace base_local_planner */
