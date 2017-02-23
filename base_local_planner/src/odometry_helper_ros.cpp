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

OdometryHelperRos::OdometryHelperRos(std::string odom_topic) :
    cmd_vel_time_(0.0), velocity_loop_delays_(0.067), linear_acceleration_rate_(0.7),
    angular_acceleration_rate_(2.3), cmd_vel_timeout_(0.2) {
  setOdomTopic( odom_topic );
}

void OdometryHelperRos::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO_ONCE("odom received!");

  //we assume that the odometry is published in the frame of the base
  boost::mutex::scoped_lock lock(odom_mutex_);
  base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
  base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
  base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
  base_odom_.child_frame_id = msg->child_frame_id;
  // Store the header too (for time stamp purposes)
  base_odom_.header = msg->header;
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

void OdometryHelperRos::getEstimatedRobotVel(tf::Stamped<tf::Pose>& robot_vel) {
  // Set current velocities from odometry
  geometry_msgs::Twist global_vel = estimateRobotVel();

  robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
  robot_vel.stamp_ = ros::Time();
}

geometry_msgs::Twist OdometryHelperRos::estimateRobotVel()
{
    // Estimate current velocities from odometry and command velocities.
    // Copy out data
    geometry_msgs::Twist cmd_vel;
    double cmd_vel_time = 0;

    geometry_msgs::Twist reported_vel;
    double reported_time = 0;
    {
      boost::mutex::scoped_lock lock(odom_mutex_);
      cmd_vel = cmd_vel_;
      cmd_vel_time = cmd_vel_time_;

      reported_time = base_odom_.header.stamp.toSec();
      reported_vel = base_odom_.twist.twist;
    }

    // Get the current time
    double current_time = ros::Time::now().toSec();

    double time_since_measurement = current_time - reported_time; // Unsure if these clocks are the same.

    // Estimate
    geometry_msgs::Twist estimated_vel;
    if ((current_time - cmd_vel_time) > cmd_vel_timeout_)
    {
      if (cmd_vel_time)
      {
        // It's been too long since a cmd vel was sent.  Use the reported velocity.
        ROS_DEBUG_THROTTLE(1.0f, "cmd vel has a timeout: %f", (current_time - cmd_vel_time));
      }
      estimated_vel.linear.x = reported_vel.linear.x;
      estimated_vel.angular.z = reported_vel.angular.z;
    }
    else
    {
      double estimate_dt = time_since_measurement + velocity_loop_delays_;
      ROS_DEBUG("Estimate dt: %f", estimate_dt);
      estimated_vel.linear.x = forwardEstimateVelocity(reported_vel.linear.x, cmd_vel.linear.x, linear_acceleration_rate_, estimate_dt);
      estimated_vel.angular.z = forwardEstimateVelocity(reported_vel.angular.z, cmd_vel.angular.z, angular_acceleration_rate_, estimate_dt);
    }
    return estimated_vel;
}

double OdometryHelperRos::forwardEstimateVelocity(double old, double cmd, double accel, double dt)
{
    // Make sure that signs work correctly.
    double res = 0;
    if (old > cmd)
    {
      res = std::max(old - accel*dt, cmd);
    }
    else
    {
      res = std::min(old + accel*dt, cmd);
    }
    return res;
}

void OdometryHelperRos::setCmdVel(geometry_msgs::Twist vel)
{
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    cmd_vel_ = vel;
    cmd_vel_time_ = ros::Time::now().toSec();
  }
}

void OdometryHelperRos::setAccelerationRates(double linear, double angular)
{
  linear_acceleration_rate_ = linear;
  angular_acceleration_rate_ = angular;
}

void OdometryHelperRos::setExpectedVelocityLoopDelay(double delay)
{
  velocity_loop_delays_ = delay;
}

} /* namespace base_local_planner */
