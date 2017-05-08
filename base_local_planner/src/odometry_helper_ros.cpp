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
#include <base_local_planner/geometry_math_helpers.h>

namespace base_local_planner {

OdometryHelperRos::OdometryHelperRos(std::string odom_topic) :
    cmd_vel_time_(0.0), forward_estimation_time_(0.067), linear_acceleration_rate_(0.7),
    angular_acceleration_rate_(2.3), cmd_vel_timeout_(0.2),
    wheelbase_(0.5235) {
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

  base_odom_.pose = msg->pose;
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
  robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z),
    tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
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
  nav_msgs::Odometry odom = calculateEstimatedOdometry();
  geometry_msgs::Twist vel = odom.twist.twist;

  robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(vel.angular.z),
    tf::Vector3(vel.linear.x, vel.linear.y, 0)));
  robot_vel.stamp_ = ros::Time();
}

void OdometryHelperRos::getEstimatedOdom(nav_msgs::Odometry& base_odom) {
  base_odom = calculateEstimatedOdometry();
}

void OdometryHelperRos::getEstimatedOdomPose(tf::Stamped<tf::Pose>& pose) {
  nav_msgs::Odometry base_odom = calculateEstimatedOdometry();
  tf::poseMsgToTF(base_odom.pose.pose, pose);
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

void OdometryHelperRos::setWheelbase(double wheelbase)
{
  wheelbase_= wheelbase;
}

void OdometryHelperRos::setForwardEstimationTime(double time)
{
  forward_estimation_time_ = time;
}

nav_msgs::Odometry OdometryHelperRos::calculateEstimatedOdometry()
{
  // Estimate current velocities from odometry and command velocities.
  // Copy out data
  geometry_msgs::Twist cmd_vel;
  double cmd_vel_time = 0;

  geometry_msgs::Twist reported_vel;
  double reported_time = 0;
  nav_msgs::Odometry reported_odom;
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    cmd_vel = cmd_vel_;
    cmd_vel_time = cmd_vel_time_;

    reported_time = base_odom_.header.stamp.toSec();
    reported_vel = base_odom_.twist.twist;
    reported_odom = base_odom_;
  }

  // Get the current time
  double current_time = ros::Time::now().toSec();

  double time_since_measurement = current_time - reported_time; // Unsure if these clocks are the same.

  // Estimate
  if ((current_time - cmd_vel_time) > cmd_vel_timeout_)
  {
    if (cmd_vel_time)
    {
      // It's been too long since a cmd vel was sent.  Use the reported velocity.
      ROS_DEBUG_THROTTLE(1.0f, "cmd vel has a timeout: %f", (current_time - cmd_vel_time));
    }
    return reported_odom;
  }
  else
  {
    double estimate_dt = time_since_measurement + forward_estimation_time_;
    ROS_DEBUG("Estimate dt: %f", estimate_dt);
    return forwardEstimateOdometry(cmd_vel, reported_odom, estimate_dt);
  }
}

nav_msgs::Odometry OdometryHelperRos::forwardEstimateOdometry(geometry_msgs::Twist cmd_vel,
  const nav_msgs::Odometry& current_odom, double estimate_dt)
{
  // Get the current pose and velocity in eigen format.
  Eigen::Vector3f pos = poseToVector3f(current_odom.pose.pose);
  Eigen::Vector3f start_pos = pos;
  Eigen::Vector3f vel = twistToVector3f(current_odom.twist.twist);
  Eigen::Vector3f start_vel = vel;
  Eigen::Vector3f des_vel = twistToVector3f(cmd_vel);

  ROS_DEBUG("Desired: %f, %f", des_vel[0], des_vel[2]);
  double loop_dt = 0.001;
  double t = 0;
  while (t < estimate_dt)
  {
    ROS_DEBUG("updating from [%f, %f, %f] and vel from [%f, %f]",
      pos[0], pos[1], pos[2], vel[0], vel[2]);
    t += loop_dt;
    pos = computeNewPositions(pos, vel, loop_dt / 2);
    vel = computeNewVelocities(des_vel, vel, linear_acceleration_rate_, wheelbase_, loop_dt);
    pos = computeNewPositions(pos, vel, loop_dt / 2);

  ROS_DEBUG("updating to [%f, %f, %f] and vel to [%f, %f]",
    pos[0], pos[1], pos[2], vel[0], vel[2]);
  }

  // Package up the information into an odometry message
  nav_msgs::Odometry output;
  output = current_odom;
  output.header.stamp += ros::Duration(t);
  output.pose.pose = vector3fToPose(pos);
  output.twist.twist = vector3fToTwist(vel);

  ROS_DEBUG("Updated odom pose from [%f, %f, %f] to [%f, %f, %f] and vel from [%f, %f] to [%f, %f]",
    start_pos[0], start_pos[1], start_pos[2], pos[0], pos[1], pos[2],
    start_vel[0], start_vel[2], vel[0], vel[2]);
  return output;
}

Eigen::Vector3f OdometryHelperRos::computeNewPositions(Eigen::Vector3f pos,
    Eigen::Vector3f vel, double dt) {
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

/**
 * cheange vel using acceleration limits to converge towards sample_target-vel
 */
Eigen::Vector3f OdometryHelperRos::computeNewVelocities(Eigen::Vector3f desired_vel,
    Eigen::Vector3f vel, double wheel_limits, double wheelbase, double dt) {
  Eigen::Vector3f new_vel = Eigen::Vector3f::Zero();

  // Calculate current and desired wheel speeds
  double left_ms_des = 0, right_ms_des = 0;
  getWheelGroundSpeedsFromVel(desired_vel, wheelbase, left_ms_des, right_ms_des);

  double left_ms_current = 0, right_ms_current = 0;
  getWheelGroundSpeedsFromVel(vel, wheelbase, left_ms_current, right_ms_current);

  // Limit wheel accelerations
  double dLeft = left_ms_des - left_ms_current;
  double dRight = right_ms_des - right_ms_current;

  // ROS_INFO("Pre-scaling: dLeft %f, dRight %f", dLeft, dRight);
  if (std::fabs(dLeft) > std::fabs(dRight))
  {
    // Scale to the left wheel
    scaleWheelSpeedChanges(dLeft, dRight, wheel_limits, dt);
  }
  else
  {
    // Scale to the right wheel
    scaleWheelSpeedChanges(dRight, dLeft, wheel_limits, dt);
  }
  ROS_DEBUG("Post-scaling: dLeft %f, dRight %f", dLeft, dRight);
  // Convert wheel velocities into body velocities.
  double new_left_ms = left_ms_current + dLeft;
  double new_right_ms = right_ms_current + dRight;

  new_vel = getVelFromWheelGroundSpeeds(wheelbase, new_left_ms, new_right_ms);

  return new_vel;
}

void OdometryHelperRos::scaleWheelSpeedChanges(double& dPrimary, double& dSecondary,
  double accel, double dt)
{
  if (std::fabs(dPrimary) < 0.0001)
  {
    // If the speed is 0, don't do anything.
    return;
  }
  // Make sure that signs work correctly.
  double new_primary = dPrimary;
  if (std::fabs(dPrimary) > accel * dt)
  {
    if (dPrimary > 0)
    {
      new_primary = accel * dt;
    }
    else
    {
      new_primary = -accel * dt;
    }
  }
  dSecondary *= new_primary / dPrimary;
  dPrimary = new_primary;
}

void OdometryHelperRos::getWheelGroundSpeedsFromVel(const Eigen::Vector3f& vel,
  double wheel_base, double& left_ms, double& right_ms)
{
  left_ms = vel[0] - vel[2] * wheel_base / 2;
  right_ms = vel[0] + vel[2] * wheel_base / 2;
}

Eigen::Vector3f OdometryHelperRos::getVelFromWheelGroundSpeeds(double wheel_base,
  double left_ms, double right_ms)
{
  Eigen::Vector3f vel = Eigen::Vector3f::Zero();

  vel[0] = 0.5 * (left_ms + right_ms);  // linear
  if (wheel_base > 0)
  {
    vel[2] = 1.0 / wheel_base * (right_ms - left_ms);
  }
  return vel;
}

} /* namespace base_local_planner */
