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
  //to get odometry information, we need to get a handle to the topic in the global namespace of the node
  ros::NodeHandle gn;
  odom_sub_ = gn.subscribe<nav_msgs::Odometry>(odom_topic, 1, boost::bind(&OdometryHelperRos::odomCallback, this, _1));
}

void OdometryHelperRos::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  //we assume that the odometry is published in the frame of the base
  boost::mutex::scoped_lock lock(odom_mutex_);
  base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
  base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
  base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
//  ROS_DEBUG_NAMED("dwa_local_planner", "In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
//      base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
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
  }
  robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
  robot_vel.frame_id_ = frame_id_;//costmap_ros_->getBaseFrameID();
  robot_vel.stamp_ = ros::Time();
}

} /* namespace base_local_planner */
