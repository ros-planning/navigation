/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

/* Author: Wim Meeussen */

#ifndef __ODOM_ESTIMATION_NODE__
#define __ODOM_ESTIMATION_NODE__

// ros stuff
#include <ros/node.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "odom_estimation.h"

// messages
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <boost/thread/mutex.hpp>

// log files
#include <fstream>

namespace estimation
{

/** \mainpage
 *  \htmlinclude manifest.html
 * 
 * <b>Package Summary</b>
 * This package provides two main classes: 
 *  1) OdomEstimation performs all sensor fusion operations, and 
 *  2) OdomEstimationNode provides a ROS wrapper around OdomEstimation
*/

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
typedef boost::shared_ptr<sensor_msgs::Imu const> ImuConstPtr;
typedef boost::shared_ptr<nav_msgs::Odometry const> VoConstPtr;
typedef boost::shared_ptr<geometry_msgs::Twist const> VelConstPtr;

class OdomEstimationNode
{
public:
  /// constructor
  OdomEstimationNode();

  /// destructor
  virtual ~OdomEstimationNode();

private:
  /// the mail filter loop that will be called periodically
  void spin(const ros::TimerEvent& e);

  /// callback function for vel data
  void velCallback(const VelConstPtr& vel);

  /// callback function for odo data
  void odomCallback(const OdomConstPtr& odom);

  /// callback function for imu data
  void imuCallback(const ImuConstPtr& imu);

  /// callback function for vo data
  void voCallback(const VoConstPtr& vo);


  ros::NodeHandle node_;
  ros::Timer timer_;
  ros::Publisher pose_pub_;
  ros::Subscriber cmd_vel_sub_, odom_sub_, imu_sub_, vo_sub_;

  // ekf filter
  OdomEstimation my_filter_;

  // estimated robot pose message to send
  geometry_msgs::PoseWithCovarianceStamped  output_; 

  // robot state
  tf::TransformListener    robot_state_;
  tf::TransformBroadcaster odom_broadcaster_;

  // vectors
  MatrixWrapper::ColumnVector vel_desi_;
  tf::Transform odom_meas_, imu_meas_, vo_meas_;
  tf::Transform base_vo_init_, vo_camera_;
  tf::Stamped<tf::Transform> camera_base_;
  ros::Time odom_time_, imu_time_, vo_time_;
  ros::Time odom_stamp_, imu_stamp_, vo_stamp_, filter_stamp_;
  ros::Time odom_init_stamp_, imu_init_stamp_, vo_init_stamp_;
  bool vel_active_, odom_active_, imu_active_, vo_active_;
  bool odom_used_, imu_used_, vo_used_;
  bool odom_initializing_, imu_initializing_, vo_initializing_;
  double timeout_;
  MatrixWrapper::SymmetricMatrix odom_covariance_, imu_covariance_, vo_covariance_;

  // mutex
  boost::mutex odom_mutex_, imu_mutex_, vo_mutex_, vel_mutex_;

  // log files for debugging
  std::ofstream odom_file_, imu_file_, vo_file_, corr_file_, time_file_, extra_file_;


}; // class

}; // namespace

#endif
