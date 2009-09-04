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
*   * Redistributions of source code must retain the above copyright
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

#include <string>
#include <gtest/gtest.h>
#include "ros/node.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


using namespace ros;


static const double time_end     = 1250701262.0;
static const double ekf_duration = 20.0;
static const double EPS_trans_x    = 0.02;
static const double EPS_trans_y    = 0.04;
static const double EPS_trans_z    = 0.00001;
static const double EPS_rot_x      = 0.005;
static const double EPS_rot_y      = 0.005;
static const double EPS_rot_z      = 0.005;
static const double EPS_rot_w      = 0.005;


int g_argc;
char** g_argv;

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
typedef boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> EkfConstPtr;

class TestEKF : public testing::Test
{
public:
  NodeHandle node_;
  ros::Subscriber odom_sub_, ekf_sub_;
  EkfConstPtr ekf_begin_, ekf_end_;
  OdomConstPtr odom_end_;
  double ekf_counter_, odom_counter_;
  Time ekf_time_begin_, odom_time_begin_;


  void OdomCallback(const OdomConstPtr& odom)
  {
    // get initial time
    if (odom_counter_ == 0)
      odom_time_begin_ = odom->header.stamp;

    odom_end_ = odom;

    // count number of callbacks
    odom_counter_++;
  }


  void EKFCallback(const EkfConstPtr& ekf)
  {
    // get initial time
    if (ekf_counter_ == 0){
      ekf_time_begin_ = ekf->header.stamp;
      ekf_begin_ = ekf;
    }
    if (ekf->header.stamp.toSec() < time_end)
      ekf_end_ = ekf;

    // count number of callbacks
    ekf_counter_++;
  }


protected:
  /// constructor
  TestEKF()
  {
    ekf_counter_ = 0;
    odom_counter_ = 0;

  }


  /// Destructor
  ~TestEKF()
  {
  }
};




TEST_F(TestEKF, test)
{
  ROS_INFO("Subscribing to robot_pose_ekf/odom_combined");
  ekf_sub_ = node_.subscribe("/robot_pose_ekf/odom_combined", 10, &TestEKF::EKFCallback, (TestEKF*)this);

  ROS_INFO("Subscribing to pr2_base_odometry/odom");
  odom_sub_ = node_.subscribe("/pr2_base_odometry/odom", 10 , &TestEKF::OdomCallback, (TestEKF*)this);

  // wait while bag is played back
  ROS_INFO("Waiting for bag to start playing");
  while (odom_counter_ == 0)
    usleep(1e4);
  ROS_INFO("Detected that bag is playing");

  ROS_INFO("Waiting untile end time is reached");
  while( odom_end_->header.stamp.toSec() < time_end){
    usleep(1e4);
  }
  ROS_INFO("End time reached");
  // give filter some time to catch up
  Duration(2.0).sleep();

  // check if callback was called enough times
  EXPECT_GT(ekf_counter_, 200);

  // check if time interval is correct
  EXPECT_LT(Duration(ekf_end_->header.stamp - ekf_time_begin_).toSec(), ekf_duration * 1.25);
  EXPECT_GT(Duration(ekf_end_->header.stamp - ekf_time_begin_).toSec(), ekf_duration * 0.85);

  // check if ekf time is same as odom time
  EXPECT_NEAR(ekf_time_begin_.toSec(),  odom_time_begin_.toSec(), 1.0);
  EXPECT_NEAR(ekf_end_->header.stamp.toSec(), time_end, 1.0);

  // check filter result
  ROS_INFO("%f %f %f %f %f %f %f -- %f",ekf_end_->pose.pose.position.x, ekf_end_->pose.pose.position.y, ekf_end_->pose.pose.position.z,
            ekf_end_->pose.pose.orientation.x, ekf_end_->pose.pose.orientation.y, ekf_end_->pose.pose.orientation.z, ekf_end_->pose.pose.orientation.w,
            ekf_end_->header.stamp.toSec());
  EXPECT_NEAR(ekf_end_->pose.pose.position.x, 3.835345, EPS_trans_x);
  EXPECT_NEAR(ekf_end_->pose.pose.position.y, 0.556243, EPS_trans_y);
  EXPECT_NEAR(ekf_end_->pose.pose.position.z, 0.000000, EPS_trans_z);
  EXPECT_NEAR(ekf_end_->pose.pose.orientation.x, 0.003916,  EPS_rot_x);
  EXPECT_NEAR(ekf_end_->pose.pose.orientation.y, -0.005158, EPS_rot_y);
  EXPECT_NEAR(ekf_end_->pose.pose.orientation.z, 0.666300,  EPS_rot_z);
  EXPECT_NEAR(ekf_end_->pose.pose.orientation.w, 0.745655,  EPS_rot_w);

  SUCCEED();
}




int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;

  init(g_argc, g_argv, "testEKF");

  boost::thread(boost::bind(&ros::spin));

  return RUN_ALL_TESTS();
}
