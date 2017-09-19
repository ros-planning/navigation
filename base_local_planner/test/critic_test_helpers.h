/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, 6 River Systems
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
 * Author: Daniel Grieneisen
 *********************************************************************/
#ifndef BASE_LOCAL_PLANNER_CRITIC_TEST_HELPERS_H_
#define BASE_LOCAL_PLANNER_CRITIC_TEST_HELPERS_H_

#include <base_local_planner/trajectory.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <costmap_2d/ObstructionMsg.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>

namespace base_local_planner
{

geometry_msgs::Pose createPose(float x, float y, float yaw);
geometry_msgs::PoseWithCovariance createPoseWithCovariance(float x, float y, float yaw);
geometry_msgs::PoseStamped createPoseStamped(float x, float y, float yaw);

tf::Pose createTfPose(float x, float y, float yaw);
tf::Stamped<tf::Pose> createTfPoseStamped(float x, float y, float yaw, std::string frame = "");

geometry_msgs::Twist createTwist(float v, float w);
geometry_msgs::TwistWithCovariance createTwistWithCovariance(float v, float w);
geometry_msgs::TwistStamped createTwistStamped(float v, float w);

nav_msgs::Odometry createOdometry(double x, double y, double yaw, double v, double w);

std::vector<geometry_msgs::Point> createFootprint(double length, double width);

std::vector<geometry_msgs::PoseStamped> createGlobalPlan();

base_local_planner::Trajectory createTrajectory(double v, double w);

Eigen::Vector3f createVector(float x, float y, float yaw);

Eigen::Vector2f create2DVector(float x, float y);

costmap_2d::ObstructionMsg createObstructionMsg(double x, double y, std::string frame_id, int type, bool cleared);

bool vector2DEqual(Eigen::Vector2f a, Eigen::Vector2f b);
bool vector3fEqual(Eigen::Vector3f a, Eigen::Vector3f b);
bool poseEqual(geometry_msgs::Pose a, geometry_msgs::Pose b);
bool twistEqual(geometry_msgs::Twist a, geometry_msgs::Twist b);
bool odometryEqual(nav_msgs::Odometry a, nav_msgs::Odometry b);

std::string printVector3f(Eigen::Vector3f a);
std::string printOdometry(nav_msgs::Odometry odom);
void printTrajectory(const base_local_planner::Trajectory& traj);

}
#endif