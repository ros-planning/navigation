/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017 6 River Systems
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
 * Author: dgrieneisen
 *********************************************************************/

#ifndef BASE_LOCAL_PLANNER_GEOMETRY_MATH_HELPERS_H_
#define BASE_LOCAL_PLANNER_GEOMETRY_MATH_HELPERS_H_

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <Eigen/Core>

namespace base_local_planner {

double distanceToLineSegment(const Eigen::Vector2f& pos,
  const Eigen::Vector2f& p0, const Eigen::Vector2f& p1);

double distanceAlongLineSegment(const Eigen::Vector2f& pos,
  const Eigen::Vector2f& p0, const Eigen::Vector2f& p1);

Eigen::Vector2f poseAtDistanceAlongLineSegment(double distance,
  const Eigen::Vector2f& p0, const Eigen::Vector2f& p1);

Eigen::Vector2f poseStampedToVector(geometry_msgs::PoseStamped pose);

double angleMinusPiToPi(double val);

Eigen::Vector3f poseToVector3f(const geometry_msgs::Pose& pose);
Eigen::Vector3f twistToVector3f(const geometry_msgs::Twist& t);
geometry_msgs::Pose vector3fToPose(const Eigen::Vector3f& vec);
geometry_msgs::Twist vector3fToTwist(const Eigen::Vector3f& vec);

double linearInterpolation(double value, double min_value, double max_value, double min_output, double max_output);

double twoLevelInterpolation(double value, 
  double min_value, double max_value, 
  double min_output, double max_output);

double threeLevelInterpolation(double value, 
  double min_value, double nominal_value_low, 
  double nominal_value_high, double max_value,
  double min_output, double nominal_output, double max_output);

}

#endif