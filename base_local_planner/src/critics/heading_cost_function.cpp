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

#include <base_local_planner/critics/heading_cost_function.h>
#include <tf/transform_datatypes.h>

namespace base_local_planner {

HeadingCostFunction::HeadingCostFunction(double rejection_half_angle) :
    rejection_half_angle_(rejection_half_angle),
    path_start_heading_(0), EPSILON(0.001),
    goal_distance_squared_(0.0),
    min_goal_distance_(0.5),
    pose_capture_min_radius_(0.5), pose_capture_max_radius_(1.5),
    heading_violation_(false) {}

void HeadingCostFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses) {
  target_poses_ = target_poses;
}

bool HeadingCostFunction::prepare() {

  heading_violation_ = false;

  if (goal_distance_squared_ < (min_goal_distance_ * min_goal_distance_))
  {
    heading_violation_ = false;
    return true;
  }

  if (target_poses_.size() == 0)
  {
    heading_violation_ = false;
    return true;
  }

  // Calculate the 'best' heading to compare to.
  double min_caputure_radius_squared = pose_capture_min_radius_ * pose_capture_min_radius_;
  double max_caputure_radius_squared = pose_capture_max_radius_ * pose_capture_max_radius_;

  // Go forwards through the poses and find the first one that exits the min radius.
  geometry_msgs::PoseStamped target_pose = target_poses_[0];
  for (size_t k = 0; k < target_poses_.size(); ++k)
  {
    double dist = poseDistanceSquared(target_poses_[k], current_pose_);
    if (dist < min_caputure_radius_squared)
    {
      target_pose = target_poses_[k];
    }
    else if (dist > max_caputure_radius_squared)
    {
        break;
    }
  }

  path_start_heading_ = tf::getYaw(target_pose.pose.orientation);

  // Check the heading difference
  double current_heading = tf::getYaw(current_pose_.pose.orientation);
  double heading_diff = current_heading - path_start_heading_;
  // Normalize the heading difference
  while (heading_diff > M_PI)
  {
    heading_diff -= 2 * M_PI;
  }
  while (heading_diff < -M_PI)
  {
    heading_diff += 2 * M_PI;
  }

  ROS_DEBUG("Path heading: %f, current heading: %f", path_start_heading_, current_heading);
  if (std::fabs(heading_diff) > rejection_half_angle_)
  {
    heading_violation_ = true;
  }

  return true;
}

double HeadingCostFunction::scoreTrajectory(Trajectory &traj) {

  if (heading_violation_)
  {
    // Check to see if this is a turn in place motion.
    if (std::fabs(traj.thetav_) < EPSILON)
    {
      return -1.0;
    }
    if (std::fabs(traj.xv_) > EPSILON)
    {
      return -2.0;
    }
  }
  return 0.0;
}

} /* namespace base_local_planner */
