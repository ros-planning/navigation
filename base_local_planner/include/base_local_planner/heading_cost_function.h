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

#ifndef HEADING_COST_FUNCTION_H_
#define HEADING_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <geometry_msgs/PoseStamped.h>

namespace base_local_planner {

/**
 * This class provides cost based on the heading of the robot relative to the trajectory.
 *
 * This can be used to favor trajectories which stay on a given path, or which
 * approach a given goal.
 */
class HeadingCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  HeadingCostFunction(double rejection_half_angle=M_PI / 2);

  ~HeadingCostFunction() {}

  void setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses);

  void setCurrentPose(geometry_msgs::PoseStamped pose) {current_pose_ = pose;}

  void setRejectionHalfAngle(double rejection_half_angle) {rejection_half_angle_ = rejection_half_angle;}

  void setGoalDistanceSquared(double goal_distance_squared) {goal_distance_squared_ = goal_distance_squared;}

  bool prepare();

  double scoreTrajectory(Trajectory &traj);

private:
  double poseDistanceSquared(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2)
  {
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    return (dx*dx + dy*dy);
  }

  std::vector<geometry_msgs::PoseStamped> target_poses_;

  geometry_msgs::PoseStamped current_pose_;

  bool heading_violation_;

  double pose_capture_min_radius_;
  double pose_capture_max_radius_;

  double rejection_half_angle_;
  double path_start_heading_;
  double goal_distance_squared_;
  double min_goal_distance_;
  double EPSILON;
};

} /* namespace base_local_planner */
#endif /* HEADING_COST_FUNCTION_H_ */
