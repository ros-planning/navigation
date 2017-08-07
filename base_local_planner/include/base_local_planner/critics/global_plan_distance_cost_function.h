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

#ifndef GLOBAL_PLAN_DISTANCE_COST_FUNCTION_H_
#define GLOBAL_PLAN_DISTANCE_COST_FUNCTION_H_

#include <base_local_planner/critics/trajectory_cost_function.h>
#include <geometry_msgs/PoseStamped.h>

namespace base_local_planner {

/**
 * This class provides cost based on distance of the robot to the global plan
 *
 * It will reject trajectories that are moving if the global plan is too far from the robot
 */
class GlobalPlanDistanceCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  /**
   * Constructor
   * @param max_allowed_distance_from_plan Sets the max_allowed_distance_from_plan (see other comments)
   */
  GlobalPlanDistanceCostFunction(double max_allowed_distance_from_plan = 0.5);

  /**
   * Destructor
   */
  ~GlobalPlanDistanceCostFunction() {}

  /**
   * Set the target poses (global plan)
   * @param target_poses The target poses
   */
  void setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses);

  /**
   * Set the current pose of the robot
   * @param pose The current pose
   */
  void setCurrentPose(geometry_msgs::PoseStamped pose) {current_pose_ = pose;}

  /**
   * Sets the global plan distance
   * If the plan is farther than this from the robot, all trajectories with non-zero velocity are rejected.
   * @param max_allowed_distance_from_plan The max_allowed_distance_from_plan
   */
  void setMaxAllowedDistanceFromPlan(double max_allowed_distance_from_plan)
  {
    max_allowed_distance_from_plan_ = max_allowed_distance_from_plan;
  }

  /**
   * Prepare for operation.
   * @return true if preparations were successful
   */
  bool prepare();

  /**
   * Scores the trajectory.  Returns a negative value for rejected trajectories.
   * @param traj The trajectory
   * @return Non-negative value if the trajectory is valid, negative otherwise.
   */
  double scoreTrajectory(Trajectory &traj);

private:
  /**
   * Calculates the square of the distance between two poses.
   * @param p1 First pose
   * @param p2 Second pose
   * @return the squared distance between the poses.
   */
  double poseDistanceSquared(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2)
  {
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    return (dx*dx + dy*dy);
  }

  std::vector<geometry_msgs::PoseStamped> target_poses_;

  geometry_msgs::PoseStamped current_pose_;

  bool distance_violation_;

  double max_allowed_distance_from_plan_;

  constexpr static double EPSILON = 0.001;
};

} /* namespace base_local_planner */
#endif /* HEADING_COST_FUNCTION_H_ */
