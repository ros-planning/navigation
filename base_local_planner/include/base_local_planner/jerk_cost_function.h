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

#ifndef JERK_COST_FUNCTION_H_
#define JERK_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>


namespace base_local_planner {

/**
 * This class provides cost based on the jerk of the robot
 *
 */
class JerkCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  /**
   * Constructor
   */
  JerkCostFunction();

  /**
   * Destructor
   */
  ~JerkCostFunction() {}

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

  void setPreviousTrajectoryAndVelocity(Trajectory* traj, Eigen::Vector3f vel);

  void setCurrentVelocity(Eigen::Vector3f vel)
  {
    current_vel_ = vel;
  };

private:
  void calculateAccelerations(Trajectory* traj, Eigen::Vector3f vel,
    double* linear_accel, double* angular_accel, std::string msg);

  double EPSILON;
  double old_linear_accel_;
  double old_angular_accel_;
  Eigen::Vector3f current_vel_;
};

} /* namespace base_local_planner */
#endif /* HEADING_COST_FUNCTION_H_ */
