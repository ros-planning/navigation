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

#ifndef SPEED_COST_FUNCTION_H_
#define SPEED_COST_FUNCTION_H_

#include <base_local_planner/critics/trajectory_cost_function.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/ObstructionMsg.h>
#include <tf/tf.h>

namespace base_local_planner {

/**
 * This class provides cost speed and distance from dynamic obstacles
 *
 * It will reject trajectories that are over a speed curve given distance to obstacles
 */
class SpeedCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  /**
   * Constructor
   */
  SpeedCostFunction();

  /**
   * Destructor
   */
  ~SpeedCostFunction() {}

  /**
   * Set the obstructions
   * @param obstructions
   */
  void setObstructions(std::shared_ptr<std::vector<costmap_2d::ObstructionMsg>> obstructions) {
    obstructions_ = obstructions;
  }

  /**
   * Set the current pose of the robot
   * @param pose The current pose
   */
  void setCurrentPose(geometry_msgs::PoseStamped pose);
  /**
   * Prepare for operation.
   * @return true if preparations were successful
   */
  bool prepare();

  /**
   * Scores the trajectory.  Returns a negative value for rejected trajectories.
   * @param traj The trajectory
   * @return Non-negative value if the trajectory is valid, negative otherwise.
   *     -1 indicates that it is in violation of the distance to obstruction metric
   */
  double scoreTrajectory(Trajectory &traj);

  void setMaxLinearVelocity(double vel) {
    max_linear_velocity_ = vel;
    calculateStopDistances();
  }

  void setMinLinearVelocity(double vel) {
    min_linear_velocity_ = vel;
    calculateStopDistances();
  }

  void setAcceleration(double accel)
  {
    linear_acceleration_ = accel;
    calculateStopDistances();
  }

  void setWorldFrameId(std::string id)
  {
    world_frame_id_ = id;
  }

  void setFootprint(std::vector<geometry_msgs::Point> footprint)
  {
    calculateFootprintBounds(footprint);
  }

  void setYBuffer(double buffer)
  {
    y_buffer_ = buffer;
  }

  void setXBuffer(double buffer)
  {
    x_buffer_ = buffer;
  }

  void setHalfAngle(double val)
  {
    half_angle_ = val;
  }

private:

  double getBearingToObstacle(costmap_2d::ObstructionMsg obs);

  double calculateAllowedSpeed(costmap_2d::ObstructionMsg obs);

  costmap_2d::ObstructionMsg obstructionToBodyFrame(const costmap_2d::ObstructionMsg& in);

  void calculateStopDistances();

  void calculateFootprintBounds(std::vector<geometry_msgs::Point> footprint);


  std::vector<geometry_msgs::PoseStamped> target_poses_;
  std::shared_ptr<std::vector<costmap_2d::ObstructionMsg>>  obstructions_;

  double max_allowed_vel_;
  double min_distance_to_stop_;
  double max_distance_to_stop_;

  // All of these params need to be filled out.
  double max_linear_velocity_;
  double min_linear_velocity_;
  double linear_acceleration_;
  double half_angle_;
  double x_buffer_;
  double y_buffer_;

  double footprint_min_x_;
  double footprint_max_x_;
  double footprint_min_y_;
  double footprint_max_y_;

  tf::Transform current_pose_inv_tf_;

  std::string body_frame_id_;
  std::string world_frame_id_;
};

} /* namespace base_local_planner */
#endif /* SPEED_COST_FUNCTION_H_ */
