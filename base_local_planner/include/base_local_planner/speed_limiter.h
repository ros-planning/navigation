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
 *   * Neither the name of 6 River Systems. nor the names of its
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

#ifndef SPEED_LIMITER_H_
#define SPEED_LIMITER_H_

#include <base_local_planner/critics/trajectory_cost_function.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/ObstructionMsg.h>
#include <tf/tf.h>

namespace base_local_planner {

struct SpeedLimiterParams {
  double max_linear_velocity_ = 1.0;
  double min_linear_velocity_ = 0.3;
  double linear_acceleration_ = 0.7;
  double half_angle_ = 1.6;
  double x_buffer_ = 0.5;
  double y_buffer_ = 0.3;

  double min_angular_velocity_effect_distance_ = 0.2;
  double max_angular_velocity_effect_distance_ = 0.5;
  double max_angular_velocity_ = 1.0;
  double min_angular_velocity_ = 0.3;
};

/**
 * This class provides cost speed and distance from dynamic obstacles
 *
 * It will reject trajectories that are over a speed curve given distance to obstacles
 */
class SpeedLimiter {
public:
  /**
   * Constructor
   */
  SpeedLimiter();

  /**
   * Destructor
   */
  ~SpeedLimiter() {}

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
  void setCurrentPose(tf::Stamped<tf::Pose> pose);
  /**
   * Prepare for operation.
   * @return true if preparations were successful
   */
  bool calculateLimits(double& max_allowed_linear_vel, double& max_allowed_angular_vel);

  void setParams(SpeedLimiterParams params) {
    params_ = params;
    calculateStopDistances();
  }

  void setWorldFrameId(std::string id)
  {
    world_frame_id_ = id;
  }

  void setBodyFrameId(std::string id)
  {
    body_frame_id_ = id;
  }


  void setFootprint(std::vector<geometry_msgs::Point> footprint)
  {
    calculateFootprintBounds(footprint);
  }

private:

  double getBearingToObstacle(costmap_2d::ObstructionMsg obs);

  double calculateAllowedLinearSpeed(costmap_2d::ObstructionMsg obs);

  double calculateAllowedAngularSpeed(costmap_2d::ObstructionMsg obs);

  costmap_2d::ObstructionMsg obstructionToBodyFrame(const costmap_2d::ObstructionMsg& in);

  void calculateStopDistances();

  void calculateFootprintBounds(std::vector<geometry_msgs::Point> footprint);

  std::vector<geometry_msgs::PoseStamped> target_poses_;
  std::shared_ptr<std::vector<costmap_2d::ObstructionMsg>>  obstructions_;

  double min_distance_to_stop_;
  double max_distance_to_stop_;

  // All of these params need to be filled out.
  SpeedLimiterParams params_;

  double footprint_min_x_;
  double footprint_max_x_;
  double footprint_min_y_;
  double footprint_max_y_;
  double circumscribed_radius_;

  tf::Transform current_pose_inv_tf_;

  std::string body_frame_id_;
  std::string world_frame_id_;
};

} /* namespace base_local_planner */
#endif /* SPEED_LIMITER_H_ */
