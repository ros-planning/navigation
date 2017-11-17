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

#ifndef SHADOW_SPEED_LIMITER_H_
#define SHADOW_SPEED_LIMITER_H_

#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/map_grid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

namespace base_local_planner {

struct ShadowSpeedLimiterParams {
  double max_linear_velocity_ = 1.0;
  double min_linear_velocity_ = 0.3;

  double max_effective_range_ = 1.5;
  double min_effective_range_ = 0.5;

  double max_angular_velocity_ = 1.0;
};

/**
 * This class provides cost speed and distance from shadow obstacles
 *
 * It will reject trajectories that are over a speed curve given distance to obstacles
 */
class ShadowSpeedLimiter {
public:
  /**
   * Constructor
   */
  ShadowSpeedLimiter();

  /**
   * Destructor
   */
  ~ShadowSpeedLimiter() {}
  void initialize(costmap_2d::Costmap2D* costmap);

  /**
   * Set the obstructions
   * @param obstructions
   */
  void setShadowedObjects(std::shared_ptr<std::vector<geometry_msgs::Point>> obs) {
    objects_ = obs;
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

  void setParams(ShadowSpeedLimiterParams params) {
    params_ = params;
  }

private:
  double getMapGridDistance(geometry_msgs::Point obj);
  double distanceToVelocity(double dist);


  costmap_2d::Costmap2D* costmap_;
  base_local_planner::MapGrid map_grid_;
  
  std::vector<geometry_msgs::PoseStamped> current_pose_as_array_;
  std::shared_ptr<std::vector<geometry_msgs::Point>>  objects_;

  bool initialized_ = false;

  // All of these params need to be filled out.
  ShadowSpeedLimiterParams params_;

};

} /* namespace base_local_planner */
#endif /* SHADOW_SPEED_LIMITER_H_ */
