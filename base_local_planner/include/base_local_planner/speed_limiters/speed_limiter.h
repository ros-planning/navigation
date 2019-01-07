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

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace base_local_planner {

/**
 * This class limits the velocity of the robot
 */
class SpeedLimiter {
public:
  /**
   * Constructor
   */
  SpeedLimiter(costmap_2d::Costmap2DROS* costmap) : costmap_(costmap) {};

  /**
   * Destructor
   */
  virtual ~SpeedLimiter() {
    // NOTE: costmap_ is not deleted here.  Responsibility for deletion is in move_base
  }

  virtual void initialize(std::string name) = 0;

  /**
   * Calculate limits
   * @return true if preparations were successful
   */
  virtual bool calculateLimits(double& max_allowed_linear_vel, double& max_allowed_angular_vel) = 0;

  virtual std::string getName() = 0;
  
  void setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    plan_ = plan;
  };

  void setMaxLimits(double linear, double angular) {
    max_linear_velocity_ = linear;
    max_angular_velocity_ = angular;
  };

protected:
  bool getCurrentPose(tf::Stamped<tf::Pose>& pose) {
    if (!costmap_->getRobotPose(pose)) {
      ROS_WARN("Could not get robot pose to calculate speed limits");
      return false;
    }  
    return true;  
  }

  costmap_2d::Costmap2DROS* costmap_;

  std::vector<geometry_msgs::PoseStamped> plan_;

  double max_linear_velocity_ = 1.0;
  double max_angular_velocity_ = 1.0;
};

} /* namespace base_local_planner */
#endif /* SPEED_LIMITER_H_ */
