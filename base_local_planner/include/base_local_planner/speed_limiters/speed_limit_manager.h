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

#ifndef SPEED_LIMIT_MANAGER_H_
#define SPEED_LIMIT_MANAGER_H_

#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/speed_limiters/speed_limiter.h>
#include <base_local_planner/SpeedLimitManagerConfig.h>

namespace base_local_planner {

/**
 * This class limits the velocity of the robot
 */
class SpeedLimitManager {
public:
  /**
   * Constructor
   */
  SpeedLimitManager() {};

  /**
   * Destructor
   */
  ~SpeedLimitManager() {}

  void intialize(costmap_2d::Costmap2D* costmap);

  /**
   * Calculate limits
   * @return true if preparations were successful
   */
  bool calculateLimits(double& max_allowed_linear_vel, double& max_allowed_angular_vel);
  
  void setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

  void setMaxLimits(double linear, double angular);

protected:
  void reconfigure(PathSpeedLimiterConfig &cfg, uint32_t level) {
    setMaxLimits(cfg.max_linear_velocity, cfg.max_angular_velocity);
  }
  std::vector<std::shared_ptr<SpeedLimiter>> limiters_;
  
  std::shared_ptr<dynamic_reconfigure::Server<SpeedLimitManagerConfig>> configServer_;
  SpeedLimitManagerConfig params_;

  double max_linear_velocity = 1.0;
  double max_angular_velocity = 1.0;
};

} /* namespace base_local_planner */
#endif /* SPEED_LIMIT_MANAGER_H_ */
