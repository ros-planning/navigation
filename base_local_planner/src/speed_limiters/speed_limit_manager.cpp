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

#include <base_local_planner/speed_limiters/speed_limit_manager.h>
#include <base_local_planner/speed_limiters/obstacle_speed_limiter.h>
#include <base_local_planner/speed_limiters/shadow_speed_limiter.h>
#include <base_local_planner/speed_limiters/path_speed_limiter.h>

namespace base_local_planner {

void SpeedLimitManager::initialize(costmap_2d::Costmap2DROS* costmap) {
  std::string name = "~/speed_limiters";
  // Create the limiters
  limiters_.clear();

  // Obstacle
  auto obs_limiter = std::make_shared<ObstacleSpeedLimiter>(costmap);
  obs_limiter->initialize(name);
  limiters_.push_back(obs_limiter);

  // Shadow
  auto shadow_limiter = std::make_shared<ShadowSpeedLimiter>(costmap);
  shadow_limiter->initialize(name);
  limiters_.push_back(shadow_limiter);

  // Path
  auto path_limiter = std::make_shared<PathSpeedLimiter>(costmap);
  path_limiter->initialize(name);
  limiters_.push_back(path_limiter);
};

/**
 * Calculate limits
 * @return true if preparations were successful
 */
bool SpeedLimitManager::calculateLimits(double& max_allowed_linear_vel, double& max_allowed_angular_vel) {
  max_allowed_linear_vel = max_linear_velocity_;
  max_allowed_angular_vel = max_angular_velocity_;

  for (const auto& limiter : limiters_)
  {
    double linear = 0, angular = 0;
    if (!limiter->calculateLimits(linear, angular))
    {
      // Some limiter failed.
      max_allowed_linear_vel = 0;
      max_allowed_angular_vel = 0;
      return false;
    }
    max_allowed_linear_vel = std::min(max_allowed_linear_vel, linear);
    max_allowed_angular_vel = std::min(max_allowed_angular_vel, angular);
  }
  return true;
}

void SpeedLimitManager::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
  for (const auto& limiter : limiters_)
  {
    limiter->setPlan(plan);
  }
};

void SpeedLimitManager::setMaxLimits(double linear, double angular) {
  for (const auto& limiter : limiters_)
  {
    limiter->setMaxLimits(linear, angular);
  }
  max_linear_velocity_ = linear;
  max_angular_velocity_ = angular;
};


} /* namespace base_local_planner */