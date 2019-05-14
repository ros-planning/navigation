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
#include <base_local_planner/speed_limiters/external_speed_limiter.h>
#include <std_msgs/String.h>
#include <base_local_planner/speed_limiters.h>
#include <base_local_planner/speed_limiter.h>

namespace base_local_planner {

void SpeedLimitManager::initialize(costmap_2d::Costmap2DROS* costmap, std::string controller_name) {
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

  // External
  auto external_limiter = std::make_shared<ExternalSpeedLimiter>(costmap);
  external_limiter->initialize(name);
  limiters_.push_back(external_limiter);

  // Create the dynamic reconfigure
  ros::NodeHandle private_nh(name);
  configClient_ = std::make_shared<dynamic_reconfigure::Client<SpeedLimitManagerConfig>>(name);
  configClient_->setConfigurationCallback(boost::bind(&SpeedLimitManager::reconfigure, this, _1));
  
  ROS_INFO_STREAM("WHAT");
  limiter_pub = private_nh.advertise<base_local_planner::speed_limiter>("limiter_greatest", 10);
  limiters_pub = private_nh.advertise<base_local_planner::speed_limiters>("limiter_values", 10);
};

/**
 * Calculate limits
 * @return true if preparations were successful
 */
bool SpeedLimitManager::calculateLimits(double& max_allowed_linear_vel, double& max_allowed_angular_vel) {
  max_allowed_linear_vel = max_linear_velocity_;
  max_allowed_angular_vel = max_angular_velocity_;
  std::string limiter_string = "Nothing";
  base_local_planner::speed_limiter greatest;
  base_local_planner::speed_limiters limiterArray;
  greatest.name = limiter_string;
  for (const auto& limiter : limiters_)
  {
    double linear = 0, angular = 0;
    if (!limiter->calculateLimits(linear, angular))
    {
      // Some limiter failed.
      max_allowed_linear_vel = 0;
      max_allowed_angular_vel = 0;
      ROS_WARN_STREAM("The " << limiter->getName() << " limiter failed.");
      return false;
    }
    if(linear < max_allowed_linear_vel){
      greatest.name = limiter->getName();
      greatest.linear_value = linear;
      greatest.angular_value = angular;
    }
    base_local_planner::speed_limiter temp;
    temp.name = limiter->getName();
    temp.linear_value = linear;
    temp.angular_value = angular;
    if(temp.name == "Shadow"){
      limiterArray.shadow = temp;
    }
    else if(temp.name == "Obstacle"){
      limiterArray.obstacle = temp;
    }
    else if(temp.name == "Path"){
      limiterArray.path = temp;
    }
    else if(temp.name == "External"){
      limiterArray.external = temp;
    }
    max_allowed_linear_vel = std::min(max_allowed_linear_vel, linear);
    max_allowed_angular_vel = std::min(max_allowed_angular_vel, angular);
  }
  limiter_pub.publish(greatest);
  limiters_pub.publish(limiterArray);
  ROS_DEBUG_THROTTLE(0.2, "Limits: %f, %f", max_allowed_linear_vel, max_allowed_angular_vel);
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
  ROS_INFO_STREAM(linear << " " << angular);
};


} /* namespace base_local_planner */
