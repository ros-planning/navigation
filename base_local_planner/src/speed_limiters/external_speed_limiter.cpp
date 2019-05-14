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

#include <base_local_planner/speed_limiters/external_speed_limiter.h>
#include <base_local_planner/geometry_math_helpers.h>
#include <tf/transform_datatypes.h>

namespace base_local_planner {

void ExternalSpeedLimiter::initialize(std::string name) {
  ros::NodeHandle private_nh(name + "/external");

  configClient_ = std::make_shared<dynamic_reconfigure::Client<ExternalSpeedLimiterConfig>>(name + "/external");
  configClient_->setConfigurationCallback(boost::bind(&ExternalSpeedLimiter::reconfigure, this, _1));

  // Create the subscriber
  subscriber_ = private_nh.subscribe("limit", 10, &ExternalSpeedLimiter::msgCallback, this);
}

std::string ExternalSpeedLimiter::getName(){
  return std::string("External");
}

bool ExternalSpeedLimiter::calculateLimits(double& max_allowed_linear_vel, double& max_allowed_angular_vel) {
  // Reset the maximum allowed velocity
  max_allowed_linear_vel = max_linear_velocity_;
  max_allowed_angular_vel = max_angular_velocity_;

  if (params_.timeout > 0 && (ros::Time::now() - last_msg_time_ > ros::Duration(params_.timeout))) {
    ROS_DEBUG_THROTTLE(0.5, "External speed limiter timeout.");
    return true;
  }

  if (last_msg_.max_linear_velocity >= 0) {
    max_allowed_linear_vel = std::min(last_msg_.max_linear_velocity, (float)max_linear_velocity_);  
  }
  
  if (last_msg_.max_angular_velocity >= 0) {
    max_allowed_angular_vel = std::min(last_msg_.max_angular_velocity, (float)max_angular_velocity_);
  }

  ROS_DEBUG_THROTTLE(0.2, "Setting external max speed to %f, %f", max_allowed_linear_vel, max_allowed_angular_vel);

  return true;
}

} /* namespace base_local_planner */
