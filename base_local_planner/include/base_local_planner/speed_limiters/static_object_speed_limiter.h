/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, 6 River Systems
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
 * Author: Thomas Preisner
 *********************************************************************/

#ifndef STATIC_OBJECT_SPEED_LIMITER_H_
#define STATIC_OBJECT_SPEED_LIMITER_H_

#include <base_local_planner/speed_limiters/speed_limiter.h>
#include <base_local_planner/StaticObjectSpeedLimiterConfig.h>
#include <base_local_planner/SpeedLimitRequest.h>
#include <costmap_2d/ObstructionMsg.h>
#include <geometry_msgs/Twist.h>
#include <srslib_framework/MsgChassisConfig.h>
#include <srslib_framework/chuck/ChuckChassisGenerations.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <srslib_framework/MsgCERealTimeData.h>

namespace base_local_planner {

class StaticObjectSpeedLimiter;
/**
 * This class provides a test harness for the StaticObjectSpeedLimiter class
 */
class StaticObjectSpeedLimiter_TEST {
public:
  struct SpeedLimiterTestData {
    bool enabledfirmwareVersion = true;
    srs::Velocity<> velocity;
    double distance_from_static_left = 0.0;
    double distance_from_static_right = 0.0;
    srs::ChuckChassisGenerations::ChuckChassisType chassis_generation_ =
        srs::ChuckChassisGenerations::ChuckChassisType::INVALID;
  };

  explicit StaticObjectSpeedLimiter_TEST(StaticObjectSpeedLimiter* limiter) : speed_limiter_(limiter) {}

  void getLimits_Test(double& max_allowed_linear_vel, double& max_allowed_angular_vel);

  bool calculateLimits_Test(const SpeedLimiterTestData* data, double& max_allowed_linear_vel,
                            double& max_allowed_angular_vel);

private:
  StaticObjectSpeedLimiter_TEST() {}  // not accessable
  StaticObjectSpeedLimiter* speed_limiter_ = nullptr;
};

/**
 * This class provides cost speed and distance from static obstacles
 *
 * It will limit the speed near static obstacle to limit false safety trips
 */
class StaticObjectSpeedLimiter : public SpeedLimiter {
public:
  /**
   * Constructor
   */
  StaticObjectSpeedLimiter(costmap_2d::Costmap2DROS* costmap) : SpeedLimiter(costmap){};

  /**
   * Destructor
   */
  virtual ~StaticObjectSpeedLimiter() {
    delete (nh_);
    nh_ = nullptr;
  }

  virtual void initialize(std::string name);

  /**
   * Prepare for operation.
   * @return true if preparations were successful
   */
  virtual bool calculateLimits(double& max_allowed_linear_vel, double& max_allowed_angular_vel);

  std::string getName();

  void reconfigure(StaticObjectSpeedLimiterConfig cfg) { params_ = cfg; };

private:
  friend StaticObjectSpeedLimiter_TEST;

  void msgCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void emulationModeCallback(const std_msgs::Bool& emulationMode);
  void chassisConfigCallback(const srslib_framework::MsgChassisConfig& config);
  void ceSensorArrayCallback(const srslib_framework::MsgCERealTimeData& msg);

  struct SpeedLimiterResult {
    double speed = 0.0;
    bool limiting = false;
  };

  struct SpeedLimiterData {
    double distLeft = 0.0;
    double distRight = 0.0;
    double speed = 0.0;
    double minVelocity = 0.0;
    double minTestDistance = 0.0;
    double maxTestDistance = 0.0;
    double minTestVelocity = 0.0;
    double maxTestVelocity = 0.0;
    double minReduction = 0.0;
    double maxReduction = 0.0;
  };

  SpeedLimiterResult calculateAllowedLinearSpeed(const double distLeft, const double distRight,
                                                 const double speed) const;
  SpeedLimiterResult calculateAllowedAngularSpeed(const double distLeft, const double distRight,
                                                  const double speed) const;

  SpeedLimiterResult calculateAllowedSpeed(const SpeedLimiterData& data) const;

  ros::NodeHandle* nh_ = nullptr;
  ros::Subscriber subscriber_;
  ros::Subscriber emulation_mode_sub_;
  ros::Subscriber chassis_generation_sub_;
  ros::Subscriber hardware_version_sub_;
  ros::Publisher staticObject_pub;

  std::shared_ptr<dynamic_reconfigure::Client<StaticObjectSpeedLimiterConfig>> configClient_;
  StaticObjectSpeedLimiterConfig params_;

  bool enabledFirmwareVersion_ = false;
  bool emulationMode_ = false;
  ros::Time last_time_ = ros::Time(0);
  double cachedMaxLinearVelocity_ = -1.0;
  double cachedMaxAngularVelocity_ = -1.0;

  srs::Velocity<> velocity_;
  srs::ChuckChassisGenerations::ChuckChassisType chassis_generation_ =
      srs::ChuckChassisGenerations::ChuckChassisType::INVALID;

  const StaticObjectSpeedLimiter_TEST::SpeedLimiterTestData* testData_ = nullptr;
};

} /* namespace base_local_planner */
#endif /* STATIC_OBJECT_SPEED_LIMITER_H_ */
