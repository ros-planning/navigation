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

namespace base_local_planner
{
  /**
 * This class provides cost speed and distance from static obstacles
 *
 * It will limit the speed near static obstacle to limit false safety trips
 */
  class StaticObjectSpeedLimiter : public SpeedLimiter
  {
  public:
    /**
   * Constructor
   */
    StaticObjectSpeedLimiter(costmap_2d::Costmap2DROS *costmap) : SpeedLimiter(costmap){};

    virtual void initialize(std::string name);

    /**
   * Prepare for operation.
   * @return true if preparations were successful
   */
    virtual bool calculateLimits(double &max_allowed_linear_vel, double &max_allowed_angular_vel);

    std::string getName();

    void reconfigure(StaticObjectSpeedLimiterConfig cfg) { params_ = cfg; };

  protected:
    std::shared_ptr<dynamic_reconfigure::Client<StaticObjectSpeedLimiterConfig>> configClient_;
    StaticObjectSpeedLimiterConfig params_;

    struct SpeedLimiterResult
    {
      double speed = 0.0;
      bool limiting = false;
    };

    struct SpeedLimiterData
    {
      double dist_left_ = 0.0;
      double dist_right_ = 0.0;
      double speed_ = 0.0;
      double min_velocity_ = 0.0;
      double min_test_distance_ = 0.0;
      double max_test_distance_ = 0.0;
      double min_test_velocity_ = 0.0;
      double max_test_velocity_ = 0.0;
      double min_reduction_ = 0.0;
      double max_reduction_ = 0.0;
    };

    SpeedLimiterResult calculateAllowedLinearSpeed(const double dist_left, const double dist_right,
                                                   const double speed) const;
    SpeedLimiterResult calculateAllowedAngularSpeed(const double dist_left, const double dist_right,
                                                    const double speed) const;

    SpeedLimiterResult calculateAllowedSpeed(const SpeedLimiterData &data) const;
    double calculateResultSpeed(const SpeedLimiterData &data, double test_dist, double test_size) const;

  private:
    void msgCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void emulationModeCallback(const std_msgs::Bool &emulationMode);
    void chassisConfigCallback(const srslib_framework::MsgChassisConfig &config);
    void ceSensorArrayCallback(const srslib_framework::MsgCERealTimeData &msg);
    
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Subscriber subscriber_;
    ros::Subscriber emulation_mode_sub_;
    ros::Subscriber chassis_generation_sub_;
    ros::Subscriber hardware_version_sub_;
    ros::Publisher staticObject_pub;

    bool enabledFirmwareVersion_ = false;
    bool forceEnabled_ = false;   //< used for testing purposes
    ros::Time last_time_ = ros::Time(0);
    double cachedMaxLinearVelocity_ = -1.0;
    double cachedMaxAngularVelocity_ = -1.0;

    srs::Velocity<> velocity_;
    srs::ChuckChassisGenerations::ChuckChassisType chassis_generation_ =
        srs::ChuckChassisGenerations::ChuckChassisType::INVALID;
  };

} /* namespace base_local_planner */
#endif /* STATIC_OBJECT_SPEED_LIMITER_H_ */
