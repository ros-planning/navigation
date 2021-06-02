/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, 6 River Systems
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
 * Author: Thomas Preisner
 *********************************************************************/

#include <base_local_planner/speed_limiters/static_object_speed_limiter.h>
#include <base_local_planner/speed_limiters/safety_ce_bounds_data.h>
#include <base_local_planner/geometry_math_helpers.h>
#include <base_local_planner/StaticObjects.h>
#include <srslib_framework/chuck/ChuckChassisGenerations.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>
#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/ros/message/VelocityMessageFactory.hpp>
#include <tf/transform_datatypes.h>
#include <costmap_2d/footprint.h>

namespace base_local_planner
{

  void StaticObjectSpeedLimiter::initialize(std::string name)
  {
    configClient_ = std::make_shared<dynamic_reconfigure::Client<StaticObjectSpeedLimiterConfig>>(name + "/staticObject");
    configClient_->setConfigurationCallback(boost::bind(&StaticObjectSpeedLimiter::reconfigure, this, _1));

    nh_ = std::make_shared<ros::NodeHandle>();

    subscriber_ =
        nh_->subscribe<geometry_msgs::Twist>(srs::ChuckTopics::driver::ODOMETRY_CMD_VELOCITY_UNFILTERED, 10, boost::bind(&StaticObjectSpeedLimiter::msgCallback, this, _1));

    chassis_generation_sub_ =
        nh_->subscribe(srs::ChuckTopics::driver::BRAINSTEM_CHASSIS_CONFIG, 10, &StaticObjectSpeedLimiter::chassisConfigCallback, this);

    hardware_version_sub_ = nh_->subscribe(srs::ChuckTopics::driver::BRAINSTEM_CE_REAL_TIME_DATA, 10,
                                           &StaticObjectSpeedLimiter::chassisConfigCallback, this);

    ros::NodeHandle private_nh(name + "/staticObject");
    staticObject_pub = private_nh.advertise<base_local_planner::StaticObjects>("staticObject_info", 5, true);
  }

  std::string StaticObjectSpeedLimiter::getName() { return std::string("StaticObject"); }

  void StaticObjectSpeedLimiter::chassisConfigCallback(const srslib_framework::MsgChassisConfig &config)
  {
    chassis_generation_ = static_cast<srs::ChuckChassisGenerations::ChuckChassisType>(config.chassisFootprintType);
  }

  void StaticObjectSpeedLimiter::msgCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    velocity_ = srs::VelocityMessageFactory::msg2Velocity(msg);
  }

  void StaticObjectSpeedLimiter::ceSensorArrayCallback(const srslib_framework::MsgCERealTimeData &msg)
  {
    if (msg.firmwareId == 6)
    {
      enabledFirmwareVersion_ = true;
    }
  }

  bool StaticObjectSpeedLimiter::calculateLimits(double &max_allowed_linear_vel, double &max_allowed_angular_vel)
  {
    // Reset the maximum allowed velocity
    max_allowed_linear_vel = max_linear_velocity_;
    max_allowed_angular_vel = max_angular_velocity_;
    if (forceEnabled_ == false && (params_.enabled == false || enabledFirmwareVersion_ == false))
    {
      return true;
    }

    if (const char *env_var = std::getenv("ROSEVN_STATIC_OBJECT_LIMITER_ENABLED"))
    {
      std::string val(env_var);

      std::transform(val.begin(), val.end(), val.begin(), ::tolower);
      if (forceEnabled_ == false && val == "false")
      {
        return true;
      }
    }

    if (params_.timeout > 0 && (ros::Time::now() - last_time_ < ros::Duration(params_.timeout)))
    {
      if (cachedMaxLinearVelocity_ > 0.0)
      {
        max_allowed_linear_vel = cachedMaxLinearVelocity_;
      }
      if (cachedMaxAngularVelocity_ > 0.0)
      {
        max_allowed_angular_vel = cachedMaxAngularVelocity_;
      }
      return true;
    }
    last_time_ = ros::Time::now();
    cachedMaxLinearVelocity_ = max_linear_velocity_;
    cachedMaxAngularVelocity_ = max_angular_velocity_;

    // Cache off the current velocity so it can't be changed during processing
    float currLinearVel = velocity_.linear;
    float currAngularVel = fabs(velocity_.angular);
    bool flipAngular = velocity_.angular < 0.0;

    /*
Get the CE stopping path edges to get the distance from the static object map. This would then test against the
configured distances. Doing a non-linear mapping versus distance with more slowing the closer the static object is. This
will be interesting as the slower the speed the tighter the region Smoothing speed will need to be done. Otherwise the
Chuck will cycle between speeding up and slowing down.
*/

    tf::Stamped<tf::Pose> current_pose;

    if (!getCurrentPose(current_pose))
    {
      ROS_WARN_THROTTLE(1.0, "No pose in static object speed limiter");
      return false;
    }

    int linearIndex, angularIndex = 0;
    for (int i = 0; i < NUM_PLANE_LINEAR_SPEEDS; ++i)
    {
      if (LINEAR_SPEED_TO_INDEX_ARRAY[i] > currLinearVel)
      {
        break;
      }
      linearIndex = i;
    }

    for (int i = 0; i < NUM_PLANE_ANGULAR_SPEEDS; ++i)
    {
      if (ANGULAR_SPEED_TO_INDEX_ARRAY[i] > currAngularVel)
      {
        break;
      }
      angularIndex = i;
    }

    geometry_msgs::Point left, right;
    srs::ChuckChassisGenerations::ChuckChassisType chassis = chassis_generation_;

    // The plane tables indicate left right pairs of x,y coords
    if (chassis == srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5_PLUS)
    {
      left.x = LOOKUP_TABLE_CHUCK_PLUS_FRONT_PLANE[linearIndex][angularIndex][0][0];
      left.y = LOOKUP_TABLE_CHUCK_PLUS_FRONT_PLANE[linearIndex][angularIndex][0][1];
      right.x = LOOKUP_TABLE_CHUCK_PLUS_FRONT_PLANE[linearIndex][angularIndex][1][0];
      right.y = LOOKUP_TABLE_CHUCK_PLUS_FRONT_PLANE[linearIndex][angularIndex][1][1];
    }
    else if (chassis == srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5)
    {
      left.x = LOOKUP_TABLE_CHUCK_FRONT_PLANE[linearIndex][angularIndex][0][0];
      left.y = LOOKUP_TABLE_CHUCK_FRONT_PLANE[linearIndex][angularIndex][0][1];
      right.x = LOOKUP_TABLE_CHUCK_FRONT_PLANE[linearIndex][angularIndex][1][0];
      right.y = LOOKUP_TABLE_CHUCK_FRONT_PLANE[linearIndex][angularIndex][1][1];
    }
    else if (forceEnabled_)
    {
      // Pretend that it is chuck 5 for emulation purposes
      left.x = LOOKUP_TABLE_CHUCK_FRONT_PLANE[linearIndex][angularIndex][0][0];
      left.y = LOOKUP_TABLE_CHUCK_FRONT_PLANE[linearIndex][angularIndex][0][1];
      right.x = LOOKUP_TABLE_CHUCK_FRONT_PLANE[linearIndex][angularIndex][1][0];
      right.y = LOOKUP_TABLE_CHUCK_FRONT_PLANE[linearIndex][angularIndex][1][1];
    }
    else
    {
      return true; //<  Not valid for this version, so don't run but let the other limiters run
    }

    if (flipAngular)
    {
      left.x *= -1.0;
      right.x *= -1.0;
      geometry_msgs::Point temp = left;
      left = right;
      right = temp;
    }

    double distance_from_static_left = 0.0;
    double distance_from_static_right = 0.0;

    const double resolution = costmap_->getCostmap()->getResolution();
    // Transform the points based on the chucks position and orientation
    geometry_msgs::PoseStamped pose;
    tf::poseStampedTFToMsg(current_pose, pose);
    double pose_yaw = tf::getYaw(pose.pose.orientation) + srs::AngleMath::deg2Rad<double>(-90.0);
    double cos_yaw = cos(pose_yaw);
    double sin_yaw = sin(pose_yaw);

    double leftX = left.x;
    double leftY = left.y;
    double rightX = right.x;
    double rightY = right.y;

    left.x = pose.pose.position.x + ((cos_yaw * leftX) - (sin_yaw * leftY));
    left.y = pose.pose.position.y + ((cos_yaw * leftY) + (sin_yaw * leftX));
    right.x = pose.pose.position.x + ((cos_yaw * rightX) - (sin_yaw * rightY));
    right.y = pose.pose.position.y + ((cos_yaw * rightY) + (sin_yaw * rightX));

    double px = left.x;
    double py = left.y;
    distance_from_static_left = costmap_->getLayeredCostmap()->getDistanceFromStaticMap(px, py);

    px = right.x;
    py = right.y;
    distance_from_static_right = costmap_->getLayeredCostmap()->getDistanceFromStaticMap(px, py);

    distance_from_static_left *= resolution;
    distance_from_static_right *= resolution;


    if (currLinearVel >= params_.min_linear_velocity_test_speed &&
        currLinearVel <= params_.max_linear_velocity_test_speed)
    {
      SpeedLimiterResult result =
          calculateAllowedLinearSpeed(distance_from_static_left, distance_from_static_right, max_allowed_linear_vel);
      if (result.limiting)
      {
        if (result.speed < currLinearVel)
        {
          cachedMaxLinearVelocity_ = result.speed;
          max_allowed_linear_vel = result.speed;
        }
      }
    }

    if (currAngularVel >= params_.min_angular_velocity_test_speed &&
        currAngularVel <= params_.max_angular_velocity_test_speed)
    {
      SpeedLimiterResult result =
          calculateAllowedAngularSpeed(distance_from_static_left, distance_from_static_right, max_allowed_angular_vel);
      if (result.limiting)
      {
        if (result.speed < currAngularVel)
        {
          cachedMaxAngularVelocity_ = result.speed;
          max_allowed_angular_vel = result.speed;
        }
      }
    }

    ROS_DEBUG_THROTTLE(0.2, "Setting max speed to %f, %f", max_allowed_linear_vel, max_allowed_angular_vel);
    base_local_planner::StaticObjects staticObject_msg;
    staticObject_msg.distance_from_static_left = distance_from_static_left;
    staticObject_msg.distance_from_static_right = distance_from_static_right;
    staticObject_pub.publish(staticObject_msg);

    return true;
  }

  StaticObjectSpeedLimiter::SpeedLimiterResult StaticObjectSpeedLimiter::calculateAllowedLinearSpeed(
      const double dist_left, const double dist_right, const double speed) const
  {
    SpeedLimiterData data;
    data.dist_left_ = dist_left;
    data.dist_right_ = dist_right;
    data.speed_ = speed;
    data.min_velocity_ = params_.min_linear_velocity;
    data.min_test_distance_ = params_.min_linear_velocity_distance;
    data.max_test_distance_ = params_.max_linear_velocity_distance;
    data.min_test_velocity_ = params_.min_linear_velocity_test_speed;
    data.max_test_velocity_ = params_.max_linear_velocity_test_speed;
    data.min_reduction_ = params_.min_linear_velocity_reduction;
    data.max_reduction_ = params_.max_linear_velocity_reduction;

    return calculateAllowedSpeed(data);
  }

  StaticObjectSpeedLimiter::SpeedLimiterResult StaticObjectSpeedLimiter::calculateAllowedAngularSpeed(
      const double dist_left, const double dist_right, const double speed) const
  {
    SpeedLimiterData data;
    data.dist_left_ = dist_left;
    data.dist_right_ = dist_right;
    data.speed_ = speed;
    data.min_velocity_ = params_.min_angular_velocity;
    data.min_test_distance_ = params_.min_angular_velocity_distance;
    data.max_test_distance_ = params_.max_angular_velocity_distance;
    data.min_test_velocity_ = params_.min_angular_velocity_test_speed;
    data.max_test_velocity_ = params_.max_angular_velocity_test_speed;
    data.min_reduction_ = params_.min_angular_velocity_reduction;
    data.max_reduction_ = params_.max_angular_velocity_reduction;

    return calculateAllowedSpeed(data);
  }

  StaticObjectSpeedLimiter::SpeedLimiterResult
  StaticObjectSpeedLimiter::calculateAllowedSpeed(const SpeedLimiterData &data) const
  {
    SpeedLimiterResult result;

    if (data.speed_ < data.min_test_velocity_)
    {
      return result;
    }

    double size_delta = (data.speed_ - data.min_test_velocity_) / (data.max_test_velocity_ - data.min_test_velocity_);
    size_delta *= (data.max_test_distance_ - data.min_test_distance_);

    double test_size = data.max_test_distance_;
    if (params_.test_distance_changes_with_speed)
    {
      if (params_.test_distance_grows_with_speed)
      {
        test_size = size_delta + data.min_test_distance_;
      }
      else
      {
        test_size = data.max_test_distance_ - size_delta;
      }
      test_size = std::max(std::min(data.max_test_distance_, test_size), data.min_test_distance_);
    }

    double result_speed = calculateResultSpeed(data, data.dist_left_, test_size);
    result_speed = std::min(result_speed, calculateResultSpeed(data, data.dist_right_, test_size));

    if (result_speed > data.min_velocity_ && result_speed <= data.speed_)
    {
      result.speed = result_speed;
      result.limiting = true;
    }

    return result;
  }

  double StaticObjectSpeedLimiter::calculateResultSpeed(const SpeedLimiterData &data, double test_dist, double test_size) const {
    if (test_dist >= data.min_test_distance_ && test_dist <= data.max_test_distance_)
    {
      if (test_dist <= test_size)
      {
        double speed_delta = twoLevelInterpolation(test_size, data.min_test_distance_, data.max_test_distance_, data.min_reduction_,
                                                   data.max_reduction_);
        const double output_speed = data.speed_ - speed_delta;
        if ((output_speed >= data.min_velocity_) && (data.speed_ > output_speed))
        {
          return output_speed;
        }
      }
    }

    return data.speed_;
  }

} /* namespace base_local_planner */
