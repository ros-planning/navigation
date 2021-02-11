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

#ifndef STATIC_SPEED_LIMITER_H_
#define STATIC_SPEED_LIMITER_H_

#include <base_local_planner/speed_limiters/speed_limiter.h>
#include <base_local_planner/StaticSpeedLimiterConfig.h>
#include <costmap_2d/ObstructionMsg.h>
#include <tf/tf.h>

namespace base_local_planner {

/**
 * This class provides cost speed and distance from static obstacles
 *
 * It will limit the speed near static obstacle to limit false safety trips
 */
class StaticSpeedLimiter : public SpeedLimiter {
public:
  /**
   * Constructor
   */
  StaticSpeedLimiter(costmap_2d::Costmap2DROS* costmap) : SpeedLimiter(costmap) {};

  /**
   * Destructor
   */
  virtual ~StaticSpeedLimiter() {}

  virtual void initialize(std::string name);

  /**
   * Prepare for operation.
   * @return true if preparations were successful
   */
  virtual bool calculateLimits(double& max_allowed_linear_vel, double& max_allowed_angular_vel);

  std::string getName();
  
private:
  void reconfigure(StaticSpeedLimiterConfig cfg) {
    params_ = cfg;
  };

  struct LinearSpeedLimiterResult {
      double speed = 0.0;
      double heading = 0.0;
      double distance = 0.0;
      bool limiting = false;
  };

  double getBearingToObstacle(const costmap_2d::ObstructionMsg& obs);

  LinearSpeedLimiterResult calculateAllowedLinearSpeed(const costmap_2d::ObstructionMsg& obs);

  double calculateAllowedAngularSpeed(const costmap_2d::ObstructionMsg& obs);

  costmap_2d::ObstructionMsg obstructionToBodyFrame(const costmap_2d::ObstructionMsg& in,
    const tf::Pose& current_pose_inv_tf);

  void calculateFootprintBounds(const std::vector<geometry_msgs::Point>& footprint);

  std::shared_ptr<dynamic_reconfigure::Client<StaticSpeedLimiterConfig>> configClient_;
  StaticSpeedLimiterConfig params_;

  double footprint_min_x_ = -0.4;
  double footprint_max_x_ = 0.4;
  double footprint_min_y_ = -0.4;
  double footprint_max_y_ = 0.4;
  double circumscribed_radius_ = 0.7;
  ros::Publisher static_pub;
};

} /* namespace base_local_planner */
#endif /* STATIC_SPEED_LIMITER_H_ */
