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


#ifndef STATIC_OBJECT_SPEED_LIMITER_TEST_H_
#define STATIC_OBJECT_SPEED_LIMITER_TEST_H_
#include <gtest/gtest.h>
#include <base_local_planner/speed_limiters/static_object_speed_limiter.h>

namespace base_local_planner
{
  /**
 * This class provides a test wrapper for the StaticObjectSpeedLimiter class
 */
  class StaticObjectSpeedLimiter_TEST : public StaticObjectSpeedLimiter
  {
  public:
    struct SpeedLimiterTestData
    {
      bool enabledfirmwareVersion = true;
      srs::Velocity<> velocity;
      double distance_from_static_left = 0.0;
      double distance_from_static_right = 0.0;
      srs::ChuckChassisGenerations::ChuckChassisType chassis_generation_ =
          srs::ChuckChassisGenerations::ChuckChassisType::INVALID;
    };

    StaticObjectSpeedLimiter_TEST(costmap_2d::Costmap2DROS *costmap) : StaticObjectSpeedLimiter(costmap){};

    void getLimits_Test(double &max_allowed_linear_vel, double &max_allowed_angular_vel);

    bool calculateLimits_Test(const SpeedLimiterTestData *data, double &max_allowed_linear_vel, double &max_allowed_angular_vel);

    virtual bool calculateLimits(double &max_allowed_linear_vel, double &max_allowed_angular_vel) override;

  private:
    const SpeedLimiterTestData* test_data_ = nullptr;
  };

} /* namespace base_local_planner */
#endif /* STATIC_OBJECT_SPEED_LIMITER_TEST_H_ */