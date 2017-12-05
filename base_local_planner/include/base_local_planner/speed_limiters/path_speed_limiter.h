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

#ifndef PATH_SPEED_LIMITER_H_
#define PATH_SPEED_LIMITER_H_

#include <base_local_planner/speed_limiters/speed_limiter.h>
#include <base_local_planner/PathSpeedLimiterConfig.h>

namespace base_local_planner {

/**
 * This class slows down the robot when approaching a turn
 *
 */
class PathSpeedLimiter : public SpeedLimiter {
public:
  /**
   * Constructor
   */
  PathSpeedLimiter();

  /**
   * Destructor
   */
  virtual ~PathSpeedLimiter() {}

  virtual initialize() {};

  /**
   * Prepare for operation.
   * @return true if preparations were successful
   */
  virtual bool calculateLimits(double& max_allowed_linear_vel, double& max_allowed_angular_vel);

private:
  void reconfigure(PathSpeedLimiterConfig &cfg, uint32_t level) {
    params_ = cfg;
  }

  double calculateAllowedLinearSpeed(double heading_diff);

  // All of these params need to be filled out.
  PathSpeedLimiterConfig params_;
};

} /* namespace base_local_planner */
#endif /* PATH_SPEED_LIMITER_H_ */
