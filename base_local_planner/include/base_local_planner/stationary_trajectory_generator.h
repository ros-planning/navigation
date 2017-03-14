/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017 6 River Systems
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
 * Author: Dgrieneisen
 *********************************************************************/

#ifndef STATIONARY_TRAJECTORY_GENERATOR_H_
#define STATIONARY_TRAJECTORY_GENERATOR_H_

#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/local_planner_limits.h>
#include <Eigen/Core>

namespace base_local_planner {

/**
 * generates a single sample trajectory if the robot isn't moving to remain stationary
 */
class StationaryTrajectoryGenerator: public base_local_planner::TrajectorySampleGenerator {
public:

  StationaryTrajectoryGenerator() {
    limits_ = NULL;
  }

  ~StationaryTrajectoryGenerator() {}

  /**
   * @param pos current robot position
   * @param vel current robot velocity
   * @param limits Current velocity limits
   */
  void initialise(
      const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel,
      base_local_planner::LocalPlannerLimits* limits);

  /**
   * Whether this generator can create more trajectories
   */
  bool hasMoreTrajectories();

  /**
   * Whether this generator can create more trajectories
   */
  bool nextTrajectory(Trajectory &traj);

  bool generateTrajectory(
        Eigen::Vector3f pos,
        Eigen::Vector3f vel,
        base_local_planner::Trajectory& traj);

  virtual double getStartLinearVelocity() { return vel_[0];};
  virtual double getStartAngularVelocity() { return vel_[2];};

protected:
  bool trajectory_generated_;
  unsigned int next_sample_index_;

  base_local_planner::LocalPlannerLimits* limits_;
  Eigen::Vector3f pos_;
  Eigen::Vector3f vel_;

};

} /* namespace base_local_planner */
#endif /* STATIONARY_TRAJECTORY_GENERATOR_H_ */
