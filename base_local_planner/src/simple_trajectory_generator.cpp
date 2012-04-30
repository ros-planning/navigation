/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/simple_trajectory_generator.h>

#include <math.h>

#include <base_local_planner/velocity_iterator.h>

namespace base_local_planner {


void SimpleTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const base_local_planner::LocalPlannerLimits* limits,
    const double sim_period,
    const Eigen::Vector3f& acc_lim,
    const Eigen::Vector3f& vsamples) {

  /*
   * We actually generate all velocity sample vectors here, from which to generate trajectories later on
   */
  double max_vel_th = limits->max_rot_vel;
  double min_vel_th = -1.0 * max_vel_th;

  pos_ = pos;
  limits_ = limits;
  next_sample_index_ = 0;
  sample_params_.clear();

  //compute the feasible velocity space based on the rate at which we run
  Eigen::Vector3f max_vel = Eigen::Vector3f::Zero();
  max_vel[0] = std::min(limits->max_vel_x, vel[0] + acc_lim[0] * sim_period);
  max_vel[1] = std::min(limits->max_vel_y, vel[1] + acc_lim[1] * sim_period);
  max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_period);

  Eigen::Vector3f min_vel = Eigen::Vector3f::Zero();
  min_vel[0] = std::max(limits->min_vel_x, vel[0] - acc_lim[0] * sim_period);
  min_vel[1] = std::max(limits->min_vel_y, vel[1] - acc_lim[1] * sim_period);
  min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_period);

  Eigen::Vector3f dv = Eigen::Vector3f::Zero();
  //we want to sample the velocity space regularly
  for(unsigned int i = 0; i < 3; ++i){
    dv[i] = (max_vel[i] - min_vel[i]) / (std::max(1.0, double(vsamples[i]) - 1));
  }
  Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();
  for(VelocityIterator x_it(min_vel[0], max_vel[0], vsamples[0]); !x_it.isFinished(); x_it++) {
    vel_samp[0] = x_it.getVelocity();
    for(VelocityIterator y_it(min_vel[1], max_vel[1], vsamples[1]); !y_it.isFinished(); y_it++) {
      vel_samp[1] = y_it.getVelocity();
      for(VelocityIterator th_it(min_vel[2], max_vel[2], vsamples[2]); !th_it.isFinished(); th_it++) {
        vel_samp[2] = th_it.getVelocity();
        //ROS_DEBUG("Sample %f, %f, %f", vel_samp[0], vel_samp[1], vel_samp[2]);
        sample_params_.push_back(vel_samp);
      }
    }
  }
}

/**
 * Whether this generator can create more trajectories
 */
bool SimpleTrajectoryGenerator::hasMoreTrajectories() {
  return next_sample_index_ < sample_params_.size();
}

/**
 * Create and return the next sample trajectory
 */
bool SimpleTrajectoryGenerator::nextTrajectory(Trajectory &comp_traj) {
  bool result = false;
  if (hasMoreTrajectories()) {
    if (generateTrajectory(
        pos_,
        sample_params_[next_sample_index_],
        limits_,
        sim_time_,
        sim_granularity_,
        comp_traj)) {
      result = true;
    }
  }
  next_sample_index_++;
  return result;
}

bool SimpleTrajectoryGenerator::generateTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f& vel,
      const base_local_planner::LocalPlannerLimits* limits,
      double sim_time,
      double sim_granularity,
      base_local_planner::Trajectory& traj) {
    //ROS_ERROR("%.2f, %.2f, %.2f - %.2f %.2f", vel[0], vel[1], vel[2], sim_time_, sim_granularity_);

    double vmag = sqrt(vel[0] * vel[0] + vel[1] * vel[1]);
    double eps = 1e-4;

    //make sure that the robot would at least be moving with one of
    // the required minimum velocities, but not exceeding any of the maximum
    // velocities
    if ((vmag + eps < limits->min_trans_vel && fabs(vel[2]) + eps < limits->min_rot_vel) ||
        (vmag - eps > limits->max_trans_vel)) {
      traj.cost_ = -1.0;
      return false;
    }

    //compute the number of steps we must take along this trajectory to be "safe"
    double sim_time_distance = (vmag * sim_time); // the distance the robot would travel in sim_time if it did not change velocity
    double sim_time_angle = (fabs(vel[2]) * sim_time); // the angle the robot would rotate in sim_time
    int num_steps =
        ceil(std::max(sim_time_distance / sim_granularity,
                      sim_time_angle / 0.1)); // 0.1 radians is ca. 6 degrees

    //compute a timestep
    double dt = sim_time / num_steps;

    //create a potential trajectory... it might be reused so we'll make sure to reset it
    traj.resetPoints();
    traj.xv_     = vel[0];
    traj.yv_     = vel[1];
    traj.thetav_ = vel[2];
    traj.cost_   = -1.0;

    //if we're not actualy going to simulate... we may as well just return now
    if (num_steps == 0) {
      traj.cost_ = -1.0;
      return false;
    }

    //simulate the trajectory and check for collisions, updating costs along the way
    for (int i = 0; i < num_steps; ++i) {

      //add the point to the trajectory so we can draw it later if we want
      traj.addPoint(pos[0], pos[1], pos[2]);

      //update the position of the robot using the velocities passed in
      pos = computeNewPositions(pos, vel, dt);

    } // end for simulation steps
    return true;
  }

Eigen::Vector3f SimpleTrajectoryGenerator::computeNewPositions(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double dt) {
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

} /* namespace base_local_planner */
