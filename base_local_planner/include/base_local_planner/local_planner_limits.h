/***********************************************************
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 ***********************************************************/


#ifndef __base_local_planner__LOCALPLANNERLIMITS_H__
#define __base_local_planner__LOCALPLANNERLIMITS_H__

#include <Eigen/Core>

#include <base_local_planner/trajectory.h>
#include <geometry_msgs/Twist.h>

namespace base_local_planner
{
class LocalPlannerLimits
{
public:

  double max_trans_vel;
  double min_trans_vel;
  double max_vel_x;
  double min_vel_x;
  double max_vel_y;
  double min_vel_y;
  double max_rot_vel;
  double min_rot_vel;
  double max_tip_vel;
  double min_tip_vel;
  double acc_lim_x;
  double acc_lim_y;
  double acc_lim_theta;
  double acc_limit_trans;
  double acc_limit_tip;
//  double jerk_lim_trans;
//  double jerk_lim_rot;
  bool   prune_plan;
  double xy_goal_tolerance;
  double xy_goal_overshoot_tolerance;
  double yaw_goal_tolerance;
  double trans_stopped_vel;
  double rot_stopped_vel;
  bool   restore_defaults;
  double stopping_scaling_percent;

  LocalPlannerLimits() {}

  LocalPlannerLimits(
      double nmax_trans_vel,
      double nmin_trans_vel,
      double nmax_vel_x,
      double nmin_vel_x,
      double nmax_vel_y,
      double nmin_vel_y,
      double nmax_rot_vel,
      double nmin_rot_vel,
      double nacc_lim_x,
      double nacc_lim_y,
      double nacc_lim_theta,
      double nacc_limit_trans,
      double nxy_goal_tolerance,
      double nxy_goal_overshoot_tolerance,
      double nyaw_goal_tolerance,
      double nstopping_scaling_percent,
//      double njerk_lim_trans = -1,
//      double njerk_lim_rot = -1,
      bool   nprune_plan = true,
      double ntrans_stopped_vel = 0.1,
      double nrot_stopped_vel = 0.1,
      double nmax_tip_vel = 0.5,
      double nmin_tip_vel = 0.01,
      double nacc_limit_tip = 0.5):
        max_trans_vel(nmax_trans_vel),
        min_trans_vel(nmin_trans_vel),
        max_vel_x(nmax_vel_x),
        min_vel_x(nmin_vel_x),
        max_vel_y(nmax_vel_y),
        min_vel_y(nmin_vel_y),
        max_rot_vel(nmax_rot_vel),
        min_rot_vel(nmin_rot_vel),
        max_tip_vel(nmax_tip_vel),
        min_tip_vel(nmin_tip_vel),
        acc_lim_x(nacc_lim_x),
        acc_lim_y(nacc_lim_y),
        acc_lim_theta(nacc_lim_theta),
        acc_limit_trans(nacc_limit_trans),
        acc_limit_tip(nacc_limit_tip),
//        jerk_lim_trans(njerk_lim_trans),
//        jerk_lim_rot(njerk_lim_rot),
        prune_plan(nprune_plan),
        xy_goal_tolerance(nxy_goal_tolerance),
        xy_goal_overshoot_tolerance(nxy_goal_overshoot_tolerance),
        yaw_goal_tolerance(nyaw_goal_tolerance),
        trans_stopped_vel(ntrans_stopped_vel),
        rot_stopped_vel(nrot_stopped_vel),
        stopping_scaling_percent(nstopping_scaling_percent) {}

  ~LocalPlannerLimits() {}

  /**
   * @brief  Get the acceleration limits of the robot
   * @return  The acceleration limits of the robot
   */
  Eigen::Vector3f getAccLimits() {
    Eigen::Vector3f acc_limits;
    acc_limits[0] = acc_lim_x;
    acc_limits[1] = acc_lim_y;
    acc_limits[2] = acc_lim_theta;
    return acc_limits;
  }

  void applyToTwist(geometry_msgs::Twist& twist) {
    float vx = twist.linear.x;
    float vy = twist.linear.y;
    float vtheta = twist.angular.z;

    applyToVelocities(vx, vy, vtheta);
    
    twist.linear.x = vx;
    twist.linear.y = vy;
    twist.angular.z = vtheta;
  }
  
  void applyToTrajectory(base_local_planner::Trajectory& traj) {
    float vx = traj.xv_;
    float vy = traj.yv_;
    float vtheta = traj.thetav_;

    applyToVelocities(vx, vy, vtheta);

    traj.xv_ = vx;
    traj.yv_ = vy;
    traj.thetav_ = vtheta;
  }


  void applyToVelocities(float& vx, float& vy, float& vtheta) {
    float vx_start = vx;
    float vy_start = vy;
    float vtheta_start = vtheta;
    
    // First, correct x
    if (fabs(vx) > max_vel_x) {
      vx = vx > 0 ? max_vel_x : -max_vel_x;
    }
    float vx_mid = vx;
    // Then scale theta by change to x
    if (fabs(vx_start) > 1e-3) {
      vtheta = vx / vx_start * vtheta_start;
    }
    float vtheta_mid = vtheta;
    // Then limit theta
    if (fabs(vtheta) > max_rot_vel) {
      vtheta = vtheta > 0 ? max_rot_vel : -max_rot_vel;
    }
    // Then scale x back by change to theta
    if (fabs(vtheta_mid) > 1e-3) {
      vx = vtheta / vtheta_mid * vx_mid;
    }
  }

};

}
#endif // __LOCALPLANNERLIMITS_H__
