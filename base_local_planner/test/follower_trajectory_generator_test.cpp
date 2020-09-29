/*
 * follower_trajectory_test.cpp
 *
 *      Author: dgrieneisen
 */
#include <gtest/gtest.h>

#include <base_local_planner/follower_trajectory_generator.h>
#include <base_local_planner/local_planner_limits.h>

#include "critic_test_helpers.h"

namespace base_local_planner {

std::shared_ptr<base_local_planner::LocalPlannerLimits> getLimits()
{
  return std::make_shared<base_local_planner::LocalPlannerLimits>(
    1.0, // max_trans_vel
    0.1, // min_trans_vel
    1.0, // max_x_vel
    0.1, // min_x_vel
    0.0, // max_y_vel
    0.0, // min_y_vel
    2.0, // max_rot_vel
    0.1, // min_rot_vel
    1.0, // acc_lim_x
    0.0, // acc_lim_y
    2.0, // acc_lim_theta
    1.0, // acc_lim_trans
    0.15, // xy_gal_tolerance,
    0.3, // overshoot_tolerance
    0.1  // yaw_goal_tolerance
    );
}

FollowerTrajectoryGenerator createFTG(base_local_planner::LocalPlannerLimits* lim)
{
  FollowerTrajectoryGenerator ftg = FollowerTrajectoryGenerator();
  ftg.setParameters(
    2.0, // sim_time
    0.1, // sim_granularity
    10.0, // kp_theta
    0.5, // lookahead_distance
    2 // num_trajectories
    );
  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  Eigen::Vector3f vel = Eigen::Vector3f::Zero();
  ftg.initialise(pos, vel, lim);
  return ftg;
}

TEST(FollowerTrajectoryGenerator, basic){
  auto lims = getLimits();
  auto ftg = createFTG(lims.get());
  EXPECT_EQ(0, 0);
}

TEST(FollowerTrajectoryGenerator, follow_on_test){
  auto lims = getLimits();
  auto ftg = createFTG(lims.get());

  // Add a global plan
  ftg.setGlobalPlan(createGlobalPlan());

  // Ask for a trajectory.
  base_local_planner::Trajectory traj;

  bool res = ftg.generateTrajectory(
    createVector(0, 0.1, 0), // pose
    createVector(0.5, 0, 0), // velocity
    0.7, // sim_time
    traj
    );
  printTrajectory(traj);
  EXPECT_TRUE(res);

  res = ftg.generateTrajectory(
    createVector(0, 0.1, 0), // pose
    createVector(0.5, 0, 0), // velocity
    1.7, // sim_time
    traj
    );
  printTrajectory(traj);
  EXPECT_TRUE(res);
}


TEST(FollowerTrajectoryGenerator, near_end_test){
  auto lims = getLimits();
  auto ftg = createFTG(lims.get());

  // Add a global plan
  ftg.setGlobalPlan(createGlobalPlan());

  // Ask for a trajectory.
  base_local_planner::Trajectory traj;

  bool res = ftg.generateTrajectory(
    createVector(0.0, 0.1, 0), // pose
    createVector(0.5, 0, 0), // velocity
    5.0, // sim_time
    traj
    );
  printTrajectory(traj);
  EXPECT_TRUE(res);
}


}
