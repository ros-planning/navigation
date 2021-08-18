/*
 * local_planner_limits_test.cpp
 *
 *      Author: dgrieneisen
 */
#include <gtest/gtest.h>

#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/trajectory.h>
#include <geometry_msgs/Twist.h>

#include "critic_test_helpers.h"

namespace base_local_planner {

std::shared_ptr<base_local_planner::LocalPlannerLimits> makeLimits(float max_x, float max_theta)
{
  return std::make_shared<base_local_planner::LocalPlannerLimits>(
    max_x, // max_trans_vel
    0.1, // min_trans_vel
    max_x, // max_x_vel
    0.1, // min_x_vel
    0.0, // max_y_vel
    0.0, // min_y_vel
    max_theta, // max_rot_vel
    0.1, // min_rot_vel
    1.0, // acc_lim_x
    0.0, // acc_lim_y
    2.0, // acc_lim_theta
    1.0, // acc_lim_trans
    0.15, // xy_gal_tolerance
    0.30, // xy_overshoot_tolerance
    0.1,  // yaw_goal_tolerance
    1.0  // stopping_scaling_percent
    );
}

Trajectory createTrajectory(float vx, float vtheta)
{
  Trajectory traj = Trajectory();
  traj.xv_ = vx;
  traj.yv_ = 0;
  traj.thetav_ = vtheta;
  return traj;
}

geometry_msgs::Twist createTestTwist(float vx, float vtheta)
{
  auto twist = geometry_msgs::Twist();
  twist.linear.x = vx;
  twist.linear.y = 0;
  twist.angular.z = vtheta;
  return twist;
}

TEST(LocalPlannerLimits, applyNoLimiting){
  auto lims = makeLimits(1.3, 0.75);
  auto traj = createTrajectory(1.0, 0.5);

  EXPECT_NEAR(lims->max_vel_x, 1.3, 1e-5);
  EXPECT_NEAR(lims->max_rot_vel, 0.75, 1e-5);

  lims->applyToTrajectory(traj);
  EXPECT_NEAR(traj.xv_, 1.0, 1e-5);
  EXPECT_NEAR(traj.thetav_, 0.5, 1e-5);
}

TEST(LocalPlannerLimits, limitV){
  float vstart = 1.5;
  float wstart = 0.5;
  auto lims = makeLimits(1.3, 0.75);
  auto traj = createTrajectory(vstart, wstart);

  lims->applyToTrajectory(traj);
  EXPECT_NEAR(traj.xv_, 1.3, 1e-5);
  EXPECT_NEAR(traj.thetav_, wstart / vstart * traj.xv_, 1e-5);
}

TEST(LocalPlannerLimits, limitVwithW0){
  float vstart = 1.5;
  float wstart = 0.0;
  auto lims = makeLimits(1.3, 0.75);
  auto traj = createTrajectory(vstart, wstart);

  lims->applyToTrajectory(traj);
  EXPECT_NEAR(traj.xv_, 1.3, 1e-5);
  EXPECT_NEAR(traj.thetav_, 0, 1e-5);
}

TEST(LocalPlannerLimits, limitW){
  float vstart = 1.3;
  float wstart = 1.0;
  auto lims = makeLimits(1.3, 0.75);
  auto traj = createTrajectory(vstart, wstart);

  lims->applyToTrajectory(traj);
  EXPECT_NEAR(traj.thetav_, 0.75, 1e-5);
  EXPECT_NEAR(traj.xv_, vstart / wstart * traj.thetav_, 1e-5);
}

TEST(LocalPlannerLimits, limitVwithV0){
  float vstart = 0.0;
  float wstart = 1.0;
  auto lims = makeLimits(1.3, 0.75);
  auto traj = createTrajectory(vstart, wstart);

  lims->applyToTrajectory(traj);
  EXPECT_NEAR(traj.thetav_, 0.75, 1e-5);
  EXPECT_NEAR(traj.xv_, 0.0, 1e-5);
}

TEST(LocalPlannerLimits, limitBoth){
  float vstart = 2.0;
  float wstart = 1.0;
  auto lims = makeLimits(1.3, 0.75);
  auto traj = createTrajectory(vstart, wstart);

  lims->applyToTrajectory(traj);
  EXPECT_LE(traj.xv_, 1.3);
  EXPECT_LE(traj.thetav_, 0.75);
  EXPECT_NEAR(traj.xv_ / traj.thetav_, vstart / wstart, 1e-5);
}

TEST(LocalPlannerLimits, limitBothTwist){
  float vstart = 2.0;
  float wstart = 1.0;
  auto lims = makeLimits(1.3, 0.75);
  auto twist = createTestTwist(vstart, wstart);

  lims->applyToTwist(twist);
  EXPECT_LE(twist.linear.x, 1.3);
  EXPECT_LE(twist.angular.z, 0.75);
  EXPECT_NEAR(twist.linear.x / twist.angular.z, vstart / wstart, 1e-5);
}

}
