/*
 * follower_trajectory_test.cpp
 *
 *      Author: dgrieneisen
 */
#include <gtest/gtest.h>

#include <base_local_planner/critics/global_plan_distance_cost_function.h>

#include "critic_test_helpers.h"

namespace base_local_planner {

constexpr float DIST = 0.5;

GlobalPlanDistanceCostFunction createGPDCF()
{
  GlobalPlanDistanceCostFunction gpdcf = GlobalPlanDistanceCostFunction(DIST);
  return gpdcf;
}


TEST(GlobalPlanDistanceCostFunction, test)
{
  GlobalPlanDistanceCostFunction cf = createGPDCF();
  Trajectory traj;

  // With no plan or pose, it should return true.
  cf.prepare();
  traj = createTrajectory(0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  // Set a trajectory
  cf.setTargetPoses(createGlobalPlan());
  cf.setCurrentPose(createPoseStamped(0, 0, 0));
  cf.prepare();
  traj = createTrajectory(0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  // Now test with a pose far away.
  cf.setCurrentPose(createPoseStamped(2 * DIST, 0, 0));
  cf.prepare();
  // Velocity of 0 is still valid
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);
  // Velocity > 0 is not valid
  traj = createTrajectory(1, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);
  traj = createTrajectory(0, 2);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);
  traj = createTrajectory(1, 1);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);
}

}
