/*
 * speed_cost_function test
 *
 *      Author: dgrieneisen
 */
#include <gtest/gtest.h>

#include <base_local_planner/critics/speed_cost_function.h>

#include "critic_test_helpers.h"

namespace base_local_planner {

std::string WORLD_FRAME = "odom";
std::string BODY_FRAME = "base_link";

SpeedCostFunction createSCF()
{
  SpeedCostFunction cf = SpeedCostFunction();
  cf.setWorldFrameId(WORLD_FRAME);
  cf.setMaxLinearVelocity(1.0);
  cf.setMinLinearVelocity(0.3);
  cf.setAcceleration(0.7);
  cf.setXBuffer(0.1);
  cf.setYBuffer(0.1);
  cf.setHalfAngle(1.6);
  cf.setFootprint(createFootprint(0.5, 0.5));

  return cf;
}

// Series of tests:
/// Test no obstructions
/// Test obstructions outside of half angle
/// Test obstruction far away
/// Test obstructions inside circumcribed radius
/// Test obstruction mid-range
/// Test obstruction wrong type
/// Test obstructions with non-zero location, same frame_id
/// Test obstructions with non-zero location, different frame_id


TEST(SpeedCostFunction, NoObstructions)
{
  ros::Time::init();
  // Create the cost function
  SpeedCostFunction cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createPoseStamped(0, 0, 0));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  // obstructions->push_back(createObstruction());

  cf.prepare();

  // Create a trajectory to test
  Trajectory traj = createTrajectory(0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.5, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.0, 1.0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 1.0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);
}

TEST(SpeedCostFunction, IgnorableObstructionBehindBot)
{
  ros::Time::init();
  // Create the cost function
  SpeedCostFunction cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createPoseStamped(0, 0, 0));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  obstructions->push_back(createObstructionMsg(-1.0, 0, WORLD_FRAME, costmap_2d::ObstructionMsg::DYNAMIC, false));
  cf.setObstructions(obstructions);

  cf.prepare();

  // Create a trajectory to test
  Trajectory traj = createTrajectory(0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.5, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.0, 1.0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 1.0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);
}

TEST(SpeedCostFunction, IgnorableObstructionFarAway)
{
  ros::Time::init();
  // Create the cost function
  SpeedCostFunction cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createPoseStamped(0, 0, 0));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  obstructions->push_back(createObstructionMsg(3.0, 0, WORLD_FRAME, costmap_2d::ObstructionMsg::DYNAMIC, false));
  cf.setObstructions(obstructions);

  cf.prepare();

  // Create a trajectory to test
  Trajectory traj = createTrajectory(0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.5, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.0, 1.0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 1.0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);
}

TEST(SpeedCostFunction, ObstructionInRadius)
{
  ros::Time::init();
  // Create the cost function
  SpeedCostFunction cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createPoseStamped(0, 0, 0));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  obstructions->push_back(createObstructionMsg(0.1, 0, WORLD_FRAME, costmap_2d::ObstructionMsg::DYNAMIC, false));
  cf.setObstructions(obstructions);

  cf.prepare();

  // Create a trajectory to test
  Trajectory traj = createTrajectory(0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.5, 0);
  EXPECT_FALSE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 0);
  EXPECT_FALSE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.0, 1.0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 1.0);
  EXPECT_FALSE(cf.scoreTrajectory(traj) >= 0);
}

TEST(SpeedCostFunction, ObstructionMidRange)
{
  ros::Time::init();
  // Create the cost function
  SpeedCostFunction cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createPoseStamped(0, 0, 0));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  obstructions->push_back(createObstructionMsg(1.0, 0, WORLD_FRAME, costmap_2d::ObstructionMsg::DYNAMIC, false));
  cf.setObstructions(obstructions);

  cf.prepare();

  // Create a trajectory to test
  Trajectory traj = createTrajectory(0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.5, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 0);
  EXPECT_FALSE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.0, 1.0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 1.0);
  EXPECT_FALSE(cf.scoreTrajectory(traj) >= 0);
}

TEST(SpeedCostFunction, ObstructionWrongType)
{
  ros::Time::init();
  // Create the cost function
  SpeedCostFunction cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createPoseStamped(0, 0, 0));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  obstructions->push_back(createObstructionMsg(-1.0, 0, WORLD_FRAME, costmap_2d::ObstructionMsg::PSEUDOSTATIC, false));
  cf.setObstructions(obstructions);

  cf.prepare();

  // Create a trajectory to test
  Trajectory traj = createTrajectory(0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.5, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.0, 1.0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 1.0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);
}

TEST(SpeedCostFunction, MovedPose_BodyFrame)
{
  ros::Time::init();
  // Create the cost function
  SpeedCostFunction cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createPoseStamped(1, 2, 1.57));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  obstructions->push_back(createObstructionMsg(1.0, 0, BODY_FRAME, costmap_2d::ObstructionMsg::DYNAMIC, false));
  cf.setObstructions(obstructions);

  cf.prepare();

  // Create a trajectory to test
  Trajectory traj = createTrajectory(0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.5, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 0);
  EXPECT_FALSE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.0, 1.0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 1.0);
  EXPECT_FALSE(cf.scoreTrajectory(traj) >= 0);
}

TEST(SpeedCostFunction, MovedPose_WorldFrame)
{
  ros::Time::init();
  // Create the cost function
  SpeedCostFunction cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createPoseStamped(1, 2, 1.57));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  obstructions->push_back(createObstructionMsg(1.0, 3.0, WORLD_FRAME, costmap_2d::ObstructionMsg::DYNAMIC, false));
  cf.setObstructions(obstructions);

  cf.prepare();

  // Create a trajectory to test
  Trajectory traj = createTrajectory(0, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.5, 0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 0);
  EXPECT_FALSE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(0.0, 1.0);
  EXPECT_TRUE(cf.scoreTrajectory(traj) >= 0);

  traj = createTrajectory(1.0, 1.0);
  EXPECT_FALSE(cf.scoreTrajectory(traj) >= 0);
}

}
