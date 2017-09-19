/*
 * speed_cost_function test
 *
 *      Author: dgrieneisen
 */
#include <gtest/gtest.h>

#include <base_local_planner/speed_limiter.h>

#include "critic_test_helpers.h"

namespace base_local_planner {

std::string WORLD_FRAME = "odom";
std::string BODY_FRAME = "base_link";

SpeedLimiter createSCF()
{
  SpeedLimiter cf = SpeedLimiter();
  SpeedLimiterParams sp_params;
  sp_params.max_linear_velocity_ = 1.0;
  sp_params.min_linear_velocity_ = 0.3;
  sp_params.linear_acceleration_ = 0.7;
  sp_params.x_buffer_ = 0.1;
  sp_params.y_buffer_ = 0.1;
  sp_params.half_angle_ = 1.6;

  sp_params.min_angular_velocity_effect_distance_ = 0.1;
  sp_params.max_angular_velocity_effect_distance_ = 0.3;
  sp_params.min_angular_velocity_ = 0.3;
  sp_params.max_angular_velocity_ = 1.0;

  cf.setParams(sp_params);
  cf.setWorldFrameId(WORLD_FRAME);
  cf.setBodyFrameId(BODY_FRAME);
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


TEST(SpeedLimiter, NoObstructions)
{
  ros::Time::init();
  // Create the cost function
  SpeedLimiter cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createTfPoseStamped(0, 0, 0));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  // obstructions->push_back(createObstruction());

  double v_lim, w_lim;
  cf.calculateLimits(v_lim, w_lim);

  EXPECT_GE(v_lim, 0.0);
  EXPECT_GE(v_lim, 0.5);
  EXPECT_GE(v_lim, 1.0);

  EXPECT_GE(w_lim, 0.0);
  EXPECT_GE(w_lim, 0.5);
  EXPECT_GE(w_lim, 1.0);
}

TEST(SpeedLimiter, IgnorableObstructionBehindBot)
{
  ros::Time::init();
  // Create the cost function
  SpeedLimiter cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createTfPoseStamped(0, 0, 0));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  obstructions->push_back(createObstructionMsg(-1.0, 0, WORLD_FRAME, costmap_2d::ObstructionMsg::DYNAMIC, false));
  cf.setObstructions(obstructions);

  double v_lim, w_lim;
  cf.calculateLimits(v_lim, w_lim);

  EXPECT_GE(v_lim, 0.0);
  EXPECT_GE(v_lim, 0.5);
  EXPECT_GE(v_lim, 1.0);

  EXPECT_GE(w_lim, 0.0);
  EXPECT_GE(w_lim, 0.5);
  EXPECT_GE(w_lim, 1.0);
}

TEST(SpeedLimiter, IgnorableObstructionFarAway)
{
  ros::Time::init();
  // Create the cost function
  SpeedLimiter cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createTfPoseStamped(0, 0, 0));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  obstructions->push_back(createObstructionMsg(3.0, 0, WORLD_FRAME, costmap_2d::ObstructionMsg::DYNAMIC, false));
  cf.setObstructions(obstructions);

  double v_lim, w_lim;
  cf.calculateLimits(v_lim, w_lim);

  EXPECT_GE(v_lim, 0.0);
  EXPECT_GE(v_lim, 0.5);
  EXPECT_GE(v_lim, 1.0);

  EXPECT_GE(w_lim, 0.0);
  EXPECT_GE(w_lim, 0.5);
  EXPECT_GE(w_lim, 1.0);
}

TEST(SpeedLimiter, ObstructionNearby)
{
  ros::Time::init();
  // Create the cost function
  SpeedLimiter cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createTfPoseStamped(0, 0, 0));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  obstructions->push_back(createObstructionMsg(0.1, 0, WORLD_FRAME, costmap_2d::ObstructionMsg::DYNAMIC, false));
  cf.setObstructions(obstructions);

  double v_lim, w_lim;
  cf.calculateLimits(v_lim, w_lim);

  EXPECT_GE(v_lim, 0.0);
  EXPECT_LE(v_lim, 0.5);
  EXPECT_LE(v_lim, 1.0);

  EXPECT_GE(w_lim, 0.0);
  EXPECT_LE(w_lim, 0.5);
  EXPECT_LE(w_lim, 1.0);
}

TEST(SpeedLimiter, ObstructionMidRange)
{
  ros::Time::init();
  // Create the cost function
  SpeedLimiter cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createTfPoseStamped(0, 0, 0));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  obstructions->push_back(createObstructionMsg(1.0, 0, WORLD_FRAME, costmap_2d::ObstructionMsg::DYNAMIC, false));
  cf.setObstructions(obstructions);

  double v_lim, w_lim;
  cf.calculateLimits(v_lim, w_lim);

  EXPECT_GE(v_lim, 0.0);
  EXPECT_GE(v_lim, 0.5);
  EXPECT_LE(v_lim, 1.0);

  EXPECT_GE(w_lim, 0.0);
  EXPECT_GE(w_lim, 0.5);
  EXPECT_LE(w_lim, 1.0);
}

TEST(SpeedLimiter, ObstructionWrongType)
{
  ros::Time::init();
  // Create the cost function
  SpeedLimiter cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createTfPoseStamped(0, 0, 0));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  obstructions->push_back(createObstructionMsg(-1.0, 0, WORLD_FRAME, costmap_2d::ObstructionMsg::PSEUDOSTATIC, false));
  cf.setObstructions(obstructions);

  double v_lim, w_lim;
  cf.calculateLimits(v_lim, w_lim);

  EXPECT_GE(v_lim, 0.0);
  EXPECT_GE(v_lim, 0.5);
  EXPECT_GE(v_lim, 1.0);

  EXPECT_GE(w_lim, 0.0);
  EXPECT_GE(w_lim, 0.5);
  EXPECT_GE(w_lim, 1.0);
}

TEST(SpeedLimiter, MovedPose_BodyFrame)
{
  ros::Time::init();
  // Create the cost function
  SpeedLimiter cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createTfPoseStamped(1, 2, 1.57));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  obstructions->push_back(createObstructionMsg(1.0, 0, BODY_FRAME, costmap_2d::ObstructionMsg::DYNAMIC, false));
  cf.setObstructions(obstructions);

  double v_lim, w_lim;
  cf.calculateLimits(v_lim, w_lim);

  EXPECT_GE(v_lim, 0.0);
  EXPECT_GE(v_lim, 0.5);
  EXPECT_LE(v_lim, 1.0);

  EXPECT_GE(w_lim, 0.0);
  EXPECT_GE(w_lim, 0.5);
  EXPECT_LE(w_lim, 1.0);
}

TEST(SpeedLimiter, MovedPose_WorldFrame)
{
  ros::Time::init();
  // Create the cost function
  SpeedLimiter cf = createSCF();

  // Set up test variables (pose, obstructions, etc...)
  cf.setCurrentPose(createTfPoseStamped(1, 2, 1.57));
  auto obstructions = std::make_shared<std::vector<costmap_2d::ObstructionMsg>>();
  obstructions->push_back(createObstructionMsg(1.0, 3.0, WORLD_FRAME, costmap_2d::ObstructionMsg::DYNAMIC, false));
  cf.setObstructions(obstructions);

  double v_lim, w_lim;
  cf.calculateLimits(v_lim, w_lim);

  EXPECT_GE(v_lim, 0.0);
  EXPECT_GE(v_lim, 0.5);
  EXPECT_LE(v_lim, 1.0);

  EXPECT_GE(w_lim, 0.0);
  EXPECT_GE(w_lim, 0.5);
  EXPECT_LE(w_lim, 1.0);
}

}
