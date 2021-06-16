/**
 *static_object_speed_limiter_test.cpp
 *
 *      Author: Tom Preisner
 */
#include <gtest/gtest.h>
#include <base_local_planner/speed_limiters/static_object_speed_limiter.h>
#include <base_local_planner/speed_limiters/safety_ce_bounds_data.h>
#include "static_object_speed_limiter_test.h"

namespace base_local_planner {
class StaticObjectSpeedLimiterTestHarness {
public:
  StaticObjectSpeedLimiterTestHarness();

  void getLimits_Test(double& max_allowed_linear_vel, double& max_allowed_angular_vel);
  bool calculateLimits_Test(const StaticObjectSpeedLimiter_TEST::SpeedLimiterTestData* data,
                            double& max_allowed_linear_vel, double& max_allowed_angular_vel);

private:
  StaticObjectSpeedLimiter_TEST speed_limiter_test_ = StaticObjectSpeedLimiter_TEST(nullptr);
};

/**
 * Constructor
 */
StaticObjectSpeedLimiterTestHarness::StaticObjectSpeedLimiterTestHarness() {
  StaticObjectSpeedLimiterConfig cfg;
  cfg.enabled = true;
  cfg.timeout = 0.0;  // Always run
  cfg.min_linear_velocity = 0.3;
  cfg.min_angular_velocity = 0.4;
  cfg.test_distance_changes_with_speed = true;
  cfg.test_distance_grows_with_speed = true;
  cfg.min_linear_velocity_test_speed = 0.3;
  cfg.max_linear_velocity_test_speed = 1.35;
  cfg.min_angular_velocity_test_speed = 0.1;
  cfg.max_angular_velocity_test_speed = 0.8;
  cfg.min_linear_velocity_distance = 0.05;
  cfg.max_linear_velocity_distance = 0.75;
  cfg.min_angular_velocity_distance = 0.05;
  cfg.max_angular_velocity_distance = 0.75;
  cfg.min_linear_velocity_reduction = 0.05;
  cfg.max_linear_velocity_reduction = 0.2;
  cfg.min_angular_velocity_reduction = 0.05;
  cfg.max_angular_velocity_reduction = 0.2;
  speed_limiter_test_.reconfigure(cfg);
  speed_limiter_test_.setMaxLimits(1.3, 0.7); //< default high
}

/**
 * Forwarding helper functions
 */
void StaticObjectSpeedLimiterTestHarness::getLimits_Test(double& max_allowed_linear_vel,
                                                         double& max_allowed_angular_vel) {
  speed_limiter_test_.getLimits_Test(max_allowed_linear_vel, max_allowed_angular_vel);
}

bool StaticObjectSpeedLimiterTestHarness::calculateLimits_Test(
    const StaticObjectSpeedLimiter_TEST::SpeedLimiterTestData* data, double& max_allowed_linear_vel,
    double& max_allowed_angular_vel) {
  return speed_limiter_test_.calculateLimits_Test(data, max_allowed_linear_vel, max_allowed_angular_vel);
}


void StaticObjectSpeedLimiter_TEST::getLimits_Test(double &max_allowed_linear_vel, double &max_allowed_angular_vel)
{
  max_allowed_linear_vel = max_linear_velocity_;
  max_allowed_angular_vel = max_angular_velocity_;
}

bool StaticObjectSpeedLimiter_TEST::calculateLimits_Test(const SpeedLimiterTestData *data,
                                                          double &max_allowed_linear_vel,
                                                          double &max_allowed_angular_vel)
{
  assert(test_data_ == nullptr);
  assert(data != nullptr);

  test_data_ = data;
  const bool result = calculateLimits(max_allowed_linear_vel, max_allowed_angular_vel);
  test_data_ = nullptr;

  return result;
}

bool StaticObjectSpeedLimiter_TEST::calculateLimits(double &max_allowed_linear_vel, double &max_allowed_angular_vel)
{
  // Reset the maximum allowed velocity
  max_allowed_linear_vel = max_linear_velocity_;
  max_allowed_angular_vel = max_angular_velocity_;

  if (const char *env_var = std::getenv("ROSEVN_STATIC_OBJECT_LIMITER_ENABLED"))
  {
    std::string val(env_var);

    std::transform(val.begin(), val.end(), val.begin(), ::tolower);
    if (val == "false")
    {
      return true;
    }
  }

  // Cache off the current velocity so it can't be changed during processing
  float currLinearVel = test_data_->velocity.linear;
  float currAngularVel = test_data_->velocity.angular;
  float flipAngular = test_data_->velocity.angular < 0.0;

  int linearIndex, angularIndex = 0;
  for (int i = 0; i < NUM_PLANE_LINEAR_SPEEDS; ++i)
  {
    if (LINEAR_SPEED_TO_INDEX_ARRAY[i] > currLinearVel)
    {
      break;
    }
    linearIndex = i;
  }

  for (int i = 0; i < NUM_PLANE_ANGULAR_SPEEDS; ++i)
  {
    if (ANGULAR_SPEED_TO_INDEX_ARRAY[i] > currAngularVel)
    {
      break;
    }
    angularIndex = i;
  }

  geometry_msgs::Point left, right;
  srs::ChuckChassisGenerations::ChuckChassisType chassis = test_data_->chassis_generation_;

  // The plane tables indicate left right pairs of x,y coords
  if (chassis == srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5_PLUS)
  {
    left.x = LOOKUP_TABLE_CHUCK_PLUS_FRONT_PLANE[linearIndex][angularIndex][0][0];
    left.y = LOOKUP_TABLE_CHUCK_PLUS_FRONT_PLANE[linearIndex][angularIndex][0][1];
    right.x = LOOKUP_TABLE_CHUCK_PLUS_FRONT_PLANE[linearIndex][angularIndex][1][0];
    right.y = LOOKUP_TABLE_CHUCK_PLUS_FRONT_PLANE[linearIndex][angularIndex][1][1];
  }
  else if (chassis == srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5)
  {
    left.x = LOOKUP_TABLE_CHUCK_FRONT_PLANE[linearIndex][angularIndex][0][0];
    left.y = LOOKUP_TABLE_CHUCK_FRONT_PLANE[linearIndex][angularIndex][0][1];
    right.x = LOOKUP_TABLE_CHUCK_FRONT_PLANE[linearIndex][angularIndex][1][0];
    right.y = LOOKUP_TABLE_CHUCK_FRONT_PLANE[linearIndex][angularIndex][1][1];
  }
  else
  {
    return true; //<  Not valid for this version, so don't run but let the other limiters run
  }

  if (flipAngular)
  {
    left.x *= -1.0;
    right.x *= -1.0;
    geometry_msgs::Point temp = left;
    left = right;
    right = temp;
  }

  double distance_from_static_left = test_data_->distance_from_static_left;
  double distance_from_static_right = test_data_->distance_from_static_right;

  if (currLinearVel >= params_.min_linear_velocity_test_speed &&
      currLinearVel <= params_.max_linear_velocity_test_speed)
  {
    SpeedLimiterResult result =
        calculateAllowedLinearSpeed(distance_from_static_left, distance_from_static_right, max_allowed_linear_vel);
    if (result.limiting)
    {
      if (result.speed < currLinearVel)
      {
        max_allowed_linear_vel = result.speed;
      }
    }
  }

  if (currAngularVel >= params_.min_angular_velocity_test_speed &&
      currAngularVel <= params_.max_angular_velocity_test_speed)
  {
    SpeedLimiterResult result =
        calculateAllowedAngularSpeed(distance_from_static_left, distance_from_static_right, max_allowed_angular_vel);
    if (result.limiting)
    {
      if (result.speed < currAngularVel)
      {
        max_allowed_angular_vel = result.speed;
      }
    }
  }

  return true;
}


/**
 * Check if the no ops work for the enabled flag and the min max speeds
 * Test 1: Invalid firmware on CHK5 and CHK5+
 * Test 2: The Chuck has the wrong chassis model
 * Test 3: Static Obstacles are too far from the CHK5 and CHK5+
 * Test 4: The CHK5 and CHK5+ are moving too slow to run the test
 * Test 5: The CHK5 and CHK5+ are moving too quickly to run the test
 */
TEST(StaticObjectSpeedLimiter, no_op_testing) {
  base_local_planner::StaticObjectSpeedLimiterTestHarness testHarness;
  ros::Time::init();

  double max_allowed_linear_vel;
  double max_allowed_angular_vel;
  testHarness.getLimits_Test(max_allowed_linear_vel, max_allowed_angular_vel);

  double max_allowed_linear_vel_result = max_allowed_linear_vel;
  double max_allowed_angular_vel_result = max_allowed_angular_vel;
  StaticObjectSpeedLimiter_TEST::SpeedLimiterTestData data;

  // Not firmware capable
  data.enabledfirmwareVersion = false;
  data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5;
  data.distance_from_static_left = 0.5;
  data.distance_from_static_right = 0.5;
  data.velocity.linear = 1.0;
  data.velocity.angular = 0.5;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_EQ(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_EQ(max_allowed_angular_vel_result, max_allowed_angular_vel_result);

  data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5_PLUS;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_EQ(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_EQ(max_allowed_angular_vel_result, max_allowed_angular_vel_result);

  // Invalid chassis
  data.enabledfirmwareVersion = true;
  data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK4;
  data.distance_from_static_left = 0.5;
  data.distance_from_static_right = 0.5;
  data.velocity.linear = 1.0;
  data.velocity.angular = 0.5;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_EQ(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_EQ(max_allowed_angular_vel_result, max_allowed_angular_vel_result);

  // Too Far
  data.enabledfirmwareVersion = true;
  data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5;
  data.distance_from_static_left = 10.0;
  data.distance_from_static_right = 10.0;
  data.velocity.linear = 1.0;
  data.velocity.angular = 0.5;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_EQ(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_EQ(max_allowed_angular_vel_result, max_allowed_angular_vel_result);

  data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5_PLUS;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_EQ(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_EQ(max_allowed_angular_vel_result, max_allowed_angular_vel_result);

  // Too Slow
  data.enabledfirmwareVersion = true;
  data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5;
  data.distance_from_static_left = 0.5;
  data.distance_from_static_right = 0.5;
  data.velocity.linear = 0.1;
  data.velocity.angular = 0.5;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_EQ(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_EQ(max_allowed_angular_vel_result, max_allowed_angular_vel_result);

  data.velocity.linear = 1.0;
  data.velocity.angular = 0.05;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_EQ(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_EQ(max_allowed_angular_vel_result, max_allowed_angular_vel_result);

  data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5_PLUS;

  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_EQ(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_EQ(max_allowed_angular_vel_result, max_allowed_angular_vel_result);

  data.velocity.linear = 0.1;
  data.velocity.angular = 0.5;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_EQ(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_EQ(max_allowed_angular_vel_result, max_allowed_angular_vel_result);

  // Too Fast
  data.enabledfirmwareVersion = true;
  data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5;
  data.distance_from_static_left = 0.5;
  data.distance_from_static_right = 0.5;
  data.velocity.linear = 1.4;
  data.velocity.angular = 0.5;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_EQ(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_EQ(max_allowed_angular_vel_result, max_allowed_angular_vel_result);

  data.velocity.linear = 1.0;
  data.velocity.angular = 1.0;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_EQ(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_EQ(max_allowed_angular_vel_result, max_allowed_angular_vel_result);

  data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5_PLUS;

  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_EQ(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_EQ(max_allowed_angular_vel_result, max_allowed_angular_vel_result);

  data.velocity.linear = 1.4;
  data.velocity.angular = 0.5;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_EQ(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_EQ(max_allowed_angular_vel_result, max_allowed_angular_vel_result);
}

/**
 * Test to see if the speed of the Chuck is reduced under optimal conditions
 * for both the CHK5 and CHK5+
 */
TEST(StaticObjectSpeedLimiter, limiter_in_action) {
  base_local_planner::StaticObjectSpeedLimiterTestHarness testHarness;
  ros::Time::init();

  double max_allowed_linear_vel;
  double max_allowed_angular_vel;
  testHarness.getLimits_Test(max_allowed_linear_vel, max_allowed_angular_vel);

  double max_allowed_linear_vel_result = max_allowed_linear_vel;
  double max_allowed_angular_vel_result = max_allowed_angular_vel;

  StaticObjectSpeedLimiter_TEST::SpeedLimiterTestData data;
  data.enabledfirmwareVersion = true;
  data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5;
  data.distance_from_static_left = 0.5;
  data.distance_from_static_right = 0.5;
  data.velocity.linear = 1.34;
  data.velocity.angular = 0.79;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_NE(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_NE(max_allowed_angular_vel, max_allowed_angular_vel_result);

  max_allowed_linear_vel_result = max_allowed_linear_vel;
  max_allowed_angular_vel_result = max_allowed_angular_vel;

  data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5_PLUS;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_NE(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_NE(max_allowed_angular_vel, max_allowed_angular_vel_result);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

}  // namespace base_local_planner
