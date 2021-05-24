/**
 *static_object_speed_limiter_test.cpp
 *
 *      Author: Tom Preisner
 */
#include <gtest/gtest.h>
#include <base_local_planner/speed_limiters/static_object_speed_limiter.h>

namespace base_local_planner {
class StaticObjectSpeedLimiterTestHarness {
public:
  StaticObjectSpeedLimiterTestHarness();
  ~StaticObjectSpeedLimiterTestHarness();

  void getLimits_Test(double& max_allowed_linear_vel, double& max_allowed_angular_vel);
  bool calculateLimits_Test(const StaticObjectSpeedLimiter_TEST::SpeedLimiterTestData* data,
                            double& max_allowed_linear_vel, double& max_allowed_angular_vel);

private:
  tf::TransformListener* tf_ = nullptr;
  costmap_2d::Costmap2DROS* map_ = nullptr;
  StaticObjectSpeedLimiter* speed_limiter_ = nullptr;
  StaticObjectSpeedLimiter_TEST* speed_limiter_test_ = nullptr;
};

/**
 * Constructor
 */
StaticObjectSpeedLimiterTestHarness::StaticObjectSpeedLimiterTestHarness() {
  tf_ = new tf::TransformListener(ros::Duration(10));
  map_ = new costmap_2d::Costmap2DROS("testSOSLMap", *tf_);
  speed_limiter_ = new StaticObjectSpeedLimiter(map_);

  StaticObjectSpeedLimiterConfig cfg;
  cfg.enabled = true;
  cfg.timeout = 0.0;  // Always run
  cfg.min_linear_velocity_test_speed = 0.3;
  cfg.max_linear_velocity_test_speed = 1.35;
  cfg.min_angular_velocity_test_speed = 0.1;
  cfg.max_angular_velocity_test_speed = 0.8;
  cfg.min_linear_velocity = 0.3;
  cfg.min_angular_velocity = 0.4;
  speed_limiter_->reconfigure(cfg);

  speed_limiter_test_ = new StaticObjectSpeedLimiter_TEST(speed_limiter_);
}

/**
 *Destructor
 */
StaticObjectSpeedLimiterTestHarness::~StaticObjectSpeedLimiterTestHarness() {
  delete (speed_limiter_test_);
  speed_limiter_test_ = nullptr;

  delete (speed_limiter_);
  speed_limiter_ = nullptr;

  delete (map_);
  map_ = nullptr;

  delete (tf_);
  tf_ = nullptr;
}

/**
 * Forwarding helper functions
 */
void StaticObjectSpeedLimiterTestHarness::getLimits_Test(double& max_allowed_linear_vel,
                                                         double& max_allowed_angular_vel) {
  assert(speed_limiter_test_ != nullptr);
  speed_limiter_test_->getLimits_Test(max_allowed_linear_vel, max_allowed_angular_vel);
}

bool StaticObjectSpeedLimiterTestHarness::calculateLimits_Test(const StaticObjectSpeedLimiter_TEST::SpeedLimiterTestData* data,
                                                               double& max_allowed_linear_vel,
                                                               double& max_allowed_angular_vel) {
  assert(speed_limiter_test_ != nullptr);
  return speed_limiter_test_->calculateLimits_Test(data, max_allowed_linear_vel, max_allowed_angular_vel);
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
  data.velocity.angular = 0.1;
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

  double max_allowed_linear_vel;
  double max_allowed_angular_vel;
  testHarness.getLimits_Test(max_allowed_linear_vel, max_allowed_angular_vel);

  double max_allowed_linear_vel_result = max_allowed_linear_vel;
  double max_allowed_angular_vel_result = max_allowed_angular_vel;

  StaticObjectSpeedLimiter_TEST::SpeedLimiterTestData data;
  data.enabledfirmwareVersion = false;
  data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5;
  data.distance_from_static_left = 0.5;
  data.distance_from_static_right = 0.5;
  data.velocity.linear = 1.0;
  data.velocity.angular = 0.5;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_NE(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_NE(max_allowed_angular_vel_result, max_allowed_angular_vel_result);

  max_allowed_linear_vel_result = max_allowed_linear_vel;
  max_allowed_angular_vel_result = max_allowed_angular_vel;

  data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5_PLUS;
  testHarness.calculateLimits_Test(&data, max_allowed_linear_vel_result, max_allowed_angular_vel_result);
  EXPECT_NE(max_allowed_linear_vel, max_allowed_linear_vel_result);
  EXPECT_NE(max_allowed_angular_vel_result, max_allowed_angular_vel_result);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}
