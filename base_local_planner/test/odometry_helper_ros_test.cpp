/*
 * follower_trajectory_test.cpp
 *
 *      Author: dgrieneisen
 */
#include <gtest/gtest.h>

#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/local_planner_limits.h>

#include "critic_test_helpers.h"

namespace base_local_planner {
const double WHEELBASE = 0.5;
const double EPSILON = 0.00001;

TEST(OdometryHelperRosTest, wheel_speeds){
  // Test wheel speed conversions
  // Converting back and forth should be correct.

  // Start with both wheels forward
  double left_s, left_f, right_s, right_f;
  left_s = 1.0;
  right_s = 1.0;
  Eigen::Vector3f vel = OdometryHelperRos::getVelFromWheelGroundSpeeds(WHEELBASE, left_s, right_s);
  OdometryHelperRos::getWheelGroundSpeedsFromVel(vel, WHEELBASE, left_f, right_f);
  EXPECT_NEAR(left_s, left_f, EPSILON);
  EXPECT_NEAR(right_s, right_f, EPSILON);

  // Other speed combos
  left_s = 1.0;
  right_s = -1.0;
  vel = OdometryHelperRos::getVelFromWheelGroundSpeeds(WHEELBASE, left_s, right_s);
  OdometryHelperRos::getWheelGroundSpeedsFromVel(vel, WHEELBASE, left_f, right_f);
  std::cerr << "S: " << left_s << ", " << right_s << "; F: " << left_f << ", " << right_f
    << ".  Vel: " << vel[0] << ", " << vel[1] << ", " << vel[2] << std::endl;
  EXPECT_NEAR(left_s, left_f, EPSILON);
  EXPECT_NEAR(right_s, right_f, EPSILON);


  left_s = 0.0;
  right_s = 1.0;
  vel = OdometryHelperRos::getVelFromWheelGroundSpeeds(WHEELBASE, left_s, right_s);
  OdometryHelperRos::getWheelGroundSpeedsFromVel(vel, WHEELBASE, left_f, right_f);
  EXPECT_NEAR(left_s, left_f, EPSILON);
  EXPECT_NEAR(right_s, right_f, EPSILON);


  left_s = -1.0;
  right_s = -1.0;
  vel = OdometryHelperRos::getVelFromWheelGroundSpeeds(WHEELBASE, left_s, right_s);
  OdometryHelperRos::getWheelGroundSpeedsFromVel(vel, WHEELBASE, left_f, right_f);
  EXPECT_NEAR(left_s, left_f, EPSILON);
  EXPECT_NEAR(right_s, right_f, EPSILON);


  left_s = 1.0;
  right_s = 2.0;
  vel = OdometryHelperRos::getVelFromWheelGroundSpeeds(WHEELBASE, left_s, right_s);
  OdometryHelperRos::getWheelGroundSpeedsFromVel(vel, WHEELBASE, left_f, right_f);
  EXPECT_NEAR(left_s, left_f, EPSILON);
  EXPECT_NEAR(right_s, right_f, EPSILON);

}

TEST(OdometryHelperRosTest, velocity_iteration)
{
  // Create data structures for test.
  Eigen::Vector3f cmd = Eigen::Vector3f::Zero();
  Eigen::Vector3f vel = Eigen::Vector3f::Zero();
  Eigen::Vector3f res = Eigen::Vector3f::Zero();

  double wheel_limits = 0.7;
  double dt = 0.1;

  // Test 0 to 0
  EXPECT_TRUE(vector3fEqual(createVector(0.0, 0.0, 0.0),  // result
    OdometryHelperRos::computeNewVelocities(
      createVector(0.0, 0.0, 0.0), // desired
      createVector(0.0, 0.0, 0.0), // current
      wheel_limits, WHEELBASE, dt)));

  // TIP small
  EXPECT_TRUE(vector3fEqual(createVector(0.0, 0.0, 0.05),  // result
    OdometryHelperRos::computeNewVelocities(
      createVector(0.0, 0.0, 0.05), // desired
      createVector(0.0, 0.0, 0.0), // current
      wheel_limits, WHEELBASE, dt)));

  // Forwards small
  EXPECT_TRUE(vector3fEqual(createVector(0.05, 0.0, 0.0),  // result
    OdometryHelperRos::computeNewVelocities(
      createVector(0.05, 0.0, 0.0), // desired
      createVector(0.0, 0.0, 0.0), // current
      wheel_limits, WHEELBASE, dt)));

  // TIP big
  EXPECT_TRUE(vector3fEqual(createVector(0.0, 0.0, 0.28),  // result
    OdometryHelperRos::computeNewVelocities(
      createVector(0.0, 0.0, 1.00), // desired
      createVector(0.0, 0.0, 0.0), // current
      wheel_limits, WHEELBASE, dt)));

  // Forwards big
  EXPECT_TRUE(vector3fEqual(createVector(0.07, 0.0, 0.0),  // result
    OdometryHelperRos::computeNewVelocities(
      createVector(1.0, 0.0, 0.0), // desired
      createVector(0.0, 0.0, 0.0), // current
      wheel_limits, WHEELBASE, dt)));
}

TEST(OdometryHelperRosTest, pose_update)
{
  OdometryHelperRos* ohr = new OdometryHelperRos("");
  double dt = 0.1;

  // Basic test
  EXPECT_TRUE(odometryEqual(createOdometry(0, 0, 0, 0, 0), // result
    ohr->forwardEstimateOdometry(
      createTwist(0.0, 0.0), // command velocity
      createOdometry(0, 0, 0, 0, 0), // input odometry
      dt)));

  // Test with velocity
  EXPECT_TRUE(odometryEqual(createOdometry(0.0035, 0, 0, 0.07, 0), // result
    ohr->forwardEstimateOdometry(
      createTwist(0.25, 0.0), // command velocity
      createOdometry(0, 0, 0, 0, 0), // input odometry
      dt)));

  // Clean up
  delete ohr;
}


}
