/**
 *static_object_speed_limiter_test.cpp
 *
 *      Author: Tom Preisner
 */
#include <gtest/gtest.h>
#include <base_local_planner/speed_limiters/static_object_speed_limiter.h>
#include <tf/transform_listener.h>
#include "wavefront_map_accessor.h"

namespace base_local_planner {
/**
 * setup the config for static object speed limiter(sosl)
 * and create static object speed limiter object
 * set the global plan for sosl and return it
*/
StaticObjectSpeedLimiter setupAndCreateSOSL() {
    tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS* map = new costmap_2d::Costmap2DROS("testSOSLMap", tf);
    StaticObjectSpeedLimiter static_object_speed_limiter = StaticObjectSpeedLimiter(map);
    
    StaticObjectSpeedLimiterConfig cfg;
    cfg.enabled = true;
    cfg.min_linear_velocity_test_speed = 0.3;
    cfg.max_linear_velocity_test_speed = 1.35;
    cfg.min_angular_velocity_test_speed = 0.1;
    cfg.max_angular_velocity_test_speed = 0.8;
    cfg.min_linear_velocity = 0.3;
    cfg.min_angular_velocity = 0.4;
    static_object_speed_limiter.reconfigure(cfg);
    
    return static_object_speed_limiter;
}

void cleanupSOSL(StaticObjectSpeedLimiter& sosl) {
    delete(sosl.costmap_);
    sosl.costmap_ = nullptr;
}

/**
 * Test 1: Check if the no ops work for the enabled flag and the min max speeds
 * Test 2: Test the speed reduction code for the CHK5
 * Test 3: Test the speed reduction code for the CHK5+
*/
TEST(StaticObjectSpeedLimiter, noOpTesting)
{
    auto sosl = setupAndCreateSOSL();

    StaticObjectSpeedLimiter_TEST::SpeedLimiterTestData data;
    // Not firmware capable
    data.enabledfirmwareVersion = false;
    data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5;
    data.distance_from_static_left = 0.5;
    data.distance_from_static_right = 0.5;
    data.velocity.linear = 1.0;
    data.velocity.angular = 0.5;

    data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5_PLUS;

    // Invalid chassis
    data.enabledfirmwareVersion = true;
    data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK4;
    data.distance_from_static_left = 10.0;
    data.distance_from_static_right = 10.0;
    data.velocity.linear = 1.0;
    data.velocity.angular = 0.5;

    // Too Far
    data.enabledfirmwareVersion = true;
    data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5;
    data.distance_from_static_left = 10.0;
    data.distance_from_static_right = 10.0;
    data.velocity.linear = 1.0;
    data.velocity.angular = 0.5;

    data.chassis_generation_ = srs::ChuckChassisGenerations::ChuckChassisType::CHUCK5_PLUS;

    // Too Slow

    // Too Fast

    cleanupSOSL(sosl);
}

/**
 * Test 1: If path is curved, path limiter should limit the velocity
 * by setting the linear velocity to min_linear_velocity 
 * set by PathSpeedLimiterConfig in setupAndCreatePSL function 
*/
TEST(StaticObjectSpeedLimiter, limiter_in_action)
{
    auto psl = setupAndCreatePSL();
    psl.setPlan(createCurvedPlan());
    
    double v_lim = 0, w_lim = 0;
    geometry_msgs::PoseStamped currentRobotPose;
    currentRobotPose.pose.position.x = 4;
    currentRobotPose.pose.position.y = 1;
    currentRobotPose.pose.position.z = 0;
    currentRobotPose.pose.orientation.x = 0;
    currentRobotPose.pose.orientation.y = 0;
    currentRobotPose.pose.orientation.z = 0.5;
    currentRobotPose.pose.orientation.w = 0.8660254;

    bool result = psl.calculateLimitsFromPathAndPose(v_lim, w_lim, currentRobotPose);
    EXPECT_EQ(v_lim, 0.12);
}

}
