/* Author: Hayato Terao
 * Date Created: December 21st, 2020
 * Last Modified: December 21st, 2020
 * Purpose: Unit tests for verifying that the switching vector is computed properly
*/

#include <gtest/gtest.h>
#include <math.h>
#include <base_local_planner/trajectory_planner_ros.h>

#define PI acos(-1)

namespace base_local_planner {
TEST(SwitchingVectorAngleTest, differentQuadrants){
    TrajectoryPlannerROS tpr;
    double goal_x = 1.0;
    double goal_y = 3.0;
    double goal_yaw = PI / 4.0;
    double constant_vector_length = 2.0;
    double agv_x, agv_y, total_dist;

    // Basic Cases
    // First Quadrant
    double robot_x = 2.0;
    double robot_y = 4.3;
    //expect ~= 15.4 degrees = 0.2687 radians from manual calculation
    double expect = atan(abs(goal_y + sin(goal_yaw) * constant_vector_length - robot_y) / abs(goal_x + cos(goal_yaw) * constant_vector_length - robot_x));
    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_EQ(expect, result);

    // Second Quadrant
    robot_x = 5.6;
    robot_y = 3.9;
    //expect ~= 170.84 degrees = 2.982 radians from manual calculation
    expect = PI - atan(abs(goal_y + sin(goal_yaw) * constant_vector_length - robot_y) / abs(goal_x + cos(goal_yaw) * constant_vector_length - robot_x));
    result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_EQ(expect, result);

    // Third Quadrant
    robot_x = 4.5;
    robot_y = 6.7;
    //expect ~= 227.62 degrees = 3.973 radians from manual calculation
    expect = PI + atan(abs(goal_y + sin(goal_yaw) * constant_vector_length - robot_y) / abs(goal_x + cos(goal_yaw) * constant_vector_length - robot_x));
    result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_EQ(expect, result);

    // Fourth Quadrant
    robot_x = 0.98;
    robot_y = 5.2;
    //expect ~= 331.27 degrees = 5.782 radians from manual calculation
    expect = 2*PI - atan(abs(goal_y + sin(goal_yaw) * constant_vector_length - robot_y) / abs(goal_x + cos(goal_yaw) * constant_vector_length - robot_x));
    result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_EQ(expect, result);


    // Special Cases
    // Robot is on the AGV line
    robot_x = agv_x;
    robot_y = agv_y;
    expect = goal_yaw; //TODO: Subject to change. Need to decide what to do for this case.
    result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_EQ(expect, result);

    // Robot is at the designated goal
    robot_x = goal_x;
    robot_y = goal_y;
    expect = goal_yaw;
    result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_EQ(expect, result);

    // Robot is displaced from the AGV line only in the y-direction (positive)
    robot_x = agv_x;
    robot_y = 2 * agv_y;
    expect = -PI / 2.0;
    result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_EQ(expect, result);

    // Robot is displaced from the AGV line only in the y-direction (negative)
    robot_x = agv_x;
    robot_y = -2 * agv_y;
    expect = PI / 2.0;
    result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_EQ(expect, result);

    // Robot is displaced from the AGV line only in the x-direction (positive)
    robot_x = 2 * agv_x;
    robot_y = agv_y;
    expect = -PI;
    result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_EQ(expect, result);

    // Robot is displaced from the AGV line only in the x-direction (negative)
    robot_x = -2 * agv_x;
    robot_y = agv_y;
    expect = 0.0;
    result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_EQ(expect, result);
}

/*
TEST(SwitchingVectorAngleTest, firstQuadrant){
    TrajectoryPlannerROS tpr;
    const double goal_x = 1.0;
    const double goal_y = 3.0;
    const double goal_yaw = PI / 4.0;
    const double robot_x = 2.0;
    const double robot_y = 4.3;
    const double constant_vector_length = 2.0;
    double agv_x, agv_y, total_dist;

    //expect ~= 15.4 degrees = 0.2687 radians from manual calculation
    double expect = atan(abs(goal_y + sin(goal_yaw) * constant_vector_length - robot_y) / abs(goal_x + cos(goal_yaw) * constant_vector_length - robot_x));

    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_EQ(expect, result);
}

TEST(SwitchingVectorAngleTest, secondQuadrant){
    TrajectoryPlannerROS tpr;
    const double goal_x = 1.0;
    const double goal_y = 3.0;
    const double goal_yaw = PI / 4.0;
    const double robot_x = 5.6;
    const double robot_y = 3.9;
    const double constant_vector_length = 2.0;
    double agv_x, agv_y, total_dist;

    //expect ~= 170.84 degrees = 2.982 radians from manual calculation
    double expect = PI - atan(abs(goal_y + sin(goal_yaw) * constant_vector_length - robot_y) / abs(goal_x + cos(goal_yaw) * constant_vector_length - robot_x));

    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_EQ(expect, result);
}

TEST(SwitchingVectorAngleTest, thirdQuadrant){
    TrajectoryPlannerROS tpr;
    const double goal_x = 1.0;
    const double goal_y = 3.0;
    const double goal_yaw = PI / 4.0;
    const double robot_x = 4.5;
    const double robot_y = 6.7;
    const double constant_vector_length = 2.0;
    double agv_x, agv_y, total_dist;

    //expect ~= 227.62 degrees = 3.973 radians from manual calculation
    double expect = PI + atan(abs(goal_y + sin(goal_yaw) * constant_vector_length - robot_y) / abs(goal_x + cos(goal_yaw) * constant_vector_length - robot_x));

    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_EQ(expect, result);
}

TEST(SwitchingVectorAngleTest, fourthQuadrant){
    TrajectoryPlannerROS tpr;
    const double goal_x = 1.0;
    const double goal_y = 3.0;
    const double goal_yaw = PI / 4.0;
    const double robot_x = 0.98;
    const double robot_y = 5.2;
    const double constant_vector_length = 2.0;
    double agv_x, agv_y, total_dist;

    //expect ~= 331.27 degrees = 5.782 radians from manual calculation
    double expect = 2*PI - atan(abs(goal_y + sin(goal_yaw) * constant_vector_length - robot_y) / abs(goal_x + cos(goal_yaw) * constant_vector_length - robot_x));

    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_EQ(expect, result);
}
*/
}
