/* Author: Hayato Terao
 * Date Created: December 21st, 2020
 * Last Modified: December 21st, 2020
 * Purpose: Unit tests for verifying that the switching vector is computed properly
*/

#include <gtest/gtest.h>
#include <math.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>

#define PI acos(-1)

namespace base_local_planner {

class SwitchingVectorAngleTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      geometry_msgs::TransformStamped base_rel_map;
      base_rel_map.transform = tf2::toMsg(tf2::Transform::getIdentity());
      base_rel_map.child_frame_id = "base_link";
      base_rel_map.header.frame_id = "map";
      base_rel_map.header.stamp = ros::Time::now();

      tf2_ros::Buffer* tf_;
      tf_ = new tf2_ros::Buffer( ros::Duration( 10 ));
      tf_->setTransform( base_rel_map, "switching_vector_test" );
      tf_->setUsingDedicatedThread(true);

      tf2_ros::Buffer tf;
      costmap_2d::Costmap2DROS map("testmap", *tf_);
      tpr.initialize("test_planner", &tf, &map);
    }
    
    TrajectoryPlannerROS tpr;
};

TEST_F(SwitchingVectorAngleTest, firstQuadrant){
    const double goal_x = 1.0;
    const double goal_y = 3.0;
    const double goal_yaw = PI / 4.0;
    const double constant_vector_length = 2.0;
    const double robot_x = 2.0;
    const double robot_y = 4.3;
    double agv_x, agv_y, total_dist;

    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    //expect ~= 15.4 degrees = 0.2687 radians from manual calculation
    double expect = atan(abs(goal_y + sin(goal_yaw) * constant_vector_length - robot_y) / abs(goal_x + cos(goal_yaw) * constant_vector_length - robot_x));
    EXPECT_NEAR(expect, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, secondQuadrant){
    const double goal_x = 1.0;
    const double goal_y = 3.0;
    const double goal_yaw = PI / 4.0;
    const double constant_vector_length = 2.0;
    const double robot_x = 5.6;
    const double robot_y = 3.9;
    double agv_x, agv_y, total_dist;

    //expect ~= 170.84 degrees = 2.982 radians from manual calculation
    double expect = PI - atan(abs(goal_y + sin(goal_yaw) * constant_vector_length - robot_y) / abs(goal_x + cos(goal_yaw) * constant_vector_length - robot_x));
    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_NEAR(expect, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, thirdQuadrant){
    const double goal_x = 1.0;
    const double goal_y = 3.0;
    const double goal_yaw = PI / 4.0;
    const double constant_vector_length = 2.0;
    const double robot_x = 4.5;
    const double robot_y = 6.7;
    double agv_x, agv_y, total_dist;

    //expect ~= 227.62 degrees = 3.973 radians from manual calculation
    double expect = PI + atan(abs(goal_y + sin(goal_yaw) * constant_vector_length - robot_y) / abs(goal_x + cos(goal_yaw) * constant_vector_length - robot_x));
    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_NEAR(expect, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, fourthQuadrant){
    const double goal_x = 1.0;
    const double goal_y = 3.0;
    const double goal_yaw = PI / 4.0;
    const double constant_vector_length = 2.0;
    const double robot_x = 0.98;
    const double robot_y = 5.2;
    double agv_x, agv_y, total_dist;

    //expect ~= 331.27 degrees = 5.782 radians from manual calculation
    double expect = 2*PI - atan(abs(goal_y + sin(goal_yaw) * constant_vector_length - robot_y) / abs(goal_x + cos(goal_yaw) * constant_vector_length - robot_x));
    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_NEAR(expect, result, 0.001);
}

// TODO: Subject to change. Need to decide what to do for this case.
TEST_F(SwitchingVectorAngleTest, RobotOnAgvLine){
    const double goal_x = 1.0;
    const double goal_y = 3.0;
    const double goal_yaw = PI / 4.0;
    const double constant_vector_length = 2.0;
    const double robot_x = 2.41421;
    const double robot_y = 4.41421;
    double agv_x, agv_y, total_dist;

    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_NEAR(goal_yaw, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, RobotAtDesignatedGoal){
    const double goal_x = 1.0;
    const double goal_y = 3.0;
    const double goal_yaw = PI / 4.0;
    const double constant_vector_length = 2.0;
    const double robot_x = goal_x;
    const double robot_y = goal_y;
    double agv_x, agv_y, total_dist;

    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_NEAR(goal_yaw, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, RobotDisplacedYdirectionPositive){
    const double goal_x = 1.0;
    const double goal_y = 3.0;
    const double goal_yaw = PI / 4.0;
    const double constant_vector_length = 2.0;
    const double robot_x = goal_x + cos(goal_yaw) * constant_vector_length;
    const double robot_y = 8.82842;
    double agv_x, agv_y, total_dist;

    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_NEAR(-PI / 2.0, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, RobotDisplacedYdirectionNegative){
    const double goal_x = 1.0;
    const double goal_y = 3.0;
    const double goal_yaw = PI / 4.0;
    const double constant_vector_length = 2.0;
    const double robot_x = goal_x + cos(goal_yaw) * constant_vector_length;
    const double robot_y = -8.82842;
    double agv_x, agv_y, total_dist;

    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_NEAR(PI / 2.0, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, RobotDisplacedXdirectionPositive){
    const double goal_x = 1.0;
    const double goal_y = 3.0;
    const double goal_yaw = PI / 4.0;
    const double constant_vector_length = 2.0;
    const double robot_x = 4.82842;
    const double robot_y = goal_y + sin(goal_yaw) * constant_vector_length;
    double agv_x, agv_y, total_dist;

    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_NEAR(-PI, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, RobotDisplacedXdirectionNegative){
    const double goal_x = 1.0;
    const double goal_y = 3.0;
    const double goal_yaw = PI / 4.0;
    const double constant_vector_length = 2.0;
    const double robot_x = -4.82842;
    const double robot_y = goal_y + sin(goal_yaw) * constant_vector_length;
    double agv_x, agv_y, total_dist;

    double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    EXPECT_NEAR(0.0, result, 0.001);
}
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "switching_vector_test");
  return RUN_ALL_TESTS();
}
