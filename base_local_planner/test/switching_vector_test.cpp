/* Author: Hayato Terao
 * Date Created: December 21st, 2020
 * Last Modified: December 25th, 2020
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
    // double result = tpr.computeSwitchingVectorAngle(goal_x, goal_y, goal_yaw, robot_x, robot_y, constant_vector_length, &agv_x, &agv_y, &total_dist);
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped robot;
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 3.0;
    robot.pose.position.x = 2.0;
    robot.pose.position.y = 4.3;
    const double constant_vector_length = 2.0;

    const double goal_yaw = PI / 4.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw);
    q.normalize();
    goal.pose.orientation.x = q[0];
    goal.pose.orientation.y = q[1];
    goal.pose.orientation.z = q[2];
    goal.pose.orientation.w = q[3];

    //expect ~= 15.4 degrees = 0.2687 radians from manual calculation
    double expect = atan(abs(goal.pose.position.y + sin(goal_yaw) * constant_vector_length - robot.pose.position.y) / abs(goal.pose.position.x + cos(goal_yaw) * constant_vector_length - robot.pose.position.x));
    double result = tpr.computeSwitchingVectorAngle(goal, robot, constant_vector_length);
    EXPECT_NEAR(expect, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, secondQuadrant){
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped robot;
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 3.0;
    robot.pose.position.x = 5.6;
    robot.pose.position.y = 3.9;
    const double constant_vector_length = 2.0;

    const double goal_yaw = PI / 4.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw);
    q.normalize();
    goal.pose.orientation.x = q[0];
    goal.pose.orientation.y = q[1];
    goal.pose.orientation.z = q[2];
    goal.pose.orientation.w = q[3];

    //expect ~= 170.84 degrees = 2.982 radians from manual calculation
    double expect = PI - atan(abs(goal.pose.position.y + sin(goal_yaw) * constant_vector_length - robot.pose.position.y) / abs(goal.pose.position.x + cos(goal_yaw) * constant_vector_length - robot.pose.position.x));
    double result = tpr.computeSwitchingVectorAngle(goal, robot, constant_vector_length);
    EXPECT_NEAR(expect, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, thirdQuadrant){
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped robot;
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 3.0;
    robot.pose.position.x = 4.5;
    robot.pose.position.y = 6.7;
    const double constant_vector_length = 2.0;

    const double goal_yaw = PI / 4.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw);
    q.normalize();
    goal.pose.orientation.x = q[0];
    goal.pose.orientation.y = q[1];
    goal.pose.orientation.z = q[2];
    goal.pose.orientation.w = q[3];

    //expect ~= 227.62 degrees = 3.973 radians from manual calculation
    double expect = PI + atan(abs(goal.pose.position.y + sin(goal_yaw) * constant_vector_length - robot.pose.position.y) / abs(goal.pose.position.x + cos(goal_yaw) * constant_vector_length - robot.pose.position.x));
    double result = tpr.computeSwitchingVectorAngle(goal, robot, constant_vector_length);
    EXPECT_NEAR(expect, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, fourthQuadrant){
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped robot;
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 3.0;
    robot.pose.position.x = 0.98;
    robot.pose.position.y = 5.2;
    const double constant_vector_length = 2.0;

    const double goal_yaw = PI / 4.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw);
    q.normalize();
    goal.pose.orientation.x = q[0];
    goal.pose.orientation.y = q[1];
    goal.pose.orientation.z = q[2];
    goal.pose.orientation.w = q[3];

    //expect ~= 331.27 degrees = 5.782 radians from manual calculation
    double expect = 2*PI - atan(abs(goal.pose.position.y + sin(goal_yaw) * constant_vector_length - robot.pose.position.y) / abs(goal.pose.position.x + cos(goal_yaw) * constant_vector_length - robot.pose.position.x));
    double result = tpr.computeSwitchingVectorAngle(goal, robot, constant_vector_length);
    EXPECT_NEAR(expect, result, 0.001);
}

// TODO: Subject to change. Need to decide what to do for this case.
TEST_F(SwitchingVectorAngleTest, RobotOnAgvLine){
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped robot;
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 3.0;
    robot.pose.position.x = 2.41421;
    robot.pose.position.y = 4.41421;
    const double constant_vector_length = 2.0;

    const double goal_yaw = PI / 4.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw);
    q.normalize();
    goal.pose.orientation.x = q[0];
    goal.pose.orientation.y = q[1];
    goal.pose.orientation.z = q[2];
    goal.pose.orientation.w = q[3];

    double result = tpr.computeSwitchingVectorAngle(goal, robot, constant_vector_length);
    EXPECT_NEAR(goal_yaw, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, RobotAtDesignatedGoal){
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped robot;
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 3.0;
    robot.pose.position.x = goal.pose.position.x;
    robot.pose.position.y = goal.pose.position.y;
    const double constant_vector_length = 2.0;

    const double goal_yaw = PI / 4.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw);
    q.normalize();
    goal.pose.orientation.x = q[0];
    goal.pose.orientation.y = q[1];
    goal.pose.orientation.z = q[2];
    goal.pose.orientation.w = q[3];

    double result = tpr.computeSwitchingVectorAngle(goal, robot, constant_vector_length);
    EXPECT_NEAR(goal_yaw, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, RobotDisplacedYdirectionPositive){
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped robot;
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 3.0;
    const double goal_yaw = PI / 4.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw);
    q.normalize();
    goal.pose.orientation.x = q[0];
    goal.pose.orientation.y = q[1];
    goal.pose.orientation.z = q[2];
    goal.pose.orientation.w = q[3];
    const double constant_vector_length = 2.0;
    robot.pose.position.x = goal.pose.position.x + cos(goal_yaw) * constant_vector_length;
    robot.pose.position.y = 8.82842;

    double result = tpr.computeSwitchingVectorAngle(goal, robot, constant_vector_length);
    EXPECT_NEAR(2 * PI -PI / 2.0, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, RobotDisplacedYdirectionNegative){
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped robot;
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 3.0;
    const double goal_yaw = PI / 4.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw);
    q.normalize();
    goal.pose.orientation.x = q[0];
    goal.pose.orientation.y = q[1];
    goal.pose.orientation.z = q[2];
    goal.pose.orientation.w = q[3];
    const double constant_vector_length = 2.0;
    robot.pose.position.x = goal.pose.position.x + cos(goal_yaw) * constant_vector_length;
    robot.pose.position.y = -8.82842;

    double result = tpr.computeSwitchingVectorAngle(goal, robot, constant_vector_length);
    EXPECT_NEAR(PI / 2.0, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, RobotDisplacedXdirectionPositive){
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped robot;
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 3.0;
    const double goal_yaw = PI / 4.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw);
    q.normalize();
    goal.pose.orientation.x = q[0];
    goal.pose.orientation.y = q[1];
    goal.pose.orientation.z = q[2];
    goal.pose.orientation.w = q[3];
    const double constant_vector_length = 2.0;
    robot.pose.position.x = 4.82842;
    robot.pose.position.y = goal.pose.position.y + sin(goal_yaw) * constant_vector_length;

    double result = tpr.computeSwitchingVectorAngle(goal, robot, constant_vector_length);
    EXPECT_NEAR(PI, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, RobotDisplacedXdirectionNegative){
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped robot;
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 3.0;
    const double goal_yaw = PI / 4.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw);
    q.normalize();
    goal.pose.orientation.x = q[0];
    goal.pose.orientation.y = q[1];
    goal.pose.orientation.z = q[2];
    goal.pose.orientation.w = q[3];
    const double constant_vector_length = 2.0;
    robot.pose.position.x = -4.82842;
    robot.pose.position.y = goal.pose.position.y + sin(goal_yaw) * constant_vector_length;

    double result = tpr.computeSwitchingVectorAngle(goal, robot, constant_vector_length);
    EXPECT_NEAR(0.0, result, 0.001);
}

//TODO
TEST_F(SwitchingVectorAngleTest, shorterConstantVectorAtThirdQuadrant){
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped robot;
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 3.0;
    robot.pose.position.x = 2.0;
    robot.pose.position.y = 4.3;
    const double constant_vector_length = 0.3;

    const double goal_yaw = PI / 4.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw);
    q.normalize();
    goal.pose.orientation.x = q[0];
    goal.pose.orientation.y = q[1];
    goal.pose.orientation.z = q[2];
    goal.pose.orientation.w = q[3];


    // agv ~= (1.212, 3.212) --> third quadrant
    //expect ~= 234.1 degrees = 4.086 radians from manual calculation
    double expect = PI + atan(abs(goal.pose.position.y + sin(goal_yaw) * constant_vector_length - robot.pose.position.y) / abs(goal.pose.position.x + cos(goal_yaw) * constant_vector_length - robot.pose.position.x));
    double result = tpr.computeSwitchingVectorAngle(goal, robot, constant_vector_length);
    EXPECT_NEAR(expect, result, 0.001);
}

TEST_F(SwitchingVectorAngleTest, longerConstantVectorAtFirstQuadrant){
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped robot;
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 3.0;
    robot.pose.position.x = 2.0;
    robot.pose.position.y = 4.3;
    const double constant_vector_length = 4.5;

    const double goal_yaw = PI / 4.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw);
    q.normalize();
    goal.pose.orientation.x = q[0];
    goal.pose.orientation.y = q[1];
    goal.pose.orientation.z = q[2];
    goal.pose.orientation.w = q[3];

    //expect ~= 15.4 degrees = 0.2687 radians from manual calculation
    double expect = atan(abs(goal.pose.position.y + sin(goal_yaw) * constant_vector_length - robot.pose.position.y) / abs(goal.pose.position.x + cos(goal_yaw) * constant_vector_length - robot.pose.position.x));
    double result = tpr.computeSwitchingVectorAngle(goal, robot, constant_vector_length);
    EXPECT_NEAR(expect, result, 0.001);
}
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "switching_vector_test");
  return RUN_ALL_TESTS();
}