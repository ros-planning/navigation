/*
 * follower_trajectory_test.cpp
 *
 *      Author: dgrieneisen
 */
#include <gtest/gtest.h>

#include <base_local_planner/geometry_math_helpers.h>
#include <base_local_planner/local_planner_limits.h>

#include "critic_test_helpers.h"

namespace base_local_planner {


TEST(GeometryMathHelpers, distance_to_line_segment){
  // Test distance to line segments
  // First, test degenerate line segment
  EXPECT_EQ(1.0, distanceToLineSegment(
                                      create2DVector(1.0, 0.0),
                                      create2DVector(0.0, 0.0),
                                      create2DVector(0.0, 0.0)));
  // Point on line end
  EXPECT_EQ(0.0, distanceToLineSegment(
                                      create2DVector(1.0, 0.0),
                                      create2DVector(0.0, 0.0),
                                      create2DVector(1.0, 0.0)));
  EXPECT_EQ(0.0, distanceToLineSegment(
                                      create2DVector(1.0, 0.0),
                                      create2DVector(1.0, 0.0),
                                      create2DVector(0.0, 0.0)));
  // Point relative to line off axis
  EXPECT_EQ(1.0, distanceToLineSegment(
                                      create2DVector(1.0, 0.0),
                                      create2DVector(0.0, 0.0),
                                      create2DVector(0.0, 1.0)));
  EXPECT_EQ(1.0, distanceToLineSegment(
                                      create2DVector(1.0, 0.0),
                                      create2DVector(0.0, 1.0),
                                      create2DVector(0.0, 0.0)));
  EXPECT_EQ(1.0, distanceToLineSegment(
                                      create2DVector(1.0, 0.5),
                                      create2DVector(0.0, 0.0),
                                      create2DVector(0.0, 1.0)));
  // Point relative to line on axis
  EXPECT_EQ(0.0, distanceToLineSegment(
                                      create2DVector(0.0, 1.0),
                                      create2DVector(0.0, 0.0),
                                      create2DVector(0.0, 1.0)));
  EXPECT_EQ(0.0, distanceToLineSegment(
                                      create2DVector(0.0, 1.0),
                                      create2DVector(0.0, 1.0),
                                      create2DVector(0.0, 0.0)));
  EXPECT_EQ(0.0, distanceToLineSegment(
                                      create2DVector(0.0, 0.5),
                                      create2DVector(0.0, 0.0),
                                      create2DVector(0.0, 1.0)));

  EXPECT_EQ(1.5, distanceToLineSegment(
                                      create2DVector(0.0, 2.5),
                                      create2DVector(0.0, 0.0),
                                      create2DVector(0.0, 1.0)));

  // off axis line
  EXPECT_TRUE(0.0001 > std::fabs(0.707107 - distanceToLineSegment(
                                              create2DVector(0.0, 1.0),
                                              create2DVector(0.0, 0.0),
                                              create2DVector(1.0, 1.0))));
}

TEST(GeometryMathHelpers, distance_along_line_segment){
  // Test distance to line segments
  // First, test degenerate line segment
  EXPECT_EQ(0.0, distanceAlongLineSegment(
                                          create2DVector(1.0, 0.0),
                                          create2DVector(0.0, 0.0),
                                          create2DVector(0.0, 0.0)));

  // Next, at end points
  EXPECT_EQ(0.0, distanceAlongLineSegment(
                                          create2DVector(1.0, 0.0),
                                          create2DVector(0.0, 0.0),
                                          create2DVector(0.0, 1.0)));
  EXPECT_EQ(1.0, distanceAlongLineSegment(
                                          create2DVector(1.0, 0.0),
                                          create2DVector(0.0, 0.0),
                                          create2DVector(1.0, 0.0)));
  EXPECT_EQ(0.0, distanceAlongLineSegment(
                                          create2DVector(1.0, 0.0),
                                          create2DVector(1.0, 0.0),
                                          create2DVector(0.0, 0.0)));
  EXPECT_EQ(0.5, distanceAlongLineSegment(
                                          create2DVector(0.5, 1.0),
                                          create2DVector(0.0, 0.0),
                                          create2DVector(1.0, 0.0)));

}

TEST(GeometryMathHelpers, pose_at_distance_along_line_segment){
  // Test distance to line segments
  // First, test degenerate line segment
  EXPECT_TRUE(vector2DEqual(create2DVector(0.0, 0.0),
    poseAtDistanceAlongLineSegment(
                                  0.0,
                                  create2DVector(0.0, 0.0),
                                  create2DVector(0.0, 0.0))));

  EXPECT_TRUE(vector2DEqual(create2DVector(0.0, 0.0),
    poseAtDistanceAlongLineSegment(
                                  0.0,
                                  create2DVector(0.0, 0.0),
                                  create2DVector(1.0, 0.0))));

  EXPECT_TRUE(vector2DEqual(create2DVector(0.5, 0.0),
    poseAtDistanceAlongLineSegment(
                                  0.5,
                                  create2DVector(0.0, 0.0),
                                  create2DVector(1.0, 0.0))));
}

}
