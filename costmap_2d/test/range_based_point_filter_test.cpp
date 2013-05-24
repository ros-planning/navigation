/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <costmap_2d/range_based_point_filter.h>

using namespace costmap_2d;

geometry_msgs::Point p( double x, double y, double z )
{
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

TEST( RangeBasedPointFilter, no_points )
{
  RangeBasedPointFilter filter;
  EXPECT_FALSE( filter.checkPoint( p( 0, 0, -.01 )));
  EXPECT_TRUE( filter.checkPoint( p( 0, 0, .01 )));
  EXPECT_FALSE( filter.checkPoint( p( 10, 0, -.01 )));
  EXPECT_TRUE( filter.checkPoint( p( 10, 0, .01 )));
}

TEST( RangeBasedPointFilter, one_point )
{
  RangeBasedPointFilter filter;
  filter.addControlPoint( 1, 2 );
  EXPECT_FALSE( filter.checkPoint( p( 0, 0, 1.99 )));
  EXPECT_TRUE( filter.checkPoint( p( 0, 0, 2.01 )));
  EXPECT_FALSE( filter.checkPoint( p( 10, 0, 1.99 )));
  EXPECT_TRUE( filter.checkPoint( p( 10, 0, 2.01 )));
}

TEST( RangeBasedPointFilter, three_points )
{
  RangeBasedPointFilter filter;
  filter.addControlPoint( 1, 2 );
  filter.addControlPoint( 2, 3 );
  filter.addControlPoint( 3, 5 );
  EXPECT_FALSE( filter.checkPoint( p( 0, 0, 1.99 )));
  EXPECT_TRUE( filter.checkPoint( p( 0, 0, 2.01 )));
  EXPECT_FALSE( filter.checkPoint( p( 1, 0, 1.99 )));
  EXPECT_TRUE( filter.checkPoint( p( 1, 0, 2.01 )));
  EXPECT_FALSE( filter.checkPoint( p( 1.5, 0, 2.49 )));
  EXPECT_TRUE( filter.checkPoint( p( 1.5, 0, 2.51 )));
  EXPECT_FALSE( filter.checkPoint( p( 2.5, 0, 3.99 )));
  EXPECT_TRUE( filter.checkPoint( p( 2.5, 0, 4.01 )));
  EXPECT_FALSE( filter.checkPoint( p( 3.5, 0, 5.99 )));
  EXPECT_TRUE( filter.checkPoint( p( 3.5, 0, 6.01 )));
}

// Test the bounds the same as above but with the ranges coming from
// points along a 45 degree line instead of along the axis.
TEST( RangeBasedPointFilter, polar_coordinates )
{
  RangeBasedPointFilter filter;
  filter.addControlPoint( 1, 2 );
  filter.addControlPoint( 2, 3 );
  filter.addControlPoint( 3, 5 );
  double r2o2 = 0.5 * sqrt(2); // root 2 over 2
  EXPECT_FALSE( filter.checkPoint( p( r2o2, r2o2, 1.99 )));
  EXPECT_TRUE( filter.checkPoint( p( r2o2, r2o2, 2.01 )));
  EXPECT_FALSE( filter.checkPoint( p( 1.5 * r2o2, 1.5 * r2o2, 2.49 )));
  EXPECT_TRUE( filter.checkPoint( p( 1.5 * r2o2, 1.5 * r2o2, 2.51 )));
  EXPECT_FALSE( filter.checkPoint( p( 2.5 * r2o2, 2.5 * r2o2, 3.99 )));
  EXPECT_TRUE( filter.checkPoint( p( 2.5 * r2o2, 2.5 * r2o2, 4.01 )));
  EXPECT_FALSE( filter.checkPoint( p( 3.5 * r2o2, 3.5 * r2o2, 5.99 )));
  EXPECT_TRUE( filter.checkPoint( p( 3.5 * r2o2, 3.5 * r2o2, 6.01 )));
}

// Same as above but with filter at a non-zero position and orientation.
TEST( RangeBasedPointFilter, filter_pose )
{
  RangeBasedPointFilter filter;
  filter.setFilterPose( tf::Pose( tf::Quaternion( tf::Vector3( 0, 0, 1), M_PI / 2 ), tf::Vector3( 1, 2, 3 )));
  filter.addControlPoint( 1, 2 );
  filter.addControlPoint( 2, 3 );
  filter.addControlPoint( 3, 5 );
  EXPECT_FALSE( filter.checkPoint( p( 1, 2, 4.99 )));
  EXPECT_TRUE( filter.checkPoint( p( 1, 2, 5.01 )));
  EXPECT_FALSE( filter.checkPoint( p( 1, 3, 4.99 )));
  EXPECT_TRUE( filter.checkPoint( p( 1, 3, 5.01 )));
  EXPECT_FALSE( filter.checkPoint( p( 1, 3.5, 5.49 )));
  EXPECT_TRUE( filter.checkPoint( p( 1, 3.5, 5.51 )));
  EXPECT_FALSE( filter.checkPoint( p( 1, 4.5, 6.99 )));
  EXPECT_TRUE( filter.checkPoint( p( 1, 4.5, 7.01 )));
  EXPECT_FALSE( filter.checkPoint( p( 1, 5.5, 8.99 )));
  EXPECT_TRUE( filter.checkPoint( p( 1, 5.5, 9.01 )));
}

TEST( RangeBasedPointFilter, clear )
{
  RangeBasedPointFilter filter;
  filter.setFilterPose( tf::Pose( tf::Quaternion( tf::Vector3( 0, 0, 1), M_PI / 2 ), tf::Vector3( 1, 2, 3 )));
  filter.addControlPoint( 1, 2 );
  filter.addControlPoint( 2, 3 );
  filter.addControlPoint( 3, 5 );
  EXPECT_FALSE( filter.checkPoint( p( 1, 5.5, 8.99 )));
  EXPECT_TRUE( filter.checkPoint( p( 1, 5.5, 9.01 )));

  filter.clear();
  EXPECT_FALSE( filter.checkPoint( p( 0, 0, -.01 )));
  EXPECT_TRUE( filter.checkPoint( p( 0, 0, .01 )));
  EXPECT_FALSE( filter.checkPoint( p( 10, 0, -.01 )));
  EXPECT_TRUE( filter.checkPoint( p( 10, 0, .01 )));
}

TEST( RangeBasedPointFilter, describe )
{
  RangeBasedPointFilter filter;
  filter.addControlPoint( 1.1, 2.0 );
  filter.addControlPoint( 3.4, -0.01 );
  EXPECT_EQ( "1.1 2  3.4 -0.01  ", filter.describe() );
}

TEST( RangeBasedPointFilter, readDescription )
{
  RangeBasedPointFilter filter;
  filter.readDescription( "1.1 2.0 3.4 -0.01" );
  EXPECT_EQ( 3, filter.size() );

  EXPECT_EQ( 0.0f, filter.getRange( 0 ));
  EXPECT_EQ( 2.0f, filter.getZ( 0 ));

  EXPECT_EQ( 1.1f, filter.getRange( 1 ));
  EXPECT_EQ( 2.0f, filter.getZ( 1 ));

  EXPECT_EQ( 3.4f, filter.getRange( 2 ));
  EXPECT_EQ( -0.01f, filter.getZ( 2 ));
}

int main( int argc, char **argv ) {
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
