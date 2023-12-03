/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

// Tests ripped from https://github.com/locusrobotics/robot_navigation/blob/master/nav_grid/test/utest.cpp

#include <gtest/gtest.h>
#include <costmap_2d/costmap_2d.h>

using namespace costmap_2d;

TEST(CostmapCoordinates, easy_coordinates_test)
{
  Costmap2D costmap(2, 3, 1.0, 0, 0);

  double wx, wy;
  costmap.mapToWorld(0u, 0u, wx, wy);
  EXPECT_DOUBLE_EQ(wx, 0.5);
  EXPECT_DOUBLE_EQ(wy, 0.5);
  costmap.mapToWorld(1u, 2u, wx, wy);
  EXPECT_DOUBLE_EQ(wx, 1.5);
  EXPECT_DOUBLE_EQ(wy, 2.5);

  unsigned int umx, umy;
  int mx, my;
  ASSERT_TRUE(costmap.worldToMap(wx, wy, umx, umy));
  EXPECT_EQ(umx, 1);
  EXPECT_EQ(umy, 2);
  costmap.worldToMapNoBounds(wx, wy, mx, my);
  EXPECT_EQ(mx, 1);
  EXPECT_EQ(my, 2);

  // Invalid Coordinate
  wx = 2.5;
  EXPECT_FALSE(costmap.worldToMap(wx, wy, umx, umy));
  costmap.worldToMapEnforceBounds(wx, wy, mx, my);
  EXPECT_EQ(mx, 1);
  EXPECT_EQ(my, 2);
  costmap.worldToMapNoBounds(wx, wy, mx, my);
  EXPECT_EQ(mx, 2);
  EXPECT_EQ(my, 2);

  // Border Cases
  EXPECT_TRUE(costmap.worldToMap(0.0, wy, umx, umy));
  EXPECT_EQ(umx, 0);
  EXPECT_TRUE(costmap.worldToMap(0.25, wy, umx, umy));
  EXPECT_EQ(umx, 0);
  EXPECT_TRUE(costmap.worldToMap(0.75, wy, umx, umy));
  EXPECT_EQ(umx, 0);
  EXPECT_TRUE(costmap.worldToMap(0.9999, wy, umx, umy));
  EXPECT_EQ(umx, 0);
  EXPECT_TRUE(costmap.worldToMap(1.0, wy, umx, umy));
  EXPECT_EQ(umx, 1);
  EXPECT_TRUE(costmap.worldToMap(1.25, wy, umx, umy));
  EXPECT_EQ(umx, 1);
  EXPECT_TRUE(costmap.worldToMap(1.75, wy, umx, umy));
  EXPECT_EQ(umx, 1);
  EXPECT_TRUE(costmap.worldToMap(1.9999, wy, umx, umy));
  EXPECT_EQ(umx, 1);
  EXPECT_FALSE(costmap.worldToMap(2.0, wy, umx, umy));
  costmap.worldToMapEnforceBounds(2.0, wy, mx, my);
  EXPECT_EQ(mx, 1);
}

TEST(CostmapCoordinates, hard_coordinates_test)
{
  Costmap2D costmap(2, 3, 0.1, -0.2, 0.2);

  double wx, wy;
  costmap.mapToWorld(0, 0, wx, wy);
  EXPECT_DOUBLE_EQ(wx, -0.15);
  EXPECT_DOUBLE_EQ(wy, 0.25);
  costmap.mapToWorld(1, 2, wx, wy);
  EXPECT_DOUBLE_EQ(wx, -0.05);
  EXPECT_DOUBLE_EQ(wy, 0.45);

  unsigned int umx, umy;
  int mx, my;
  EXPECT_TRUE(costmap.worldToMap(wx, wy, umx, umy));
  EXPECT_EQ(umx, 1);
  EXPECT_EQ(umy, 2);
  costmap.worldToMapNoBounds(wx, wy, mx, my);
  EXPECT_EQ(mx, 1);
  EXPECT_EQ(my, 2);

  // Invalid Coordinate
  wx = 2.5;
  EXPECT_FALSE(costmap.worldToMap(wx, wy, umx, umy));
  costmap.worldToMapEnforceBounds(wx, wy, mx, my);
  EXPECT_EQ(mx, 1);
  EXPECT_EQ(my, 2);
  costmap.worldToMapNoBounds(wx, wy, mx, my);
  EXPECT_EQ(mx, 27);
  EXPECT_EQ(my, 2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}

