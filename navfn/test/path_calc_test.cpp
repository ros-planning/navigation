/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <string>
#include <ros/package.h>
#include <gtest/gtest.h>
#include <navfn/navfn.h>
#include <navfn/read_pgm_costmap.h>

// Load a willow garage costmap and return a NavFn instance using it.
// If the costmap file fails to load, returns NULL.
navfn::NavFn* make_willow_nav()
{
  int sx,sy;
  std::string path = ros::package::getPath( ROS_PACKAGE_NAME ) + "/test/willow_costmap.pgm";

  COSTTYPE *cmap = readPGM( path.c_str(), &sx, &sy, true );
  if( cmap == NULL )
  {
    return NULL;
  }
  navfn::NavFn* nav = new navfn::NavFn(sx,sy);

  nav->priInc = 2*COST_NEUTRAL;	// thin wavefront

  memcpy( nav->costarr, cmap, sx*sy );

  return nav;
}

TEST(PathCalc, oscillate_in_pinch_point)
{
  navfn::NavFn* nav = make_willow_nav();
  ASSERT_TRUE( nav != NULL );

  int goal[2];
  int start[2];

  start[0] = 428;
  start[1] = 746;
  
  goal[0] = 350;
  goal[1] = 450;

  nav->setGoal( goal );
  nav->setStart( start );

  EXPECT_TRUE( nav->calcNavFnDijkstra( true ));
}

TEST(PathCalc, easy_nav_should_always_work)
{
  navfn::NavFn* nav = make_willow_nav();
  ASSERT_TRUE( nav != NULL );

  int goal[2];
  int start[2];

  start[0] = 350;
  start[1] = 400;

  goal[0] = 350;
  goal[1] = 450;

  nav->setGoal( goal );
  nav->setStart( start );

  EXPECT_TRUE( nav->calcNavFnDijkstra( true ));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
