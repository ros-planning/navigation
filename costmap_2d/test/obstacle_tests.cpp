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

/**
 * @author David Lu!!
 * Test harness for ObstacleLayer for Costmap2D
 */

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>
#include <testing_helper.h>
#include <set>
#include <gtest/gtest.h>
#include <tf/transform_listener.h>

using namespace costmap_2d;

/**
 * Test for ray tracing free space
 */
TEST(costmap, testRaytracing){
  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf);
  ObstacleLayer* olayer = addObstacleLayer(layers, tf);
  
  // Add a point cloud, should not affect the map
  addObservation(olayer, 0.0, 0.0, MAX_Z/2, 0, 0, MAX_Z/2);

  layers.updateMap(0,0,0);
  //printMap(*(layers.getCostmap()));
  
  int lethal_count = countValues(*(layers.getCostmap()), LETHAL_OBSTACLE);
  //we expect just one obstacle to be added (20 in static map)
  ASSERT_EQ(lethal_count, 21);
}

/**
 * Test for ray tracing free space
 */
TEST(costmap, testRaytracing2){
  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf);
  ObstacleLayer* olayer = addObstacleLayer(layers, tf);

  layers.updateMap(0,0,0);
  int obs_before =  countValues(*(layers.getCostmap()), LETHAL_OBSTACLE);

  // The sensor origin will be <0,0>. So if we add an obstacle at 9,9, we would expect cells
  // <0, 0> thru <8, 8> to be traced through
  addObservation(olayer, 9.5, 9.5, MAX_Z/2, 0.5, 0.5, MAX_Z/2);
  layers.updateMap(0,0,0);
  int obs_after = countValues(*(layers.getCostmap()), LETHAL_OBSTACLE);

  // Change from previous test:
  // No obstacles from the static map will be cleared, so the
  // net change is +1. 
  ASSERT_EQ(obs_after, obs_before + 1);


  for(int i=0;i<olayer->getSizeInCellsY();i++){
    olayer->setCost(i,i,LETHAL_OBSTACLE);
  }
  layers.updateMap(0,0,0);
  int with_static = countValues(*(layers.getCostmap()), LETHAL_OBSTACLE);

  // Should be the same as before
  ASSERT_EQ(with_static, obs_after);
}

/**
 * Test for wave interference
 */
TEST(costmap, testWaveInterference){
  tf::TransformListener tf;
  // Start with an empty map
  LayeredCostmap layers("frame", false, true);
  layers.resizeMap(10, 10, 1, 0, 0);
  ObstacleLayer* olayer = addObstacleLayer(layers, tf);

  // Lay out 3 obstacles in a line - along the diagonal, separated by a cell.
  addObservation(olayer, 3.0, 3.0, MAX_Z);
  addObservation(olayer, 5.0, 5.0, MAX_Z);
  addObservation(olayer, 7.0, 7.0, MAX_Z);
  layers.updateMap(0,0,0);

  Costmap2D* costmap = layers.getCostmap();
  //printMap(*costmap);
  ASSERT_EQ(countValues(*costmap, costmap_2d::NO_INFORMATION), 92);
  ASSERT_EQ(countValues(*costmap, costmap_2d::LETHAL_OBSTACLE), 3);
  ASSERT_EQ(countValues(*costmap, costmap_2d::FREE_SPACE), 5);
}

/**
 * Make sure we ignore points outside of our z threshold
 */
TEST(costmap, testZThreshold){
  tf::TransformListener tf;
  // Start with an empty map
  LayeredCostmap layers("frame", false, true);
  layers.resizeMap(10, 10, 1, 0, 0);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);

  // A point cloud with 2 points falling in a cell with a non-lethal cost
  addObservation(olayer, 0.0, 5.0, 0.4);
  addObservation(olayer, 1.0, 5.0, 2.2);

  layers.updateMap(0,0,0);

  Costmap2D* costmap = layers.getCostmap();
  ASSERT_EQ(countValues(*costmap, costmap_2d::LETHAL_OBSTACLE), 1);
}


/**
 * Verify that dynamic obstacles are added
 */
TEST(costmap, testDynamicObstacles){
  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);

  // Add a point cloud and verify its insertion. There should be only one new one
  addObservation(olayer, 0.0, 0.0);
  addObservation(olayer, 0.0, 0.0);
  addObservation(olayer, 0.0, 0.0);

  layers.updateMap(0,0,0);

  Costmap2D* costmap = layers.getCostmap();
  // Should now have 1 insertion and no deletions
  ASSERT_EQ(countValues(*costmap, costmap_2d::LETHAL_OBSTACLE), 21);

  // Repeating the call - we should see no insertions or deletions
  ASSERT_EQ(countValues(*costmap, costmap_2d::LETHAL_OBSTACLE), 21);
}


/**
 * Verify that if we add a point that is already a static obstacle we do not end up with a new ostacle
 */
TEST(costmap, testMultipleAdditions){
  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);
  addStaticLayer(layers, tf);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);

  // A point cloud with one point that falls within an existing obstacle
  addObservation(olayer, 9.5, 0.0);
  layers.updateMap(0,0,0);
  Costmap2D* costmap = layers.getCostmap();
  //printMap(*costmap);

  ASSERT_EQ(countValues(*costmap, costmap_2d::LETHAL_OBSTACLE), 20);

}


int main(int argc, char** argv){
  ros::init(argc, argv, "obstacle_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
