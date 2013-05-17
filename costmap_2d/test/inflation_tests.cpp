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
 * @file Test harness for InflationLayer for Costmap2D
 */

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/testing_helper.h>
#include <set>
#include <gtest/gtest.h>
#include <tf/transform_listener.h>

using namespace costmap_2d;
using geometry_msgs::Point;

std::vector<Point> setRadii(LayeredCostmap& layers, double length, double width, double inflation_radius)
{
  std::vector<Point> polygon;
  Point p;
  p.x = width;
  p.y = length; 
  polygon.push_back(p);
  p.x = width;
  p.y = -length; 
  polygon.push_back(p);
  p.x = -width;
  p.y = -length; 
  polygon.push_back(p);
  p.x = -width;
  p.y = length; 
  polygon.push_back(p);
  layers.setFootprint(polygon);

  ros::NodeHandle nh;
  nh.setParam("/inflation_tests/inflation/inflation_radius", inflation_radius);

  return polygon;
}

TEST(costmap, testAdjacentToObstacleCanStillMove){
  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  // Footprint with inscribed radius = 2.1
  //               circumscribed radius = 3.1
  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3, 4.1);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);  
  InflationLayer* ilayer = addInflationLayer(layers, tf);
  ilayer->setFootprint(polygon);

  addObservation(olayer, 0, 0, MAX_Z);

  layers.updateMap(0,0,0);
  Costmap2D* costmap = layers.getCostmap();
  //printMap(*costmap);
  EXPECT_EQ( LETHAL_OBSTACLE, costmap->getCost( 0, 0 ));
  EXPECT_EQ( INSCRIBED_INFLATED_OBSTACLE, costmap->getCost( 1, 0 ));
  EXPECT_EQ( INSCRIBED_INFLATED_OBSTACLE, costmap->getCost( 2, 0 ));
  EXPECT_TRUE( INSCRIBED_INFLATED_OBSTACLE > costmap->getCost( 3, 0 ));
  EXPECT_TRUE( INSCRIBED_INFLATED_OBSTACLE > costmap->getCost( 2, 1 ));
  EXPECT_EQ( INSCRIBED_INFLATED_OBSTACLE, costmap->getCost( 1, 1 ));
}

TEST(costmap, testInflationShouldNotCreateUnknowns){
  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);
  layers.resizeMap(10, 10, 1, 0, 0);

  // Footprint with inscribed radius = 2.1
  //               circumscribed radius = 3.1
  std::vector<Point> polygon = setRadii(layers, 2.1, 2.3, 4.1);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);  
  InflationLayer* ilayer = addInflationLayer(layers, tf);
  ilayer->setFootprint(polygon);

  addObservation(olayer, 0, 0, MAX_Z);

  layers.updateMap(0,0,0);
  Costmap2D* costmap = layers.getCostmap();

  EXPECT_EQ( countValues(*costmap, NO_INFORMATION), 0 );
}


/**
 * Test for the cost function correctness with a larger range and different values
 */
TEST(costmap, testCostFunctionCorrectness){
  tf::TransformListener tf;
  LayeredCostmap layers("frame", false, false);
  layers.resizeMap(100, 100, 1, 0, 0);

  // Footprint with inscribed radius = 5.0
  //               circumscribed radius = 8.0
  std::vector<Point> polygon = setRadii(layers, 5.0, 6.25, 10.5);

  ObstacleLayer* olayer = addObstacleLayer(layers, tf);  
  InflationLayer* ilayer = addInflationLayer(layers, tf);
  ilayer->setFootprint(polygon);

  addObservation(olayer, 50, 50, MAX_Z);

  layers.updateMap(0,0,0);
  Costmap2D* map = layers.getCostmap();

  // Verify that the circumscribed cost lower bound is as expected: based on the cost function.
  //unsigned char c = ilayer->computeCost(8.0);
  //ASSERT_EQ(ilayer->getCircumscribedCost(), c);

  for(unsigned int i = 0; i <= (unsigned int)ceil(5.0); i++){
    // To the right
    ASSERT_EQ(map->getCost(50 + i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50 + i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // To the left
    ASSERT_EQ(map->getCost(50 - i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50 - i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // Down
    ASSERT_EQ(map->getCost(50, 50 + i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50, 50 + i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // Up
    ASSERT_EQ(map->getCost(50, 50 - i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map->getCost(50, 50 - i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
  }

  // Verify the normalized cost attenuates as expected
  for(unsigned int i = (unsigned int)(ceil(5.0) + 1); i <= (unsigned int)ceil(10.5); i++){
    unsigned char expectedValue = ilayer->computeCost(i/1.0);
    ASSERT_EQ(map->getCost(50 + i, 50), expectedValue);
  }

  // Update with no hits. Should clear (revert to the static map
  /*map->resetMapOutsideWindow(0, 0, 0.0, 0.0);
  cloud.points.resize(0);

  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs2(p, cloud, 100.0, 100.0);
  std::vector<Observation> obsBuf2;
  obsBuf2.push_back(obs2);

  map->updateWorld(0, 0, obsBuf2, obsBuf2);

  for(unsigned int i = 0; i < 100; i++)
    for(unsigned int j = 0; j < 100; j++)
      ASSERT_EQ(map->getCost(i, j), costmap_2d::FREE_SPACE);*/
}



/**
 * Test inflation for both static and dynamic obstacles
 *
TEST(costmap, testInflation){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS, 
      10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  // Verify that obstacles correctly identified
  std::vector<unsigned int> occupiedCells;

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE || map.getCost(i, j) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        occupiedCells.push_back(map.getIndex(i, j));
      }
    }
  }

  // There should be no duplicates
  std::set<unsigned int> setOfCells;
  for(unsigned int i=0;i<occupiedCells.size(); i++)
    setOfCells.insert(i);

  ASSERT_EQ(setOfCells.size(), occupiedCells.size());
  ASSERT_EQ(setOfCells.size(), (unsigned int)48);

  // Iterate over all id's and verify they are obstacles
  for(std::vector<unsigned int>::const_iterator it = occupiedCells.begin(); it != occupiedCells.end(); ++it){
    unsigned int ind = *it;
    unsigned int x, y;
    map.indexToCells(ind, x, y);
    ASSERT_EQ(find(occupiedCells, map.getIndex(x, y)), true);
    ASSERT_EQ(map.getCost(x, y) == costmap_2d::LETHAL_OBSTACLE || map.getCost(x, y) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
  }

  // Set an obstacle at the origin and observe insertions for it and its neighbors
  pcl::PointCloud<pcl::PointXYZ> c0;
  c0.points.resize(1);
  c0.points[0].x = 0;
  c0.points[0].y = 0;
  c0.points[0].z = 0.4;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, c0, 100.0, 100.0);
  std::vector<Observation> obsBuf, empty;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, empty);

  occupiedCells.clear();
  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE || map.getCost(i, j) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        occupiedCells.push_back(map.getIndex(i, j));
      }
    }
  }

  // It and its 2 neighbors makes 3 obstacles
  ASSERT_EQ(occupiedCells.size(), (unsigned int)51);

  // @todo Rewrite 
  // Add an obstacle at <2,0> which will inflate and refresh to of the other inflated cells
  pcl::PointCloud<pcl::PointXYZ> c1;
  c1.points.resize(1);
  c1.points[0].x = 2;
  c1.points[0].y = 0;
  c1.points[0].z = 0.0;

  geometry_msgs::Point p1;
  p1.x = 0.0;
  p1.y = 0.0;
  p1.z = MAX_Z;

  Observation obs1(p1, c1, 100.0, 100.0);
  std::vector<Observation> obsBuf1;
  obsBuf1.push_back(obs1);

  map.updateWorld(0, 0, obsBuf1, empty);

  occupiedCells.clear();
  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE || map.getCost(i, j) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        occupiedCells.push_back(map.getIndex(i, j));
      }
    }
  }

  // Now we expect insertions for it, and 2 more neighbors, but not all 5. Free space will propagate from
  // the origin to the target, clearing the point at <0, 0>, but not over-writing the inflation of the obstacle
  // at <0, 1>
  ASSERT_EQ(occupiedCells.size(), (unsigned int)54);


  // Add an obstacle at <1, 9>. This will inflate obstacles around it
  pcl::PointCloud<pcl::PointXYZ> c2;
  c2.points.resize(1);
  c2.points[0].x = 1;
  c2.points[0].y = 9;
  c2.points[0].z = 0.0;

  geometry_msgs::Point p2;
  p2.x = 0.0;
  p2.y = 0.0;
  p2.z = MAX_Z;

  Observation obs2(p2, c2, 100.0, 100.0);
  std::vector<Observation> obsBuf2;
  obsBuf2.push_back(obs2);

  map.updateWorld(0, 0, obsBuf2, empty);

  ASSERT_EQ(map.getCost(1, 9), costmap_2d::LETHAL_OBSTACLE);
  ASSERT_EQ(map.getCost(0, 9), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  ASSERT_EQ(map.getCost(2, 9), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  // Add an obstacle and verify that it over-writes its inflated status
  pcl::PointCloud<pcl::PointXYZ> c3;
  c3.points.resize(1);
  c3.points[0].x = 0;
  c3.points[0].y = 9;
  c3.points[0].z = 0.0;

  geometry_msgs::Point p3;
  p3.x = 0.0;
  p3.y = 0.0;
  p3.z = MAX_Z;

  Observation obs3(p3, c3, 100.0, 100.0);
  std::vector<Observation> obsBuf3;
  obsBuf3.push_back(obs3);

  map.updateWorld(0, 0, obsBuf3, empty);

  ASSERT_EQ(map.getCost(0, 9), costmap_2d::LETHAL_OBSTACLE);
}

/**
 * Test specific inflation scenario to ensure we do not set inflated obstacles to be raw obstacles.
 *
TEST(costmap, testInflation2){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS, 
      10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  // Creat a small L-Shape all at once
  pcl::PointCloud<pcl::PointXYZ> c0;
  c0.points.resize(3);
  c0.points[0].x = 1;
  c0.points[0].y = 1;
  c0.points[0].z = MAX_Z;
  c0.points[1].x = 1;
  c0.points[1].y = 2;
  c0.points[1].z = MAX_Z;
  c0.points[2].x = 2;
  c0.points[2].y = 2;
  c0.points[2].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, c0, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  ASSERT_EQ(map.getCost(3, 2), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);  
  ASSERT_EQ(map.getCost(3, 3), costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

/**
 * Test inflation behavior, starting with an empty map
 *
TEST(costmap, testInflation3){
  std::vector<unsigned char> mapData;
  for(unsigned int i=0; i< GRID_WIDTH; i++){
    for(unsigned int j = 0; j < GRID_HEIGHT; j++){
      mapData.push_back(0);
    }
  }

  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS * 2, ROBOT_RADIUS * 3, 
      10.0, MAX_Z, 10.0, 1, mapData, THRESHOLD);

  // There should be no occupied cells
  std::vector<unsigned int> ids;

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE || map.getCost(i, j) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        ids.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(ids.size(), (unsigned int)0);

  // Add an obstacle at 5,5
  pcl::PointCloud<pcl::PointXYZ> c0;
  c0.points.resize(1);
  c0.points[0].x = 5;
  c0.points[0].y = 5;
  c0.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, c0, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) != costmap_2d::FREE_SPACE){
        ids.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(ids.size(), (unsigned int)29);

  ids.clear();
  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE || map.getCost(i, j) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        ids.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(ids.size(), (unsigned int)5);

  // Update again - should see no change
  map.updateWorld(0, 0, obsBuf, obsBuf);

  ids.clear();
  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) != costmap_2d::FREE_SPACE){
        ids.push_back(map.getIndex(i, j));
      }
    }
  }
  
  ASSERT_EQ(ids.size(), (unsigned int)29);
}
//*/

int main(int argc, char** argv){
  ros::init(argc, argv, "inflation_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
