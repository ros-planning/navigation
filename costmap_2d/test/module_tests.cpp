/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 * @author Conor McGann
 * @file Test harness for Costmap2D
 */

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/observation_buffer.h>
#include <set>
#include <gtest/gtest.h>
#include <tf/transform_listener.h>

using namespace costmap_2d;

const unsigned char MAP_10_BY_10_CHAR[] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 200, 200, 200,
  0, 0, 0, 0, 100, 0, 0, 200, 200, 200,
  0, 0, 0, 0, 100, 0, 0, 200, 200, 200,
  70, 70, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 200, 200, 200, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 255, 255, 255,
  0, 0, 0, 0, 0, 0, 0, 255, 255, 255
};

std::vector<unsigned char> MAP_10_BY_10;
std::vector<unsigned char> EMPTY_10_BY_10;
std::vector<unsigned char> EMPTY_100_BY_100;

const unsigned int GRID_WIDTH(10);
const unsigned int GRID_HEIGHT(10);
const double RESOLUTION(1);
const double WINDOW_LENGTH(10);
const unsigned char THRESHOLD(100);
const double MAX_Z(1.0);
const double RAYTRACE_RANGE(20.0);
const double OBSTACLE_RANGE(20.0);
const double ROBOT_RADIUS(1.0);

bool find(const std::vector<unsigned int>& l, unsigned int n){
  for(std::vector<unsigned int>::const_iterator it = l.begin(); it != l.end(); ++it){
    if(*it == n)
      return true;
  }

  return false;
}

/**
 * Tests the reset method
 */
TEST(costmap, testResetForStaticMap){
  // Define a static map with a large object in the center
  std::vector<unsigned char> staticMap;
  for(unsigned int i=0; i<10; i++){
    for(unsigned int j=0; j<10; j++){
      staticMap.push_back(costmap_2d::LETHAL_OBSTACLE);
    }
  }

  // Allocate the cost map, with a inflation to 3 cells all around
  Costmap2D map(10, 10, RESOLUTION, 0.0, 0.0, 3, 3, 3, OBSTACLE_RANGE, MAX_Z, RAYTRACE_RANGE, 25, staticMap, THRESHOLD);

  // Populate the cost map with a wall around the perimeter. Free space should clear out the room.
  sensor_msgs::PointCloud cloud;
  cloud.set_points_size(40);

  // Left wall
  unsigned int ind = 0;
  for (unsigned int i = 0; i < 10; i++){
    // Left
    cloud.points[ind].x = 0;
    cloud.points[ind].y = i;
    cloud.points[ind].z = MAX_Z;
    ind++;

    // Top
    cloud.points[ind].x = i;
    cloud.points[ind].y = 0;
    cloud.points[ind].z = MAX_Z;
    ind++;

    // Right
    cloud.points[ind].x = 9;
    cloud.points[ind].y = i;
    cloud.points[ind].z = MAX_Z;
    ind++;

    // Bottom
    cloud.points[ind].x = i;
    cloud.points[ind].y = 9;
    cloud.points[ind].z = MAX_Z;
    ind++;
  }

  double wx = 5.0, wy = 5.0;
  geometry_msgs::Point p;
  p.x = wx;
  p.y = wy;
  p.z = MAX_Z;
  Observation obs(p, cloud, OBSTACLE_RANGE, RAYTRACE_RANGE);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  // Update the cost map for this observation
  map.updateWorld(wx, wy, obsBuf, obsBuf);

  // Verify that we now have only 36 cells with lethal cost, thus provong that we have correctly cleared
  // free space
  int hitCount = 0;
  for(unsigned int i=0; i < 10; ++i){
    for(unsigned int j=0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE){
        hitCount++;
      }
    }
  }
  ASSERT_EQ(hitCount, 36);

  // Veriy that we have 64 non-leathal
  hitCount = 0;
  for(unsigned int i=0; i < 10; ++i){
    for(unsigned int j=0; j < 10; ++j){
      if(map.getCost(i, j) != costmap_2d::LETHAL_OBSTACLE)
        hitCount++;
    }
  }
  ASSERT_EQ(hitCount, 64);

  // Now if we reset the cost map, we should have our map go back to being completely occupied
  map.resetMapOutsideWindow(wx, wy, 0.0, 0.0);

  //We should now go back to everything being occupied
  hitCount = 0;
  for(unsigned int i=0; i < 10; ++i){
    for(unsigned int j=0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
        hitCount++;
    }
  }
  ASSERT_EQ(hitCount, 100);

}

/**
 * Basic testing for observation buffer
TEST(costmap, test15){
  double keep_alive = 10.0;
  double refresh_interval = 0.02; // 200 ms
  tf::TransformListener tf;
  ObservationBuffer buffer("Foo", keep_alive, refresh_interval, 0.0, MAX_Z, OBSTACLE_RANGE, RAYTRACE_RANGE, tf, "map", "map");

  // Initially it should be false
  ASSERT_EQ(buffer.isCurrent(), false);

  geometry_msgs::Point origin; // Map origin
  origin.x = 0;
  origin.y = 0;
  origin.z = 0;

  ros::Time epoch; // Beginning of time

  // Buffer a point cloud with a time stamp that is very old. It should still not be current
  sensor_msgs::PointCloud* p0 = new sensor_msgs::PointCloud();
  p0->set_points_size(1);
  p0->points[0].x = 50;
  p0->points[0].y = 50;
  p0->points[0].z = MAX_Z;
  p0->header.stamp = epoch;
  p0->header.frame_id = "map";
  buffer.bufferCloud(*p0);
  // Up to date - ignores the time stamp.
  ASSERT_EQ(buffer.isCurrent(), true);

  // Now buffer another which has a current time stamp
  sensor_msgs::PointCloud* p1 = new sensor_msgs::PointCloud();
  p1->set_points_size(1);
  p1->points[0].x = 50;
  p1->points[0].y = 50;
  p1->points[0].z = MAX_Z;
  p1->header.stamp = ros::Time::now();
  p1->header.frame_id = "map";
  buffer.bufferCloud(*p1);
  ASSERT_EQ(buffer.isCurrent(), true);

  // Now go again after sleeping for a bit too long - 300 ms
  ros::Time oldValue = ros::Time::now();
  ros::Duration excessiveSleep(0, 300000000);
  excessiveSleep.sleep();
  ASSERT_EQ(buffer.isCurrent(), false);
}
 */

/**
 * Test for the cost function correctness with a larger range and different values
 */
TEST(costmap, test14){
  Costmap2D map(100, 100, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS * 5.0, ROBOT_RADIUS * 8.0, ROBOT_RADIUS * 10.5, 
      100.0, MAX_Z, 100.0, 25, EMPTY_100_BY_100, THRESHOLD);

  // Verify that the circumscribed cost lower bound is as expected: based on the cost function.
  unsigned char c = map.computeCost((ROBOT_RADIUS * 8.0 / RESOLUTION));
  ASSERT_EQ(map.getCircumscribedCost(), c);

  // Add a point in the center
  sensor_msgs::PointCloud cloud;
  cloud.set_points_size(1);
  cloud.points[0].x = 50;
  cloud.points[0].y = 50;
  cloud.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, cloud, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  for(unsigned int i = 0; i <= (unsigned int)ceil(ROBOT_RADIUS * 5.0); i++){
    // To the right
    ASSERT_EQ(map.getCost(50 + i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map.getCost(50 + i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // To the left
    ASSERT_EQ(map.getCost(50 - i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map.getCost(50 - i, 50) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // Down
    ASSERT_EQ(map.getCost(50, 50 + i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map.getCost(50, 50 + i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    // Up
    ASSERT_EQ(map.getCost(50, 50 - i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
    ASSERT_EQ(map.getCost(50, 50 - i) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE, true);
  }

  // Verify the normalized cost attenuates as expected
  for(unsigned int i = (unsigned int)(ceil(ROBOT_RADIUS * 5.0) + 1); i <= (unsigned int)ceil(ROBOT_RADIUS * 10.5); i++){
    unsigned char expectedValue = map.computeCost(i / RESOLUTION);
    ASSERT_EQ(map.getCost(50 + i, 50), expectedValue);
  }

  // Update with no hits. Should clear (revert to the static map
  map.resetMapOutsideWindow(0, 0, 0.0, 0.0);
  cloud.set_points_size(0);

  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs2(p, cloud, 100.0, 100.0);
  std::vector<Observation> obsBuf2;
  obsBuf2.push_back(obs2);

  map.updateWorld(0, 0, obsBuf2, obsBuf2);

  for(unsigned int i = 0; i < 100; i++)
    for(unsigned int j = 0; j < 100; j++)
      ASSERT_EQ(map.getCost(i, j), costmap_2d::FREE_SPACE);
}

/**
 * Test for wave interference
 */
TEST(costmap, test12){
  // Start with an empty map
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS * 2, ROBOT_RADIUS * 3.01,
      10.0, MAX_Z * 2, 10.0, 1, EMPTY_10_BY_10, THRESHOLD);

  // Lay out 3 obstacles in a line - along the diagonal, separated by a cell.
  sensor_msgs::PointCloud cloud;
  cloud.set_points_size(3);
  cloud.points[0].x = 3;
  cloud.points[0].y = 3;
  cloud.points[0].z = MAX_Z;
  cloud.points[1].x = 5;
  cloud.points[1].y = 5;
  cloud.points[1].z = MAX_Z;
  cloud.points[2].x = 7;
  cloud.points[2].y = 7;
  cloud.points[2].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, cloud, 100.0, 100.0);
  std::vector<Observation> obsBuf;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  int update_count = 0;

  // Expect to see a union of obstacles
  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) != costmap_2d::FREE_SPACE){
        update_count++;
      }
    }
  }

  ASSERT_EQ(update_count, 79);
}

/**
 * Test for ray tracing free space
 */
TEST(costmap, test0){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS, 
      10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  // Add a point cloud, should not affect the map
  sensor_msgs::PointCloud cloud;
  cloud.set_points_size(1);
  cloud.points[0].x = 0;
  cloud.points[0].y = 0;
  cloud.points[0].z = MAX_Z;

  geometry_msgs::Point p;
  p.x = 0.0;
  p.y = 0.0;
  p.z = MAX_Z;

  Observation obs(p, cloud, 100.0, 100.0);
  std::vector<Observation> obsBuf, empty;
  obsBuf.push_back(obs);

  map.updateWorld(0, 0, obsBuf, obsBuf);

  int lethal_count = 0;

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE){
        lethal_count++;
      }
    }
  }

  //we expect just one obstacle to be added
  ASSERT_EQ(lethal_count, 21);
}

unsigned int worldToIndex(Costmap2D& map, double wx, double wy){
  unsigned int mx, my;
  map.worldToMap(wx, wy, mx, my);
  return map.getIndex(mx, my);
}

void indexToWorld(Costmap2D& map, unsigned int index, double& wx, double& wy){
  unsigned int mx, my;
  map.indexToCells(index, mx, my);
  map.mapToWorld(mx, my, wx, wy);
}

TEST(costmap, test1){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS, 
      10.0, MAX_Z, 10.0, 25, MAP_10_BY_10, THRESHOLD);

  ASSERT_EQ(map.getSizeInCellsX(), (unsigned int)10);
  ASSERT_EQ(map.getSizeInCellsY(), (unsigned int)10);

  // Verify that obstacles correctly identified from the static map.
  std::vector<unsigned int> occupiedCells;

  for(unsigned int i = 0; i < 10; ++i){
    for(unsigned int j = 0; j < 10; ++j){
      if(map.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE){
        occupiedCells.push_back(map.getIndex(i, j));
      }
    }
  }

  ASSERT_EQ(occupiedCells.size(), (unsigned int)20);

  // Iterate over all id's and verify that they are present according to their
  for(std::vector<unsigned int>::const_iterator it = occupiedCells.begin(); it != occupiedCells.end(); ++it){
    unsigned int ind = *it;
    unsigned int x, y;
    map.indexToCells(ind, x, y);
    ASSERT_EQ(find(occupiedCells, map.getIndex(x, y)), true);
    ASSERT_EQ(MAP_10_BY_10[ind] >= 100, true);
    ASSERT_EQ(map.getCost(x, y) >= 100, true);
  }

  // Block of 200
  ASSERT_EQ(find(occupiedCells, map.getIndex(7, 2)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(8, 2)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(9, 2)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(7, 3)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(8, 3)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(9, 3)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(7, 4)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(8, 4)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(9, 4)), true);

  // Block of 100
  ASSERT_EQ(find(occupiedCells, map.getIndex(4, 3)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(4, 4)), true);

  // Block of 200
  ASSERT_EQ(find(occupiedCells, map.getIndex(3, 7)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(4, 7)), true);
  ASSERT_EQ(find(occupiedCells, map.getIndex(5, 7)), true);


  // Verify Coordinate Transformations, ROW MAJOR ORDER
  ASSERT_EQ(worldToIndex(map, 0.0, 0.0), (unsigned int)0);
  ASSERT_EQ(worldToIndex(map, 0.0, 0.99), (unsigned int)0);
  ASSERT_EQ(worldToIndex(map, 0.0, 1.0), (unsigned int)10);
  ASSERT_EQ(worldToIndex(map, 1.0, 0.99), (unsigned int)1);
  ASSERT_EQ(worldToIndex(map, 9.99, 9.99), (unsigned int)99);
  ASSERT_EQ(worldToIndex(map, 8.2, 3.4), (unsigned int)38);

  // Ensure we hit the middle of the cell for world co-ordinates
  double wx, wy;
  indexToWorld(map, 99, wx, wy);
  ASSERT_EQ(wx, 9.5);
  ASSERT_EQ(wy, 9.5);
}


/**
 * Verify that dynamic obstacles are added

TEST(costmap, test3){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, MAP_10_BY_10, RESOLUTION, THRESHOLD);

  // Add a point cloud and verify its insertion. There should be only one new one
  sensor_msgs::PointCloud cloud;
  cloud.set_points_size(3);
  cloud.points[0].x = 0;
  cloud.points[0].y = 0;
  cloud.points[1].x = 0;
  cloud.points[1].y = 0;
  cloud.points[2].x = 0;
  cloud.points[2].y = 0;

  std::vector<unsigned int> updates;
  std::vector<unsigned int> ids;
  map.updateDynamicObstacles(cloud, updates);

  // Should now have 1 insertion and no deletions
  ASSERT_EQ(updates.size(), 1);
  map.getOccupiedCellDataIndexList(ids);
  ASSERT_EQ(ids.size(), 21);

  // Repeating the call - we should see no insertions or deletions
  map.updateDynamicObstacles(cloud, updates);
  ASSERT_EQ(updates.empty(), true);
}
 */

/**
 * Verify that if we add a point that is already a static obstacle we do not end up with a new ostacle
TEST(costmap, test4){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, MAP_10_BY_10, RESOLUTION, THRESHOLD);

  // A point cloud with one point that falls within an existing obstacle
  sensor_msgs::PointCloud cloud;
  cloud.set_points_size(1);
  cloud.points[0].x = 7;
  cloud.points[0].y = 2;

  std::vector<unsigned int> updates;
  std::vector<unsigned int> ids;
  map.updateDynamicObstacles(cloud, updates);
  ASSERT_EQ(updates.empty(), true);
}
 */

/**
 * Make sure we ignore points outside of our z threshold
TEST(costmap, test6){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, MAP_10_BY_10, RESOLUTION, THRESHOLD, MAX_Z, MAX_Z, MAX_Z);

  // A point cloud with 2 points falling in a cell with a non-lethal cost
  sensor_msgs::PointCloud c0;
  c0.set_points_size(2);
  c0.points[0].x = 0;
  c0.points[0].y = 5;
  c0.points[0].z = 0.4;
  c0.points[1].x = 1;
  c0.points[1].y = 5;
  c0.points[1].z = 1.2;

  std::vector<unsigned int> updates;
  map.updateDynamicObstacles(c0, updates);
  ASSERT_EQ(updates.size(), 1);
}
 */

/**
 * Test inflation for both static and dynamic obstacles

TEST(costmap, test7){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, MAP_10_BY_10, RESOLUTION, THRESHOLD, MAX_Z, MAX_Z, MAX_Z,
		ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS);


  // Verify that obstacles correctly identified
  std::vector<unsigned int> occupiedCells;
  map.getOccupiedCellDataIndexList(occupiedCells);

  // There should be no duplicates
  std::set<unsigned int> setOfCells;
  for(unsigned int i=0;i<occupiedCells.size(); i++)
    setOfCells.insert(i);

  ASSERT_EQ(setOfCells.size(), occupiedCells.size());
  ASSERT_EQ(setOfCells.size(), 48);

  const unsigned char* costData = map.getMap();

  // Iterate over all id's and verify they are obstacles
  for(std::vector<unsigned int>::const_iterator it = occupiedCells.begin(); it != occupiedCells.end(); ++it){
    unsigned int ind = *it;
    unsigned int x, y;
    map.IND_MC(ind, x, y);
    ASSERT_EQ(find(occupiedCells, map.MC_IND(x, y)), true);
    ASSERT_EQ(costData[ind] == Costmap2D::LETHAL_OBSTACLE || costData[ind] == Costmap2D::INSCRIBED_INFLATED_OBSTACLE, true);
  }

  // Set an obstacle at the origin and observe insertions for it and its neighbors
  std::vector<unsigned int> updates;
  sensor_msgs::PointCloud c0;
  c0.set_points_size(1);
  c0.points[0].x = 0;
  c0.points[0].y = 0;
  c0.points[0].z = 0.4;
  map.updateDynamicObstacles(c0, updates);

  // It and its 2 neighbors makes 3 obstacles
  ASSERT_EQ(updates.size(), 3);

  // @todo Rewrite 
  // Add an obstacle at <2,0> which will inflate and refresh to of the other inflated cells
  sensor_msgs::PointCloud c1;
  c1.set_points_size(1);
  c1.points[0].x = 2;
  c1.points[0].y = 0;
  c1.points[0].z = 0.0;
  map.updateDynamicObstacles(c1, updates);

  // Now we expect insertions for it, and 2 more neighbors, but not all 5. Free space will propagate from
  // the origin to the target, clearing the point at <0, 0>, but not over-writing the inflation of the obstacle
  // at <0, 1>
  ASSERT_EQ(updates.size(), 3);


  // Add an obstacle at <1, 9>. This will inflate obstacles around it
  std::vector<sensor_msgs::PointCloud*> cv2;
  sensor_msgs::PointCloud c2;
  cv2.push_back(&c2);
  c2.set_points_size(1);
  c2.points[0].x = 1;
  c2.points[0].y = 9;
  c2.points[0].z = 0.0;
  map.updateDynamicObstacles(0.0, 0.0, cv2); //, updates); //WINDOW_LENGTH + 2
  ASSERT_EQ(map.getCost(1, 9), Costmap2D::LETHAL_OBSTACLE);
  ASSERT_EQ(map.getCost(0, 9), Costmap2D::INSCRIBED_INFLATED_OBSTACLE);
  ASSERT_EQ(map.getCost(2, 9), Costmap2D::INSCRIBED_INFLATED_OBSTACLE);

  // Add an obstacle and verify that it over-writes its inflated status
  std::vector<sensor_msgs::PointCloud*> cv3;
  sensor_msgs::PointCloud c3;
  cv3.push_back(&c3);
  c3.set_points_size(1);
  c3.points[0].x = 0;
  c3.points[0].y = 9;
  c3.points[0].z = 0.0;
  map.updateDynamicObstacles(0.0, 0.0, cv3); //, updates); //WINDOW_LENGTH + 3
  ASSERT_EQ(map.getCost(0, 9), Costmap2D::LETHAL_OBSTACLE);
}
 */

/**
 * Test specific inflation scenario to ensure we do not set inflated obstacles to be raw obstacles.
TEST(costmap, test8){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, MAP_10_BY_10, RESOLUTION, THRESHOLD, MAX_Z, MAX_Z, MAX_Z, 
		ROBOT_RADIUS, ROBOT_RADIUS, ROBOT_RADIUS);

  std::vector<unsigned int> updates;

  // Creat a small L-Shape all at once
  sensor_msgs::PointCloud c0;
  c0.set_points_size(3);
  c0.points[0].x = 1;
  c0.points[0].y = 1;
  c0.points[0].z = MAX_Z;
  c0.points[1].x = 1;
  c0.points[1].y = 2;
  c0.points[1].z = MAX_Z;
  c0.points[2].x = 2;
  c0.points[2].y = 2;
  c0.points[2].z = MAX_Z;

  map.updateDynamicObstacles(c0, updates);

  ASSERT_EQ(map.getCost(3, 2), Costmap2D::INSCRIBED_INFLATED_OBSTACLE);  
  ASSERT_EQ(map.getCost(3, 3), Costmap2D::INSCRIBED_INFLATED_OBSTACLE);
}
 */

/**
 * Test inflation behavior, starting with an empty map
TEST(costmap, test9){
  std::vector<unsigned char> mapData;
  for(unsigned int i=0; i< GRID_WIDTH; i++){
    for(unsigned int j = 0; j < GRID_HEIGHT; j++){
      mapData.push_back(0);
    }
  }

  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, mapData, RESOLUTION, THRESHOLD, MAX_Z, MAX_Z, MAX_Z, 
		ROBOT_RADIUS * 3, ROBOT_RADIUS * 2, ROBOT_RADIUS);

  // There should be no occupied cells
  std::vector<unsigned int> ids;
  map.getOccupiedCellDataIndexList(ids);
  ASSERT_EQ(ids.size(), 0);

  // Add an obstacle at 5,5
  sensor_msgs::PointCloud c0;
  c0.set_points_size(1);
  c0.points[0].x = 5;
  c0.points[0].y = 5;
  c0.points[0].z = MAX_Z;

  std::vector<unsigned int> updates;
  map.updateDynamicObstacles(c0, updates);

  ASSERT_EQ(updates.size(), 45);

  map.getOccupiedCellDataIndexList(ids);
  ASSERT_EQ(ids.size(), 5);

  // Update again - should see no change
  map.updateDynamicObstacles(c0, updates);
  ASSERT_EQ(updates.size(), 0);
}
 */

/**
 * Test for the cost map accessor
TEST(costmap, test10){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, MAP_10_BY_10, RESOLUTION, THRESHOLD, MAX_Z, MAX_Z, ROBOT_RADIUS);

  // A window around a robot in the top left
  CostMapAccessor ma(map, 5, 0, 0);
  double wx, wy;

  // Origin
  ma.MC_WC(0, 0, wx, wy);
  ASSERT_EQ(wx, 0.5);
  ASSERT_EQ(wy, 0.5);


  // Max in x and y
  ma.updateForRobotPosition(9.5, 9.5);
  ma.MC_WC(0, 0, wx, wy);
  ASSERT_EQ(wx, 4.5);
  ASSERT_EQ(wy, 4.5);

  // Off the map in x - assume it ignores the change
  ma.updateForRobotPosition(10.5, 9.5);
  ma.MC_WC(0, 0, wx, wy);
  ASSERT_EQ(wx, 4.5);
  ASSERT_EQ(wy, 4.5);

  // Off the map in y - assume it ignores the change
  ma.updateForRobotPosition(9.5, 10.5);
  ma.MC_WC(0, 0, wx, wy);
  ASSERT_EQ(wx, 4.5);
  ASSERT_EQ(wy, 4.5);

  ASSERT_EQ(map.getCircumscribedCostLowerBound(), ma.getCircumscribedCostLowerBound());
}
 */

/**
 * Test for ray tracing free space

TEST(costmap, test11){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, MAP_10_BY_10, RESOLUTION, THRESHOLD, MAX_Z * 2, MAX_Z, MAX_Z, ROBOT_RADIUS, 0, 0, 1, 100.0, 100.0, 7.0);

  // The initial position will be <0,0> by default. So if we add an obstacle at 9,9, we would expect cells
  // <0, 0> thru <8, 8> to be free
  sensor_msgs::PointCloud c0;
  c0.set_points_size(1);
  c0.points[0].x = 9.5;
  c0.points[0].y = 9.5;
  c0.points[0].z = MAX_Z;
  std::vector<unsigned int> updates;
  map.updateDynamicObstacles(c0, updates);

  // Actual hit point and 3 cells along the diagonal. Note that neigbors are unchanged because they have higher cost in the static map (NO_INFORMATION).
  // I considered allowing the cost function to over-ride this case but we quickly find that the planner will plan through walls once it gets out of sensor range.
  // Note that this will not be the case when we persist the changes to the static map more aggressively since we will retain high cost obstacle data that 
  // has not been ray tarced thru. If that is the case, this update count would change to 6
  ASSERT_EQ(updates.size(), 5);


  // many cells will have been switched to free space along the diagonal except for those inflated in the update
  unsigned char test[9]= {0, 0, 0, 126, 126, 0, 0, 126, 254};
  for(unsigned int i=0; i < 9; i++)
    ASSERT_EQ(map.getCost(i, i), test[i]);
}



bool compareFiles(std::string a, std::string b) {
  FILE *test, *compare;

  test = fopen(a.c_str(), "r");
  compare = fopen(b.c_str(), "r");
  if (!test) {
    ROS_INFO("Could not open: %s\n", a.c_str());
    return false;
  }
  if (!compare) {
    ROS_INFO("Could not open: %s\n", b.c_str());
    return false;
  }

  while (1) {
    char out = getc(test);
    char com = getc(compare);
    if (out == EOF && com == EOF) {
      fclose(test);
      fclose(compare);
      return true;
    }
    if (out != com) {
      fclose(test);
      fclose(compare);
      return false;
    }
    if (out == EOF || com == EOF) {
      fclose(test);
      fclose(compare);
      return false;
    }
  }

}
 */


/**
 * Test for saving.

TEST(costmap, test16){
  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, MAP_10_BY_10, RESOLUTION, THRESHOLD, MAX_Z * 2, MAX_Z, MAX_Z, ROBOT_RADIUS, 0, 0, 1, 100.0, 100.0);
  map.saveText("testmap.txt");
  map.saveBinary("testmap.bin");
  map.savePgm("testmap.pgm");
  
  ASSERT_EQ(compareFiles("testmap.txt", "testmap_compare.txt"), true);
  ASSERT_EQ(compareFiles("testmap.bin", "testmap_compare.bin"), true);
  ASSERT_EQ(compareFiles("testmap.pgm", "testmap_compare.pgm"), true);
}
 */






/**
 * Within a certian radius of the robot, the cost map most propagate obstacles. This
 * is to avoid a case where a hit on a far obstacle clears inscribed radius around a
 * near one.

TEST(costmap, test17){
  const unsigned char MAP_HALL_CHAR[10 * 10] = {
    0,   0, 0,   0,   0, 0,   0, 0, 0, 0,
    254, 0, 0,   0,   0, 0,   0, 0, 0, 0,
    0,   0, 0,   0,   0, 0,   0, 0, 0, 0,
    0,   0, 0,   254, 0, 0,   0, 0, 0, 0,
    0,   0, 0,   0,   0, 0,   0, 0, 0, 0,
    0,   0, 0,   0,   0, 0,   0, 0, 0, 0,
    0,   0, 0,   0,   0, 254, 0, 0, 0, 0,
    0,   0, 0,   0,   0, 254, 0, 0, 0, 0,
    0,   0, 0,   0,   0, 0,   0, 0, 0, 0,
    0,   0, 0,   0,   0, 0,   0, 0, 0, 0,
  };
  std::vector<unsigned char> MAP_HALL;
  for (int i = 0; i < 10 * 10; i++) {
    MAP_HALL.push_back(MAP_HALL_CHAR[i]);
  }


  Costmap2D map(GRID_WIDTH, GRID_HEIGHT, MAP_HALL, RESOLUTION, 
		THRESHOLD, MAX_Z * 2, MAX_Z, MAX_Z, ROBOT_RADIUS, 0, 0, 1, 100.0, 100.0, 7.0);



  //Add a dynamic obstacle
  std::vector<sensor_msgs::PointCloud*> cv2;
  sensor_msgs::PointCloud c2;
  cv2.push_back(&c2);
  c2.set_points_size(3);
  //Dynamic obstacle that raytaces.
  c2.points[0].x = 7.0;
  c2.points[0].y = 8.0;
  c2.points[0].z = 1.0;
  //Dynamic obstacle that should not be raytraced the
  //first update, but should on the second.
  c2.points[1].x = 3.0;
  c2.points[1].y = 4.0;
  c2.points[1].z = 1.0;
  //Dynamic obstacle that should not be erased.
  c2.points[2].x = 6.0;
  c2.points[2].y = 3.0;
  c2.points[2].z = 1.0;
  map.updateDynamicObstacles(0.0, 0.0, cv2);


  const unsigned char MAP_HALL_CHAR_TEST[10 * 10] = { 
    126, 254, 126,   0,   0,   0,   0,   0,   0,   0,
      0, 126,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0, 126,   0,   0,   0,   0,   0,
      0,   0,   0, 126, 254, 126,   0,   0,   0,   0,
      0,   0,   0,   0, 126,   0,   0, 126,   0,   0,
      0,   0,   0, 126,   0,   0, 126, 254, 126,   0,
      0,   0, 126, 254, 126,   0,   0, 126, 126,   0,
      0,   0,   0, 126,   0,   0,   0, 126, 254, 126,
      0,   0,   0,   0,   0,   0,   0,   0, 126,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  };

  
  for (int i = 0; i < 10 * 10; i++) {
    ASSERT_EQ(map.getCost(i / 10, i % 10), MAP_HALL_CHAR_TEST[i]);
  }

  std::vector<sensor_msgs::PointCloud*> cv;
  sensor_msgs::PointCloud c;
  cv.push_back(&c);
  c.set_points_size(1);
  //Dynamic obstacle that raytaces the one at (3.0, 4.0).
  c.points[0].x = 4.0;
  c.points[0].y = 5.0;
  c.points[0].z = 1.0;
  map.updateDynamicObstacles(0.0, 0.0, cv);


  const unsigned char MAP_HALL_CHAR_TEST2[10 * 10] = { 
    126, 254, 126,   0,   0,   0,   0,   0,   0,   0,
      0, 126,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0, 126,   0,   0,   0,   0,
      0,   0,   0,   0, 126, 254, 126, 126,   0,   0,
      0,   0,   0, 126,   0, 126, 126, 254, 126,   0,
      0,   0, 126, 254, 126,   0,   0, 126, 126,   0,
      0,   0,   0, 126,   0,   0,   0, 126, 254, 126,
      0,   0,   0,   0,   0,   0,   0,   0, 126,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
  };

  
  for (int i = 0; i < 10 * 10; i++) {
    ASSERT_EQ(map.getCost(i / 10, i % 10), MAP_HALL_CHAR_TEST2[i]);
  }


}

 */


int main(int argc, char** argv){
  for(unsigned int i = 0; i< GRID_WIDTH * GRID_HEIGHT; i++){
    EMPTY_10_BY_10.push_back(0);
    MAP_10_BY_10.push_back(MAP_10_BY_10_CHAR[i]);
  }

  for(unsigned int i = 0; i< 100 * 100; i++)
    EMPTY_100_BY_100.push_back(0);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
