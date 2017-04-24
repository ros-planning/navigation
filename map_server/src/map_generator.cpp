/*
 * map_saver
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

#include <cstdio>
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include <map_server/map_generator.h>
#include <move_base_msgs/SaveMapAction.h>

using namespace std;

MapGenerator::MapGenerator(const std::string& default_map_name,
                           const std::string& action_name) :
    mapname_(default_map_name),
    action_name_(action_name),
    saved_map_(false),
    as_(n_, action_name_,
        boost::bind(&MapGenerator::saveMapActionCallback, this, _1),
        false)
{}

MapGenerator::MapGenerator(const std::string& mapname) :
    mapname_(mapname),
    saved_map_(false),
    // TODO(@dvanthienen) How to get rid of this action server instance?
    as_(n_, "map_saver",
        boost::bind(&MapGenerator::saveMapActionCallback, this, _1),
        false)
{
  ROS_INFO("Waiting for the map");
  map_sub_ = n_.subscribe("map", 1, &MapGenerator::mapCallback, this);
}

void MapGenerator::mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
  ROS_INFO("Received a %d X %d map @ %.3f m/pix",
           map->info.width,
           map->info.height,
           map->info.resolution);


  std::string mapdatafile = mapname_ + ".pgm";
  ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
  FILE* out = fopen(mapdatafile.c_str(), "w");
  if (!out)
  {
    ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
    return;
  }

  fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
          map->info.resolution, map->info.width, map->info.height);
  for(unsigned int y = 0; y < map->info.height; y++) {
    for(unsigned int x = 0; x < map->info.width; x++) {
      unsigned int i = x + (map->info.height - y - 1) * map->info.width;
      if (map->data[i] == 0) { //occ [0,0.1)
        fputc(254, out);
      } else if (map->data[i] == +100) { //occ (0.65,1]
        fputc(000, out);
      } else { //occ [0.1,0.65]
        fputc(205, out);
      }
    }
  }

  fclose(out);


  std::string mapmetadatafile = mapname_ + ".yaml";
  ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
  FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


  /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196

   */

  geometry_msgs::Quaternion orientation = map->info.origin.orientation;
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);

  fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
          mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

  fclose(yaml);

  ROS_INFO("Done\n");
  saved_map_ = true;
}

void MapGenerator::saveMapActionCallback(
    const move_base_msgs::SaveMapGoalConstPtr &goal)
{
  mapname_ = goal->map_name;
  move_base_msgs::SaveMapResult result;
  result.response = -1;
  // helper variables
  ros::Rate r(10);
  bool success = true;

  map_sub_ = n_.subscribe("map", 1, &MapGenerator::mapCallback, this);

  while (!saved_map_) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("Map generator Preempted");
      // set the action state to preempted
      as_.setPreempted();
      success = false;
      break;
    }
    r.sleep();
  }
  if (success) {
    n_.shutdown();
    result.response = 0;
  }
  as_.setSucceeded(result, "Map saved.");
  saved_map_ = false;
}
