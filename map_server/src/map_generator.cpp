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
#include "map_server/map_generator.h"
#include <cstdio>
#include "ros/console.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

#ifndef NO_LIBPNGPP
// Use png++ for saving PNG files
#include <png++/png.hpp>
#endif

/**
 * @brief Map generation node.
 */
MapGenerator::MapGenerator(const std::string& mapname) : mapname_(mapname), saved_map_(false)
{
  // Map for filetype validity checking and filename extensions
  types_["png"] = ".png";
  types_["pgm"] = ".pgm";

  ros::NodeHandle n;
  ROS_INFO("Waiting for the map");
  map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
}

void MapGenerator::savePNG(const std::string& mapdatafile,
                           const nav_msgs::OccupancyGridConstPtr& map)
{

#ifdef NO_LIBPNGPP
  ROS_INFO("PNG is unsupported without the PNG++ library.");
  std::string mapdatafile_pgm = mapname_ + ".pgm";
  ROS_INFO("Writing map occupancy data to %s instead", mapdatafile_pgm.c_str());
  savePGM(mapdatafile, map);
#else
  png::image< png::rgb_pixel > image(map->info.width, map->info.height);
  for (unsigned int y = 0; y < map->info.height; y++) {
    for (unsigned int x = 0; x < map->info.width; x++) {
      unsigned int i = x + (map->info.height - y - 1) * map->info.width;
      if (map->data[i] == 0) { //occ [0,0.1)
        image[y][x] = png::rgb_pixel(254, 254, 254);
      } else if (map->data[i] == +100) { //occ (0.65,1]
        image[y][x] = png::rgb_pixel(0, 0, 0);
      } else { //occ [0.1,0.65]
        image[y][x] = png::rgb_pixel(205, 205, 205);
      }
    }
  }

  image.write(mapdatafile.c_str());
#endif
}


void MapGenerator::savePGM(const std::string& mapdatafile,
                           const nav_msgs::OccupancyGridConstPtr& map)
{
  FILE* out = fopen(mapdatafile.c_str(), "w");
  if (!out)
  {
    ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
    return;
  }

  fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
          map->info.resolution, map->info.width, map->info.height);
  for (unsigned int y = 0; y < map->info.height; y++) {
    for (unsigned int x = 0; x < map->info.width; x++) {
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
}

void MapGenerator::mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
  ROS_INFO("Received a %d X %d map @ %.3f m/pix",
           map->info.width,
           map->info.height,
           map->info.resolution);

  // Retrieve type to save map as
  std::string map_type;
  ros::param::param<std::string>("/map_saver/save_file_type", map_type, "pgm");
  // Transform string to lowercase
  std::transform(map_type.begin(), map_type.end(), map_type.begin(), ::tolower);
  std::string map_ext;

  if (types_.find(map_type) != types_.end()) {
    map_ext = types_[map_type];
  } else {
    ROS_INFO("Could not find a supported filetype matching given parameter '%s'",
             map_type.c_str());
    map_ext = ".pgm";
  }

  std::string mapdatafile = mapname_ + map_ext;
  ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());

  if (map_ext == ".png") {
    savePNG(mapdatafile, map);
  } else {
    savePGM(mapdatafile, map);
  }

  std::string mapmetadatafile = mapname_ + ".yaml";
  ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
  FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

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