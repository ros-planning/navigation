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
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

using namespace std;

/**
 * @brief Map generation node.
 */
class MapGenerator
{

  public:
    MapGenerator(const std::string& mapname) : mapname_(mapname), saved_map_(false)
    {
      ros::NodeHandle p_nh("~");
      p_nh.param("free_threshold", free_threshold_, 1);
      p_nh.param("occupied_threshold", occupied_threshold_, 65);
      p_nh.param("binary_threshold", binary_threshold_, 60);

      ROS_INFO("Using free threshold %d and occupied threshold %d for trinary map.",
        free_threshold_, occupied_threshold_);
      ROS_INFO("Using threshold %d binary map.", binary_threshold_);

      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
      ROS_INFO("Received a %d X %d map @ %.3f m/pix",
               map->info.width,
               map->info.height,
               map->info.resolution);

      // Open each of the 3 output files
      std::string trinaryMapdatafile = mapname_ + "-trinary.pgm";
      ROS_INFO("Writing trinary map occupancy data to %s", trinaryMapdatafile.c_str());
      FILE* outTrinary = fopen(trinaryMapdatafile.c_str(), "w");
      if (!outTrinary)
      {
        ROS_ERROR("Couldn't save map file to %s", trinaryMapdatafile.c_str());
        return;
      }

      std::string binaryMapdatafile = mapname_ + "-binary.pgm";
      ROS_INFO("Writing binary map occupancy data to %s", binaryMapdatafile.c_str());
      FILE* outBinary = fopen(binaryMapdatafile.c_str(), "w");
      if (!outBinary)
      {
        ROS_ERROR("Couldn't save binary map file to %s", binaryMapdatafile.c_str());
        return;
      }

      std::string probabilityMapdatafile = mapname_ + "-probability.pgm";
      ROS_INFO("Writing probability map occupancy data to %s", probabilityMapdatafile.c_str());
      FILE* outProbability = fopen(probabilityMapdatafile.c_str(), "w");
      if (!outProbability)
      {
        ROS_ERROR("Couldn't save probability map file to %s", probabilityMapdatafile.c_str());
        return;
      }

      fprintf(outTrinary, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);

      fprintf(outBinary, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);

      fprintf(outProbability, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);

      for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {
          unsigned int i = x + (map->info.height - y - 1) * map->info.width;

          // Write trinary
          if (map->data[i] >= 0 && map->data[i] <= free_threshold_) { //occ [0,0.1)
            fputc(255, outTrinary);
          } else if (map->data[i] > occupied_threshold_ && map->data[i] <= +100) { //occ (0.65,1]
            fputc(000, outTrinary);
          } else { //occ [0.1,0.65]
            fputc(205, outTrinary);
          }

          // Write binary
          if (map->data[i] >= 0 && map->data[i] < binary_threshold_) {
            fputc(255, outBinary);
          } else if (map->data[i] >= binary_threshold_ && map->data[i] <= +100) { //occ (0.65,1]
            fputc(000, outBinary);
          } else { // mark unknown as clear
            fputc(255, outBinary);
          }

          // Write probability
          if (map->data[i] >= 0 && map->data[i] <= +100) {
            fputc((unsigned char) floor(255 - map->data[i] * 2.55), outProbability);
          } else { // mark unknown as 50%
            fputc(128, outProbability);
          }

        }
      }

      fclose(outTrinary);
      fclose(outBinary);
      fclose(outProbability);


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

      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: %f \nfree_thresh: %f\n\n",
              trinaryMapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw,
              (float) occupied_threshold_ * 0.01, (float) free_threshold_ * 0.01);

      fclose(yaml);

      ROS_INFO("Done\n");
      saved_map_ = true;
    }

    std::string mapname_;
    ros::Subscriber map_sub_;
    bool saved_map_;

    int free_threshold_;
    int occupied_threshold_;
    int binary_threshold_;
};

#define USAGE "Usage: \n" \
              "  map_saver -h\n"\
              "  map_saver [-f <mapname>] [ROS remapping args]"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_saver");
  std::string mapname = "map";

  for(int i=1; i<argc; i++)
  {
    if(!strcmp(argv[i], "-h"))
    {
      puts(USAGE);
      return 0;
    }
    else if(!strcmp(argv[i], "-f"))
    {
      if(++i < argc)
        mapname = argv[i];
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else
    {
      puts(USAGE);
      return 1;
    }
  }

  MapGenerator mg(mapname);

  while(!mg.saved_map_ && ros::ok())
    ros::spinOnce();

  return 0;
}


