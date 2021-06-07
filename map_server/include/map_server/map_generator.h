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

#ifndef MAP_SERVER_MAP_GENERATOR_H
#define MAP_SERVER_MAP_GENERATOR_H

#include <cstdio>
#include <malloc.h>
#include <string>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "png.h"

#include "map_server/map_mode.h"

using namespace std;

namespace map_server
{

/**
 * @brief Map generation node.
 */
    class MapGenerator
    {

    public:
        MapGenerator(const std::string& mapname, int threshold_occupied, int threshold_free, MapMode map_mode)
                : mapname_(mapname), saved_map_(false), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free), map_mode_(map_mode)
        {
            ros::NodeHandle n;
            ROS_INFO("Waiting for the map");
            map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
        }

        bool Saved()
        {
            return saved_map_;
        }

    private:

        void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
        {
            ROS_INFO("Received a %d X %d map @ %.3f m/pix",
                     map->info.width,
                     map->info.height,
                     map->info.resolution);
            std::string mapdatafile;

            if (map_mode_ == TRINARY)
            {
                mapdatafile = mapname_ + ".pgm";
                ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
                FILE* out = fopen(mapdatafile.c_str(), "w");
                if (!out)
                {
                    ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
                    return;
                }
                fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
                        map->info.resolution, map->info.width, map->info.height);
                for(unsigned int y = 0; y < map->info.height; y++) {
                    for(unsigned int x = 0; x < map->info.width; x++) {
                        unsigned int i = x + (map->info.height - y - 1) * map->info.width;
                        if (map->data[i] >= 0 && map->data[i] <= threshold_free_) { // [0,free)
                            fputc(254, out);
                        } else if (map->data[i] >= threshold_occupied_) { // (occ,255]
                            fputc(000, out);
                        } else { //occ [0.25,0.65]
                            fputc(205, out);
                        }
                    }
                }
                fclose(out);
            }
            else if (map_mode_ == SCALE)
            {
                mapdatafile = mapname_ + ".png";
                ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());

                FILE *fp = fopen(mapdatafile.c_str(), "wb");
                png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
                png_infop info_ptr = png_create_info_struct(png_ptr);
                png_init_io(png_ptr, fp);
                png_set_IHDR(png_ptr, info_ptr, map->info.width, map->info.height,
                             8, PNG_COLOR_TYPE_RGB_ALPHA, PNG_INTERLACE_NONE,
                             PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
                png_write_info(png_ptr, info_ptr);
                png_bytep row = (png_bytep) malloc(4 * map->info.width * sizeof(png_byte));

                for(unsigned int y = 0; y < map->info.height; y++) {
                    for(unsigned int x = 0; x < map->info.width; x++) {
                        unsigned int i = x + (map->info.height - y - 1) * map->info.width;
                        if (map->data[i] >= 0 && map->data[i] <= threshold_free_) {   // free space
                            row[x * 4] = row[x * 4 + 1] = row[x * 4 + 2] = 254;
                            row[x * 4 + 3] = 255;
                        } else if (map->data[i] >= threshold_occupied_) {             // occupied space
                            row[x * 4] = row[x * 4 + 1] = row[x * 4 + 2] = 0;
                            row[x * 4 + 3] = 255;
                        } else if (map->data[i] > threshold_free_ && map->data[i] < threshold_occupied_){   // scale space
                            double ratio = (map->data[i] - threshold_free_) * 1.0 / (threshold_occupied_ - threshold_free_);
                            row[x * 4] = row[x * 4 + 1] = row[x * 4 + 2] = (png_byte)((1 - ratio) * 254);
                            row[x * 4 + 3] = 255;
                        } else if (map->data[i] == -1) {  // unknown space
                            row[x * 4] = row[x * 4 + 1] = row[x * 4 + 2] = 205;
                            row[x * 4 + 3] = 0;
                        } else {
                            ROS_ERROR("Unsupported map data %d", map->data[i]);
                            exit(-1);
                        }
                    }
                    png_write_row(png_ptr, row);
                }
                png_write_end(png_ptr, NULL);
                fclose(fp);
            }

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
            tf2::Matrix3x3 mat(tf2::Quaternion(
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w
            ));
            double yaw, pitch, roll;
            mat.getEulerYPR(yaw, pitch, roll);
            if (map_mode_ == TRINARY)
            {
                fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
                        mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);
            } else if (map_mode_ == SCALE) {
                fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: %f\nfree_thresh: %f\nmode: scale\n",
                        mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw, threshold_occupied_ / 100.0, threshold_free_ / 100.0);
            }
            fclose(yaml);

            ROS_INFO("Done\n");
            saved_map_ = true;
        }

        std::string mapname_;
        ros::Subscriber map_sub_;
        bool saved_map_;
        int threshold_occupied_;
        int threshold_free_;
        MapMode map_mode_;
    };

}

#endif