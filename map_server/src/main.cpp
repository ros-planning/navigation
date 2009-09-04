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

/* Author: Brian Gerkey */

/** @defgroup map_server map_server

@b map_server is a ROS node that reads an occupancy grid map from an image
file and offers up the map via a ROS service.

The map_server converts color values into ternary occupancy values:
free, occupied, and unknown.  Thresholds are used to divide the three
categories (see below for setting these via parameters).  Whiter pixels
are free, blacker pixels are occupied, and pixels in between are unknown.
Color and grayscale images are accepted, but most maps are gray (even
though they may be stored as if in color).

@todo Establish a standard for storing maps, with metadata (origin,
resolution, color thresholds, etc.) in the same file.  Perhaps we can use
PNGs with comments for this purpose. Then rewrite this node to use said
standard.

<hr>

@section usage Usage
@verbatim
map_server <map> <resolution>
           map: image file to load
           resolution: map resolution [meters/pixel]
@endverbatim

@par Example

@verbatim
map_server mymap.png 0.1
@endverbatim

<hr>

@section topic ROS topics

- None

@section services ROS services

Offers (name/type):
- @b "static_map"/nav_msgs::GetMap : Retrieve the map via this service

@section parameters ROS parameters

- @b ~negate (boolean) : If true, the black/white semantics are reversed (i.e., black is free, white is occupied), default: false

@subsection thresholds Occupancy thresholds
When comparing to the threshold parameters, the occupancy probability of an
image pixel is computed as follows: occ = (255 - color_avg) / 255.0,
where color_avg is the 8-bit value that results from averaging over
all channels.  E.g., if the image is 24-bit color, a pixel with the
color 0x0a0a0a has an probability of 0.96, which is very occupied.
The color 0xeeeeee yields 0.07, which is very unoccupied.  The default
values below usually work well with maps produced by slam_gmapping.

Above ~occupied_thresh is occupied; below ~free_thresh is free; in between
is unknown.

- @b ~occupied_thresh (float) : Minimum probability to be considered
occupied, default 0.65
- @b ~free_thresh (float) : Maximum probability to be considered
free, default 0.196


*/



#define USAGE "USAGE: map_server <map> <resolution>\n"\
              "         map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"

class MapServer
{
  public:
    /** Trivial constructor */
    MapServer(const std::string& fname, double res)
    {
      int negate;
      double occ_th, free_th;

      n.param("~negate", negate, 0);
      printf("negate: %d\n", negate);
      n.param("~occupied_thresh", occ_th, 0.65);
      n.param("~free_thresh", free_th, 0.196);

      ROS_INFO("Loading map from image \"%s\"", fname.c_str());
      map_server::loadMapFromFile(&map_resp_,fname.c_str(),res,negate,occ_th,free_th);
      map_resp_.map.info.map_load_time = ros::Time::now();
      ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
      meta_data_message_ = map_resp_.map.info;

      service = n.advertiseService("static_map", &MapServer::mapCallback, this);
      pub = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1,
                                                 boost::bind(&MapServer::metadataSubscriptionCallback, *this, _1));
    }

  private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::ServiceServer service;

    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
    {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp_;
      ROS_INFO("Sending map");
      return true;
    }

    /** The map response is cached here, to be sent out to service callers
     */
    nav_msgs::GetMap::Response map_resp_;

    void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
    {
      pub.publish( meta_data_message_ );
    }

    nav_msgs::MapMetaData meta_data_message_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server", ros::init_options::AnonymousName);
  if(argc != 3)
  {
    puts(USAGE);
    exit(-1);
  }
  std::string fname(argv[1]);
  double res = atof(argv[2]);

  try
  {
    MapServer ms(fname, res);
    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}

