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

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <boost/filesystem.hpp>

#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"

namespace
{
template<typename T>
T try_extract_param(const YAML::Node& doc, const std::string& param_name)
{
  try 
  {
    return doc[param_name].as<T>();
  }
  catch (YAML::Exception &) {
    ROS_ERROR("The map does not contain a %s tag or it is invalid.", param_name.c_str());
    exit(-1);
  }
}

using map_server::MapMode;
}

class MapServer
{

  public:
    /** Trivial constructor */
    MapServer(const std::string& fname)
    {    
      std::string frame_id;
      ros::NodeHandle private_nh("~");
      private_nh.param("frame_id", frame_id, std::string("map"));
      
      std::ifstream fin(fname.c_str());
      if (fin.fail()) {
        ROS_ERROR("Map_server could not open %s.", fname.c_str());
        exit(-1);
      }

      // The document loading process changed in yaml-cpp 0.5.
      YAML::Node doc = YAML::Load(fin);

      const double res = try_extract_param<double>(doc, "resolution");
      const bool negate = try_extract_param<int>(doc, "negate");
      const double occ_th = try_extract_param<double>(doc, "occupied_thresh");
      const double free_th = try_extract_param<double>(doc, "free_thresh");

      MapMode mode;
      try 
      {
        const auto mode_str = doc["mode"].as<std::string>();
        if(mode_str == "trinary")
        {
          mode = MapMode::TRINARY;
        }
        else if(mode_str == "scale")
        {
          mode = MapMode::SCALE;
        }
        else if(mode_str == "raw")
        {
          mode = MapMode::RAW;
        }
        else
        {
          ROS_ERROR("Invalid mode tag \"%s\".", mode_str.c_str());
          exit(-1);
        }
      } 
      catch (YAML::Exception &) 
      {
        ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
        mode = MapMode::TRINARY;
      }

      std::array<double, 3> origin;
      try 
      {
        origin[0] = doc["origin"][0].as<double>();
        origin[1] = doc["origin"][1].as<double>();
        origin[2] = doc["origin"][2].as<double>();
      } catch (YAML::InvalidScalar&) {
        ROS_ERROR("The map does not contain an origin tag or it is invalid.");
        exit(-1);
      }

      std::string mapfname = "";
      try {
        mapfname = doc["image"].as<std::string>();
        // TODO: make this path-handling more robust
        if(mapfname.size() == 0)
        {
          ROS_ERROR("The image tag cannot be an empty string.");
          exit(-1);
        }

        boost::filesystem::path mapfpath(mapfname);
        if (!mapfpath.is_absolute())
        {
          boost::filesystem::path dir(fname);
          dir = dir.parent_path();
          mapfpath = dir / mapfpath;
          mapfname = mapfpath.string();
        }
      } catch (YAML::InvalidScalar &) {
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        exit(-1);
      }

      ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
      try
      {
          map_server::loadMapFromFile(mapfname.c_str(), res, negate, occ_th, free_th, origin, mode, map_resp_);
      }
      catch (std::runtime_error& e)
      {
          ROS_ERROR("%s", e.what());
          exit(-1);
      }

      // To make sure get a consistent time in simulation
      ros::Time::waitForValid();
      map_resp_.map.info.map_load_time = ros::Time::now();
      map_resp_.map.header.frame_id = frame_id;
      map_resp_.map.header.stamp = ros::Time::now();
      ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
      meta_data_message_ = map_resp_.map.info;

      service = n.advertiseService("static_map", &MapServer::mapCallback, this);

      // Latched publisher for metadata
      metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
      metadata_pub.publish(meta_data_message_);

      // Latched publisher for data
      map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
      map_pub.publish( map_resp_.map );
    }

  private:
    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher metadata_pub;
    ros::ServiceServer service;

    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request& req,
                     nav_msgs::GetMap::Response& res )
    {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp_;
      ROS_INFO("Sending map");

      return true;
    }

    /** The map data is cached here, to be sent out to service callers
     */
    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;

};

int main(int argc, char **argv)
{
  setvbuf(stdout, NULL, _IOLBF, 4096);
  ros::init(argc, argv, "map_server", ros::init_options::AnonymousName);
  if(argc != 2)
  {
    ROS_ERROR("\nUSAGE: map_server <map.yaml>\n  map.yaml: map description file\n");
    exit(-1);
  }
  std::string fname(argv[1]);

  try
  {
    MapServer ms(fname);
    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}

