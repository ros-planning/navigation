/*
 * Copyright (c) 2017 Intermodalics bvba
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
#ifndef MAP_GENERATOR_H
#define MAP_GENERATOR_H

/*
 * Author: Dominick Vanthienen
 */

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/SaveMapAction.h>

/**
 * @brief Map generation node.
 */
class MapGenerator
{
 public:
   MapGenerator(const std::string& default_map_name,
                const std::string& action_name);
   MapGenerator(const std::string& mapname);

   void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);
   void saveMapActionCallback(const move_base_msgs::SaveMapGoalConstPtr &goal);
   bool saved_map() {return saved_map_;}
 private:
   std::string mapname_;
   std::string action_name_;
   ros::Subscriber map_sub_;
   bool saved_map_;
   ros::NodeHandle n_;
   actionlib::SimpleActionServer<move_base_msgs::SaveMapAction> as_;
};

#endif
