/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef FOOTPRINT_COSTMAP_PLUGIN_H_
#define FOOTPRINT_COSTMAP_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/plugin_ros.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

namespace common_costmap_plugins
{
class FootprintCostmapPlugin : public costmap_2d::CostmapPluginROS
{
public:
  FootprintCostmapPlugin()
  {
    layered_costmap_ = NULL;
  }

  void initialize(costmap_2d::LayeredCostmap* costmap, std::string name);
  void update_bounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                     double* max_y);
  void update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  void activate()
  {
  }
  void deactivate()
  {
  }

private:
  void footprint_cb(const geometry_msgs::Polygon& footprint);
  ros::Subscriber footprint_sub_;
  bool got_footprint_;
  geometry_msgs::Polygon footprint_spec_;
  geometry_msgs::PolygonStamped footprint_;
  void publishFootprint();
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  ros::Publisher footprint_pub_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif

