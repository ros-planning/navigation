/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Badger Technologies LLC
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
 * Author: C. Andy Martin
 *********************************************************************/
#ifndef COSTMAP_3D_STATIC_2D_LAYER_3D_H_
#define COSTMAP_3D_STATIC_2D_LAYER_3D_H_

#include <memory>

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_3d/GenericPluginConfig.h>
#include <costmap_3d/costmap_layer_3d.h>

namespace costmap_3d
{

/*
 * Represent a 2D static map in a 3D layer.
 *
 * To put a 3D static map into a 3D layer, run an octomap_server and an
 * octomap_server_layer_3d. This layer is for use with the old 2D map_server.
 */
class Static2DLayer3D : public CostmapLayer3D
{
  using super = CostmapLayer3D;
public:
  Static2DLayer3D();
  virtual ~Static2DLayer3D();

  virtual void initialize(LayeredCostmap3D* parent, std::string name, tf2_ros::Buffer *tf);

  virtual void updateBounds(const geometry_msgs::Pose robot_pose,
                            const geometry_msgs::Point& rolled_min,
                            const geometry_msgs::Point& rolled_max,
                            Costmap3D* bounds_map);

  /** @brief Deactivate this layer, unsubscribing. */
  virtual void deactivate();

  /** @brief Activate this layer, subscribing. */
  virtual void activate();

  /** @brief On a reset, resubscribe as well to force-update our static map. */
  virtual void reset();

  virtual void matchSize(const geometry_msgs::Point& min, const geometry_msgs::Point& max, double resolution);

protected:
  // Override the default behavior to not throw away our map data, as it is static
  virtual void resetBoundingBoxUnlocked(Costmap3DIndex min, Costmap3DIndex max);

  Cost occupancyGridToCost(int8_t grid_cost);

  void occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr& grid);

  virtual void reconfigureCallback(costmap_3d::GenericPluginConfig &config, uint32_t level);

  virtual void subscribe();
  virtual void unsubscribe();

  ros::NodeHandle pnh_;
  std::shared_ptr<dynamic_reconfigure::Server<costmap_3d::GenericPluginConfig>> dsrv_;
  bool active_;
  std::string map_topic_;
  ros::Subscriber map_sub_;
  double height_;
  uint8_t lethal_cost_threshold_;
};

}  // namespace costmap_3d

#endif  // COSTMAP_3D_STATIC_2D_LAYER_3D_H_
