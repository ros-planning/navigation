/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *                2017 6 River Systems
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
 *         Daniel Grieneisen
 *********************************************************************/
#ifndef COSTMAP_2D_STATIC_LAYER_WITH_INFLATION_H_
#define COSTMAP_2D_STATIC_LAYER_WITH_INFLATION_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/voronoi_inflation_layer.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>
#include <pluginlib/class_loader.h>

#include <srslib_timing/MasterTimingDataRecorder.hpp>

namespace costmap_2d
{

/**
 * Provides a costmap layer for a static map.  In addition, it inflates the map so
 * that reinflation is not necessary.
 */
class StaticLayerWithInflation : public CostmapLayer
{
public:
  StaticLayerWithInflation();
  virtual ~StaticLayerWithInflation();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void matchSize();

  virtual void onFootprintChanged();

  virtual LayerType getLayerType() {
    return LayerType::STATIC_IMPASSIBLE;
    // return impassible_ ? LayerType::STATIC_IMPASSIBLE : LayerType::STATIC_KEEPOUT;
  } 


  virtual std::shared_ptr<std::vector<double>> getDistancesFromStaticMap() override {
    return inflation_layer_->getDistancesFromStaticMap();
  }

  virtual std::shared_ptr<std::vector<int>> getAnglesFromStaticMap() override {
    return inflation_layer_->getAnglesFromStaticMap();
  }


  virtual double getDistanceFromStaticMap(double px, double py) override;
  virtual int getAngleFromStaticMap(double px, double py) override;

private:
  /**
   * @brief  Callback to update the costmap's map from the map_server
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);
  void incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update);
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  /**
   * Updates the stored costmap with the data from the stored static map.
   */
  void updateCostmapFromStaticMap();

  unsigned char interpretValue(unsigned char value);

  bool getTransform(tf::StampedTransform& transform,
    std::string from_frame_id, std::string to_frame_id);

  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string map_frame_;  /// @brief frame that map is located in
  bool subscribe_to_updates_;
  bool map_received_;
  bool has_updated_data_;
  unsigned int x_, y_, width_, height_;
  bool track_unknown_space_;
  bool use_maximum_;
  bool first_map_only_;      ///< @brief Store the first static map and reuse it on reinitializing
  bool trinary_costmap_;
  ros::Subscriber map_sub_, map_update_sub_;

  unsigned char lethal_threshold_, unknown_cost_value_;

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  pluginlib::ClassLoader<Layer> plugin_loader_;
  boost::shared_ptr<Layer> inflation_layer_;
  std::string inflation_layer_type_;
  boost::shared_ptr<Layer> secondary_inflation_layer_;
  std::string secondary_inflation_layer_type_;
  bool needs_reinflation_;

  bool impassible_;

  unsigned char* static_map_;

  // Add timing data recorder
  srs::MasterTimingDataRecorder timingDataRecorder_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_STATIC_LAYER_WITH_INFLATION_H_
