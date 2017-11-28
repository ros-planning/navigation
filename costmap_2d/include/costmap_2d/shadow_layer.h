/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017 6 River Systems.
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
 *   * Neither the name of 6 River Systems. nor the names of its
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
 * Author: Daniel Grieneisen
 *********************************************************************/
#ifndef COSTMAP_2D_SHADOW_LAYER_H_
#define COSTMAP_2D_SHADOW_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/PointCloud.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/ShadowPluginConfig.h>

namespace costmap_2d
{

class ShadowLayer : public CostmapLayer
{
public:
  ShadowLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }

  virtual ~ShadowLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void reset();

  virtual std::shared_ptr<std::vector<geometry_msgs::Point>> getShadowedObjects() {
    return shadowed_objects_;
  }

  virtual LayerType getLayerType() {
    return LayerType::SHADOW;
  } 


protected:
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

  void generateMaxShadowObservation(double robot_x, double robot_y, double robot_yaw,
                                          double* min_x, double* min_y, double* max_x, double* max_y);
  void calculateShadows(costmap_2d::Costmap2D& master_grid);
  void raytraceShadowObservation(costmap_2d::Costmap2D& master_grid);
  void calculateShadowedObjects();
  bool checkForShadowedObjectAtIndex(unsigned int idx);
  geometry_msgs::Point createPointFromIndex(unsigned int idx);
  void publishShadowObjects();


  class ShadowMarker
  {
  public:
    ShadowMarker(unsigned char* costmap, unsigned char* master_grid,
      std::shared_ptr<std::vector<unsigned int>> shadow_points,
      unsigned char visible_value, unsigned char shadow_value,
      unsigned char obstacle_value) :
        costmap_(costmap), 
        master_grid_(master_grid),
        shadow_points_(shadow_points),
        visible_value_(visible_value), 
        shadow_value_(shadow_value),
        obstacle_value_(obstacle_value),
        passed_obstacle_(false)
    {
    }
    inline void operator()(unsigned int offset)
    {
      if (master_grid_[offset] == costmap_2d::LETHAL_OBSTACLE)
      {
        costmap_[offset] = obstacle_value_;
        passed_obstacle_ = true;
      }
      else if (passed_obstacle_)
      {
        if (costmap_[offset] != shadow_value_)
        {
          costmap_[offset] = shadow_value_;
          // Gather all the shadow points
          shadow_points_->push_back(offset);
        }
      }
      else
      {
        costmap_[offset] = visible_value_;
      }
    }
  private:
    unsigned char* costmap_;
    unsigned char* master_grid_;
    std::shared_ptr<std::vector<unsigned int>> shadow_points_;
    unsigned char visible_value_;
    unsigned char shadow_value_;
    unsigned char obstacle_value_;
    bool passed_obstacle_;
  };

  std::shared_ptr<Observation> shadow_observation_ 
    = std::make_shared<Observation>();
  std::shared_ptr<std::vector<unsigned int>> shadowed_points_ 
    = std::make_shared<std::vector<unsigned int>>();
  std::shared_ptr<std::vector<geometry_msgs::Point>> shadowed_objects_ 
    = std::make_shared<std::vector<geometry_msgs::Point>>();

  double shadow_scan_range_ = 3.0;
  double shadow_scan_angular_resolution_ = 0.005;
  double shadow_scan_half_angle_ = M_PI / 2;
  double min_shadow_size_ = 0.1;
  unsigned int shadow_half_width_ = 2;
  double sensor_x_offset_ = 0.5;

  bool publish_shadow_objects_ = false;

  static const unsigned char OBSTACLE = 240;
  static const unsigned char SHADOW = 130;
  static const unsigned char VISIBLE = 55;
  static const unsigned char UNKNOWN = 0;

  std::string global_frame_;  ///< @brief The global frame for the costmap

  bool rolling_window_;
  dynamic_reconfigure::Server<costmap_2d::ShadowPluginConfig> *dsrv_;

  ros::Publisher visualization_publisher_;

private:
  void reconfigureCB(costmap_2d::ShadowPluginConfig &config, uint32_t level);
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_SHADOW_LAYER_H_
