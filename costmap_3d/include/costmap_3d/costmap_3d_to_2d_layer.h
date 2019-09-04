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
#ifndef COSTMAP_3D_COSTMAP_3D_TO_2D_LAYER_H_
#define COSTMAP_3D_COSTMAP_3D_TO_2D_LAYER_H_

#include <memory>
#include <unordered_map>
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_3d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_3d/layered_costmap_3d.h>

namespace costmap_3d
{

class Costmap3DTo2DLayer : public costmap_2d::CostmapLayer
{
  using super = costmap_2d::CostmapLayer;
public:
  Costmap3DTo2DLayer();
  virtual ~Costmap3DTo2DLayer();
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void matchSize();

  /// Apply a 3D costmap update to our 2D costmap.
  virtual void updateFrom3D(LayeredCostmap3D* layered_costmap_3d, const Costmap3D& delta, const Costmap3D& bounds_map);

protected:
  unsigned char toCostmap2D(Cost value) const;
  void reconfigureCB(costmap_3d::GenericPluginConfig &config, uint32_t level);
  std::shared_ptr<dynamic_reconfigure::Server<costmap_3d::GenericPluginConfig>> dsrv_;
  bool use_maximum_;
  bool copy_full_map_;
  LayeredCostmap3D* layered_costmap_3d_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_3D_COSTMAP_3D_TO_2D_LAYER_H_
