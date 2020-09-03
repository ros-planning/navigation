/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Badger Technologies LLC
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
#ifndef COSTMAP_3D_CYLINDER_CLEARING_LAYER_3D_H_
#define COSTMAP_3D_CYLINDER_CLEARING_LAYER_3D_H_

#include <memory>
#include <dynamic_reconfigure/server.h>
#include <costmap_3d/CylinderClearingPluginConfig.h>
#include <costmap_3d/layer_3d.h>
#include <tf2_ros/transform_listener.h>

namespace costmap_3d
{

class CylinderClearingLayer3D : public Layer3D
{
  using super = Layer3D;
public:
  CylinderClearingLayer3D() { current_ = true; }
  virtual ~CylinderClearingLayer3D() = default;

  virtual void initialize(LayeredCostmap3D* parent, std::string name, tf2_ros::Buffer *tf);

  virtual void updateBounds(const geometry_msgs::Pose robot_pose,
                            const geometry_msgs::Point& rolled_min,
                            const geometry_msgs::Point& rolled_max,
                            Costmap3D* bounds_map);

  virtual void updateCosts(const Costmap3D& bounds_map, Costmap3D* master_map);

  /** @brief Deactivate this layer, no longer clearing.
   */
  virtual void deactivate();

  /** @brief Activate this layer, clearing the cylinder. */
  virtual void activate();

  virtual void reset();

  virtual void resetBoundingBox(geometry_msgs::Point min_point, geometry_msgs::Point max_point);

  virtual void matchSize(const geometry_msgs::Point& min, const geometry_msgs::Point& max, double resolution);

protected:
  virtual void reconfigureCallback(costmap_3d::CylinderClearingPluginConfig &config, uint32_t level);

  ros::NodeHandle pnh_;
  std::shared_ptr<dynamic_reconfigure::Server<costmap_3d::CylinderClearingPluginConfig>> dsrv_;
  bool enabled_ = false;
  bool active_ = false;
  double radius_ = 0.0;
  double min_z_ = 0.0;
  double max_z_ = 2.0;
  Costmap3DPtr clearing_bounds_map_;
};

}  // namespace costmap_3d

#endif  // COSTMAP_3D_POINT_CLOUD_LAYER_3D_H_
