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
#ifndef COSTMAP_3D_COSTMAP_3D_TO_2D_LAYER_3D_H_
#define COSTMAP_3D_COSTMAP_3D_TO_2D_LAYER_3D_H_

#include <costmap_3d/costmap_3d.h>
#include <costmap_3d/layered_costmap_3d.h>
#include <string>

namespace costmap_3d
{

/**
 * Interface to glue a 3D costmap to a 2D costmap layer.
 * Note that both layers must be in the same Costmap3DROS to work.
 */
class Costmap3DTo2DLayer3D : public Layer3D
{
  using super = Layer3D;
public:
  Costmap3DTo2DLayer3D();
  virtual ~Costmap3DTo2DLayer3D();

  virtual void updateBounds(const geometry_msgs::Pose robot_pose,
                            const geometry_msgs::Point& rolled_min,
                            const geometry_msgs::Point& rolled_max,
                            Costmap3D* bounds_map) {}

  virtual void updateCosts(const Costmap3D& bounds_map, Costmap3D* master_map) {}

  virtual void initialize(LayeredCostmap3D* parent, std::string name, tf2_ros::Buffer *tf);
  virtual void deactivate();
  virtual void activate();
  virtual void reset() {}
  virtual void resetBoundingBox(geometry_msgs::Point min_point, geometry_msgs::Point max_point) {}
  virtual bool isCurrent() const {return true;}
  virtual void matchSize(const geometry_msgs::Point& min, const geometry_msgs::Point& max, double resolution) {}
};

}  // namespace costmap_3d

#endif  // COSTMAP_3D_COSTMAP_3D_TO_2D_LAYER_3D_H_
