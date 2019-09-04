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
#include <costmap_3d/costmap_3d_to_2d_layer_3d.h>
#include <costmap_3d/costmap_3d_to_2d_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_3d::Costmap3DTo2DLayer3D, costmap_3d::Layer3D)

namespace costmap_3d
{

Costmap3DTo2DLayer3D::Costmap3DTo2DLayer3D()
{
  // This layer is always current
  current_ = true;
}

Costmap3DTo2DLayer3D::~Costmap3DTo2DLayer3D()
{
  deactivate();
}

void Costmap3DTo2DLayer3D::initialize(LayeredCostmap3D* parent, std::string name, tf2_ros::Buffer *tf)
{
  super::initialize(parent, name, tf);
  activate();
}

void Costmap3DTo2DLayer3D::activate()
{
  // Find any costmap 3D to 2D layers and connect them to the 3D costmap
  costmap_2d::LayeredCostmap* layered_costmap_2d = layered_costmap_3d_->getLayeredCostmap2D();
  costmap_3d::Costmap3DTo2DLayer* layer_2d;
  for (auto plugin_2d : *layered_costmap_2d->getPlugins())
  {
    layer_2d = dynamic_cast<costmap_3d::Costmap3DTo2DLayer*>(plugin_2d.get());
    if (layer_2d != nullptr)
    {
      layered_costmap_3d_->registerUpdateCompleteCallback(
          layer_2d->getName(),
          std::bind(&Costmap3DTo2DLayer::updateFrom3D,
                    layer_2d,
                    std::placeholders::_1,
                    std::placeholders::_2,
                    std::placeholders::_3));
    }
  }
}

void Costmap3DTo2DLayer3D::deactivate()
{
  // Find any costmap 3D to 2D layers and disconnect them from the 3D costmap
  costmap_2d::LayeredCostmap* layered_costmap_2d = layered_costmap_3d_->getLayeredCostmap2D();
  costmap_3d::Costmap3DTo2DLayer* layer_2d;
  for (auto plugin_2d : *layered_costmap_2d->getPlugins())
  {
    layer_2d = dynamic_cast<costmap_3d::Costmap3DTo2DLayer*>(plugin_2d.get());
    if (layer_2d != nullptr)
    {
      layered_costmap_3d_->unregisterUpdateCompleteCallback(layer_2d->getName());
    }
  }
}

}  // namespace costmap_3d
