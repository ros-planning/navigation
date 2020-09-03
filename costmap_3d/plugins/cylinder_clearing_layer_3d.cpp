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
#include <costmap_3d/cylinder_clearing_layer_3d.h>

#include <memory>
#include <cmath>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_3d::CylinderClearingLayer3D, costmap_3d::Layer3D)

namespace costmap_3d
{

void CylinderClearingLayer3D::initialize(LayeredCostmap3D* parent, std::string name, tf2_ros::Buffer *tf)
{
  super::initialize(parent, name, tf);

  // Get names of topics and servers from parameter server
  pnh_ = ros::NodeHandle("~/" + name);

  ROS_INFO_STREAM("CylinderClearingLayer3D " << name << ": initializing");

  dsrv_.reset(new dynamic_reconfigure::Server<costmap_3d::CylinderClearingPluginConfig>(pnh_));
  dsrv_->setCallback(std::bind(&CylinderClearingLayer3D::reconfigureCallback, this,
                               std::placeholders::_1, std::placeholders::_2));

  activate();
}

void CylinderClearingLayer3D::reconfigureCallback(costmap_3d::CylinderClearingPluginConfig &config, uint32_t level)
{
  std::lock_guard<Layer3D> lock(*this);
  enabled_ = config.enabled;
  radius_ = config.radius;
  radius_ = std::abs(radius_);
  ROS_INFO_STREAM("  radius: " << radius_);
  if (radius_ == 0.0)
  {
    ROS_WARN_STREAM("Radius is zero, this layer is effectively disabled");
  }
}

void CylinderClearingLayer3D::updateBounds(
    const geometry_msgs::Pose robot_pose,
    const geometry_msgs::Point& rolled_min,
    const geometry_msgs::Point& rolled_max,
    Costmap3D* bounds_map)
{
  std::lock_guard<Layer3D> lock(*this);
  if (clearing_bounds_map_)
  {
    // Always add the last clearing bounds into this bounds map so upstream
    // layers can copy data back in we removed in previous cycle(s). Our clearing
    // bounds map each cycle only includes what was inside the cylinder, so this
    // is not too much added burden.
    bounds_map->setTreeValues(clearing_bounds_map_.get(), true, false);
  }
  if (active_ && enabled_)
  {
    clearing_bounds_map_ = std::make_shared<Costmap3D>(bounds_map->getResolution());

    // Find bounding box of cylinder
    Costmap3DIndex min_index;
    Costmap3DIndex max_index;
    bounds_map->coordToKeyClamped(
        robot_pose.position.x - radius_,
        robot_pose.position.y - radius_,
        min_z_,
        min_index);
    bounds_map->coordToKeyClamped(
        robot_pose.position.x + radius_,
        robot_pose.position.y + radius_,
        max_z_,
        max_index);

    // Loop over 2D space to find octomap cells inside the cylinder footprint
    Costmap3DIndex index;
    double robot_x = robot_pose.position.x;
    double robot_y = robot_pose.position.y;
    double r_squared = radius_ * radius_;
    for (index[0] = min_index[0]; index[0] <= max_index[0]; ++index[0])
    {
      for (index[1] = min_index[1]; index[1] <= max_index[1]; ++index[1])
      {
        // Just consider the center of the cell for the radius check.
        const double x = bounds_map->keyToCoord(index[0]) - robot_x;
        const double y = bounds_map->keyToCoord(index[1]) - robot_y;
        if (x * x + y * y <= r_squared)
        {
          // This cell is in the cylinder, so add all z-values to the clearing bounds
          for (index[2] = min_index[2]; index[2] <= max_index[2]; ++index[2])
          {
            clearing_bounds_map_->setNodeValue(index, LETHAL);
          }
        }
      }
    }
    // Be sure everything we want to delete this cycle is also in the bounds map
    bounds_map->setTreeValues(clearing_bounds_map_.get(), true, false);
  }
  else
  {
    // Either inactive or disabled, clear out the clearing bounds map as we are
    // no longer deleting other layer's data.
    clearing_bounds_map_.reset();
  }
}

void CylinderClearingLayer3D::updateCosts(const Costmap3D& bounds_map, Costmap3D* master_map)
{
  std::lock_guard<Layer3D> lock(*this);
  if (active_ && enabled_ && clearing_bounds_map_)
  {
    // Clear (erase) any upstream map data in the clearing bounds map
    master_map->setTreeValues(nullptr, clearing_bounds_map_.get(), false, true);
  }
}

void CylinderClearingLayer3D::deactivate()
{
  std::lock_guard<Layer3D> lock(*this);
  active_ = false;
}

void CylinderClearingLayer3D::activate()
{
  std::lock_guard<Layer3D> lock(*this);
  active_ = true;
}

void CylinderClearingLayer3D::reset()
{
  std::lock_guard<Layer3D> lock(*this);
  // No longer need to remember previous clearing bounds, as a reset will force
  // all data from other layers to be re-copied.
  clearing_bounds_map_.reset();
}

void CylinderClearingLayer3D::resetBoundingBox(
    geometry_msgs::Point min_point,
    geometry_msgs::Point max_point)
{
  std::lock_guard<Layer3D> lock(*this);
  if (clearing_bounds_map_)
  {
    // No longer need to remember which cells we have deleted inside this
    // bounding box, as layers above will have to recopy their data.
    Costmap3DIndex min_index;
    Costmap3DIndex max_index;
    clearing_bounds_map_->coordToKeyClamped(
        min_point.x,
        min_point.y,
        min_point.z,
        min_index);
    clearing_bounds_map_->coordToKeyClamped(
        max_point.x,
        max_point.y,
        max_point.z,
        max_index);
    clearing_bounds_map_->deleteAABB(min_index, max_index);
  }
}

void CylinderClearingLayer3D::matchSize(
    const geometry_msgs::Point& min,
    const geometry_msgs::Point& max,
    double resolution)
{
  std::lock_guard<Layer3D> lock(*this);
  min_z_ = min.z;
  max_z_ = max.z;
  if (clearing_bounds_map_ &&
      resolution > 0.0 &&
      resolution != clearing_bounds_map_->getResolution())
  {
    // On a resolution change, all layers have to re-copy their data. There is
    // no need to remember what cells we have deleted in the past.
    clearing_bounds_map_.reset();
  }
}

}  // namespace costmap_3d
