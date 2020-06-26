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
#include "costmap_3d/static_2d_layer_3d.h"

#include <pluginlib/class_list_macros.h>

#include "costmap_3d/costmap_3d.h"

PLUGINLIB_EXPORT_CLASS(costmap_3d::Static2DLayer3D, costmap_3d::Layer3D)

namespace costmap_3d
{

Static2DLayer3D::Static2DLayer3D()
{
  current_ = false;
}

Static2DLayer3D::~Static2DLayer3D()
{
}

void Static2DLayer3D::initialize(LayeredCostmap3D* parent, std::string name, tf2_ros::Buffer *tf)
{
  super::initialize(parent, name, tf);

  // Get names of topics and servers from parameter server
  pnh_ = ros::NodeHandle("~/" + name);

  map_topic_ = pnh_.param("map_topic", std::string("map"));
  height_ = pnh_.param("height", 0.0);
  lethal_cost_threshold_ = static_cast<int8_t>(pnh_.param<int>("lethal_cost_threshold", 100));

  ROS_INFO_STREAM("Static2DLayer3D " << name << ": initializing");
  ROS_INFO_STREAM("  map_topic: " << map_topic_);
  ROS_INFO_STREAM("  height: " << height_);
  ROS_INFO_STREAM("  lethal_cost_threshold: " << static_cast<int>(lethal_cost_threshold_));

  dsrv_.reset(new dynamic_reconfigure::Server<costmap_3d::GenericPluginConfig>(pnh_));
  dsrv_->setCallback(std::bind(&Static2DLayer3D::reconfigureCallback, this,
                               std::placeholders::_1, std::placeholders::_2));

  activate();
}

void Static2DLayer3D::reconfigureCallback(costmap_3d::GenericPluginConfig &config, uint32_t level)
{
  std::lock_guard<Layer3D> lock(*this);
  if (enabled_ && !config.enabled)
  {
    if (costmap_) touch(*costmap_);
    // disable last, so touch does something
    enabled_ = false;
  }
  else if (!enabled_ && config.enabled)
  {
    // enable first, so touch does something
    enabled_ = true;
    if (costmap_) touch(*costmap_);
  }
  combination_method_ = config.combination_method;
}

void Static2DLayer3D::updateBounds(
    const geometry_msgs::Pose robot_pose,
    const geometry_msgs::Point& rolled_min,
    const geometry_msgs::Point& rolled_max,
    Costmap3D* bounds_map)
{
  std::lock_guard<Layer3D> lock(*this);
  super::updateBounds(robot_pose, rolled_min, rolled_max, bounds_map);
}


void Static2DLayer3D::deactivate()
{
  std::lock_guard<Layer3D> lock(*this);
  active_ = false;
  unsubscribe();
  super::deactivate();
  if (costmap_)
  {
    // Ensure the next costmap update will not have any old information from
    // this layer.
    touch(*costmap_);
    costmap_->clear();
  }
  current_ = false;
}

void Static2DLayer3D::activate()
{
  std::lock_guard<Layer3D> lock(*this);
  super::activate();
  subscribe();
  active_ = true;
}

void Static2DLayer3D::reset()
{
  // On a reset, resubscribe to get a new copy of the static map.
  // This is not strictly necessary, but provides an interface to force a reset.
  // Resubscribing is also what the costmap_2d static layer does on reset.
  std::lock_guard<Layer3D> lock(*this);
  unsubscribe();
  if (costmap_)
  {
    // Ensure the next costmap update will not have any old information from
    // this layer.
    touch(*costmap_);
    costmap_->clear();
  }
  current_ = false;
  subscribe();
}

void Static2DLayer3D::resetBoundingBoxUnlocked(Costmap3DIndex min, Costmap3DIndex max)
{
  // Do nothing when the bounding box reset is called.
  // Simply keep our static map intact.
  // There is no need to call touch, as this layer has no changes.
}

void Static2DLayer3D::subscribe()
{
  map_sub_ = pnh_.subscribe<nav_msgs::OccupancyGrid>(map_topic_, 1, std::bind(
      &Static2DLayer3D::occupancyGridCallback, this, std::placeholders::_1));
}

void Static2DLayer3D::unsubscribe()
{
  map_sub_.shutdown();
}

void Static2DLayer3D::matchSize(const geometry_msgs::Point& min, const geometry_msgs::Point& max, double resolution)
{
  bool need_resubscribe = false;
  if (!costmap_ || resolution != costmap_->getResolution())
  {
    // Resolution change, need to request full message.
    // Do this by re-subscribing to the map topic.
    // Don't resubscribe until after the costmap is created to avoid racing.
    need_resubscribe = true;
  }
  super::matchSize(min, max, resolution);
  if (need_resubscribe)
  {
    unsubscribe();
    subscribe();
  }
}

Cost Static2DLayer3D::occupancyGridToCost(int8_t grid_cost)
{
  if (grid_cost < 0)
    return UNKNOWN;
  else if (grid_cost >= lethal_cost_threshold_)
    return LETHAL;
  return FREE;
}

void Static2DLayer3D::occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr& grid)
{
  if (!costmap_)
  {
    // There is no costmap yet. Do nothing.
    // When the costmap is created in matchSize, we will resubscribe.
    return;
  }

  double resolution = grid->info.resolution;
  unsigned int size_x = grid->info.width;
  unsigned int size_y = grid->info.height;
  double origin_x = grid->info.origin.position.x;
  double origin_y = grid->info.origin.position.y;

  // The map topic must be in the same frame with the same resolution
  if (grid->header.frame_id != layered_costmap_3d_->getGlobalFrameID())
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Static2DLayer3D " << name_ << ": received map in frame " <<
                             grid->header.frame_id << " but global frame is " <<
                             layered_costmap_3d_->getGlobalFrameID() << ", ignoring");
    return;
  }

  if (std::abs(resolution - costmap_->getResolution()) > 1e-6)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Static2DLayer3D " << name_ << ": received map with resolution " <<
                             grid->info.resolution << " but costmap resolution is " <<
                             costmap_->getResolution() << ", ignoring");
    return;
  }

  costmap_2d::LayeredCostmap* layered_costmap = layered_costmap_3d_->getLayeredCostmap2D();
  if (!layered_costmap->isRolling())
  {
    // For a non-rolling costmap, resize entire layered 2d costmap if size or origin do not match.
    // This matches the behavior of the costmap_2d static layer plugin.
    costmap_2d::Costmap2D* master = layered_costmap->getCostmap();
    if (master->getSizeInCellsX() != size_x ||
        master->getSizeInCellsY() != size_y ||
        master->getOriginX() != origin_x ||
        master->getOriginY() != origin_y ||
        !layered_costmap->isSizeLocked())
    {
      // Update the size of the layered costmap (and all layers, including this one)
      // Note: do *not* own the master costmap lock here, as it is grabbed inside resizeMap
      // Note: do *not* hold the layer lock here, as we will grab it in our own matchSize
      layered_costmap->resizeMap(size_x, size_y, resolution, origin_x, origin_y, true);
    }
  }

  std::lock_guard<Layer3D> lock(*this);
  // Only process this map if we are active
  if (active_)
  {
    octomap::key_type key_z = costmap_->coordToKey(height_);
    Costmap3D new_cells(costmap_->getResolution());
    for (unsigned int j = 0; j < size_y; ++j)
    {
      double y = origin_y + j * resolution + resolution / 2.0;
      octomap::key_type key_y = costmap_->coordToKey(y);
      for (unsigned int i = 0; i < size_x; ++i)
      {
        double x = origin_x + i * resolution + resolution / 2.0;
        octomap::key_type key_x = costmap_->coordToKey(x);
        Costmap3DIndex key(key_x, key_y, key_z);
        Cost pt_cost = occupancyGridToCost(grid->data[j * size_x + i]);
        if (pt_cost != UNKNOWN)
        {
          new_cells.setNodeValue(key, pt_cost);
        }
      }
    }

    // Copy the old costmap state into erase cells
    Costmap3D erase_cells(*costmap_);
    // Delete any cells from erase_cells that are in the new cloud.
    erase_cells.setTreeValues(NULL, &new_cells, false, true);

    // Add any cells in new_cells to unchanged_cells that match our current costmap state.
    Costmap3D unchanged_cells(costmap_->getResolution());
    for(Costmap3D::leaf_iterator it=new_cells.begin_leafs(), end=new_cells.end_leafs(); it != end; ++it)
    {
      const Costmap3DIndex key(it.getKey());
      const Cost new_cost = it->getValue();
      const Costmap3D::NodeType* const old_node = costmap_->search(key);
      if (old_node && new_cost == old_node->getValue())
      {
	unchanged_cells.setNodeValue(key, new_cost);
      }
    }

    // Remove the unchanged cells from the new cells to avoid over-touching the costmap
    new_cells.setTreeValues(NULL, &unchanged_cells, false, true);

    // Make the changes
    eraseCells(erase_cells);
    markAndClearCells(new_cells);
  }
  // Finally up to date!
  current_ = true;
}

}  // namespace costmap_3d
