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
#include <costmap_3d/layered_costmap_3d.h>

namespace costmap_3d
{

LayeredCostmap3D::LayeredCostmap3D(costmap_2d::LayeredCostmap* layered_costmap_2d)
    : lock_layers_(this),
      layered_costmap_2d_(layered_costmap_2d),
      size_changed_(false),
      resolution_(0.0)
{
  min_point_.x = -std::numeric_limits<double>::max();
  min_point_.y = -std::numeric_limits<double>::max();
  min_point_.z = -std::numeric_limits<double>::max();
  max_point_.x = std::numeric_limits<double>::max();
  max_point_.y = std::numeric_limits<double>::max();
  max_point_.z = std::numeric_limits<double>::max();
}

LayeredCostmap3D::~LayeredCostmap3D()
{
}

void LayeredCostmap3D::updateMap(geometry_msgs::Pose robot_pose)
{
  // Lock the master costmap
  std::lock_guard<LayeredCostmap3D> lock(*this);

  matchBoundsAndResolution();

  num_updates_++;

  if (!costmap_)
  {
    // We don't have a costmap to update!
    return;
  }

  Costmap3D bounds_map(costmap_->getResolution());

  // Calculate the current aabb (most useful for rolling maps).
  octomap::point3d aabb_min = toOctomapPoint(min_point_);
  octomap::point3d aabb_max = toOctomapPoint(max_point_);
  if (isRolling())
  {
    // Adjust the x/y based on the x/y of the robot's pose, as a rolling map
    // stays centered on the robot base in x/y (but not z).
    float robot_x = (float)robot_pose.position.x;
    float robot_y = (float)robot_pose.position.y;
    float aabb_width = aabb_max.x() - aabb_min.x();
    float aabb_height = aabb_max.y() - aabb_min.y();
    aabb_min.x() += robot_x - aabb_width/2.0f;
    aabb_min.y() += robot_y - aabb_height/2.0f;
    aabb_max.x() += robot_x - aabb_width/2.0f;
    aabb_max.y() += robot_y - aabb_height/2.0f;
  }
  const geometry_msgs::Point min_msg(fromOctomapPoint(aabb_min));
  const geometry_msgs::Point max_msg(fromOctomapPoint(aabb_max));

  ROS_DEBUG_STREAM("LayeredCostmap3D: updateMap: min point " << min_msg << " max point " << max_msg);

  {
    // lock all the costmap layers
    std::lock_guard<LockLayers> layers_lock(lock_layers_);

    for (auto plugin : plugins_)
    {
      plugin->updateBounds(robot_pose, min_msg, max_msg, &bounds_map);
    }

    if (size_changed_)
    {
      // Treat the entire map as changed to pull in any data that is
      // immediately ready in the plugins. Some plugins may represent data in
      // a way that is resolution-independent and may already have data on the
      // next updateCosts call. We could use a NULL value in setTreeValues for
      // the bounds map, but we must call updateBounds on every cycle to
      // preserve the API to the plugins, and must pass a valid bounds map
      // pointer, so make the bounds map represent the entire universe.
      bounds_map.setNodeValueAtDepth(Costmap3DIndex(), 0, LETHAL);
    }

    // Remove any out-of-bounds entries from the bounds map.
    // This will prevent layers from copying back any data that is
    // out-of-bounds below.
    bounds_map.deleteAABB(aabb_min, aabb_max, true);

    // Delete the current cells that are being updated.
    costmap_->setTreeValues(NULL, &bounds_map, false, true);

    // Update the costs of the regions of interest in the bounds_map
    for (auto plugin : plugins_)
    {
      plugin->updateCosts(bounds_map, costmap_.get());
    }
  }

  // Delete any out-of-bounds information from our costmap.
  // Do this after updateCosts to ensure no out-of-bounds information remains
  // from an errant layer that ignores the bounds map.
  // Set any cells that are deleted in the bounds map to use when publishing the update.
  costmap_->deleteAABB(aabb_min, aabb_max, true,
                       std::bind([](Costmap3D* tree, const Costmap3DIndex& key, unsigned int depth)
                                 {tree->setNodeValueAtDepth(key, depth, LETHAL);},
                                 &bounds_map,
                                 std::placeholders::_3,
                                 std::placeholders::_4));

  if (size_changed_)
  {
    // Any old values that were deleted during the size change will get
    // stuck if we do not publish a full map here. Just use the univeral
    // bounds for the bounds_map if the size has changed.
    bounds_map.setNodeValueAtDepth(Costmap3DIndex(), 0, LETHAL);
    size_changed_ = false;
  }

  // Calculate the update to publish
  Costmap3D map_delta(costmap_->getResolution());
  map_delta.setTreeValues(costmap_.get(), &bounds_map, false, false);

  std::lock_guard<std::mutex> callback_lock(update_complete_callbacks_mutex_);
  for (auto cb : update_complete_callbacks_)
  {
    cb.second(this, map_delta, bounds_map);
  }
}

void LayeredCostmap3D::registerUpdateCompleteCallback(const std::string callback_id, UpdateCompleteCallback cb)
{
  std::lock_guard<std::mutex> callback_lock(update_complete_callbacks_mutex_);
  update_complete_callbacks_[callback_id] = cb;
}

void LayeredCostmap3D::unregisterUpdateCompleteCallback(const std::string callback_id)
{
  std::lock_guard<std::mutex> callback_lock(update_complete_callbacks_mutex_);
  auto it = update_complete_callbacks_.find(callback_id);
  if (it != update_complete_callbacks_.end())
  {
    update_complete_callbacks_.erase(it);
  }
}

void LayeredCostmap3D::reset()
{
  std::lock_guard<LayeredCostmap3D> lock(*this);
  // Force a full publish on the next cycle
  size_changed_ = true;
  costmap_->clear();

  for (auto plugin : plugins_)
  {
    plugin->reset();
  }
}

void LayeredCostmap3D::activate()
{
  for (auto plugin : plugins_)
  {
    plugin->activate();
  }
}

void LayeredCostmap3D::deactivate()
{
  for (auto plugin : plugins_)
  {
    plugin->deactivate();
  }
}

bool LayeredCostmap3D::isCurrent() const
{
  for (auto plugin : plugins_)
  {
    if (!plugin->isCurrent())
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "LayeredCostmap3D layer " << plugin->getName() << " is not current");
      return false;
    }
  }
  return true;
}

Costmap3DConstPtr LayeredCostmap3D::getCostmap3D() const
{
  return costmap_;
}

bool LayeredCostmap3D::isRolling() const
{
  return layered_costmap_2d_->isRolling();
}

const std::vector<boost::shared_ptr<Layer3D>>& LayeredCostmap3D::getPlugins()
{
  return plugins_;
}

void LayeredCostmap3D::addPlugin(boost::shared_ptr<Layer3D> plugin)
{
  std::lock_guard<LayeredCostmap3D> lock(*this);
  plugins_.push_back(plugin);
}

void LayeredCostmap3D::lock()
{
  // To synchronize correctly with planners, we must use the same mutex
  // they do, which is the master 2D costmap lock.
  layered_costmap_2d_->getCostmap()->getMutex()->lock();
}

void LayeredCostmap3D::unlock()
{
  layered_costmap_2d_->getCostmap()->getMutex()->unlock();
}

double LayeredCostmap3D::getResolution() const
{
  return resolution_;
}

bool LayeredCostmap3D::isPoseInMap(geometry_msgs::Pose robot_pose)
{
  double limit = costmap_->getNodeSize(1);
  return (robot_pose.position.x < limit &&
          robot_pose.position.x > -limit &&
          robot_pose.position.y < limit &&
          robot_pose.position.y > -limit &&
          robot_pose.position.z < limit &&
          robot_pose.position.z > -limit);
}

void LayeredCostmap3D::getBounds(geometry_msgs::Point* min, geometry_msgs::Point* max)
{
  // keep the view of min/max consistent
  std::lock_guard<LayeredCostmap3D> lock(*this);
  *min = min_point_;
  *max = max_point_;
}

void LayeredCostmap3D::setBounds(double min_z, double max_z)
{
  std::lock_guard<LayeredCostmap3D> lock(*this);
  matchBoundsAndResolution(min_z, max_z);
}

void LayeredCostmap3D::matchBoundsAndResolution()
{
  // Just call matchBounds w/ the current Z bounds
  matchBoundsAndResolution(min_point_.z, max_point_.z);
}

void LayeredCostmap3D::matchBoundsAndResolution(double min_z, double max_z)
{
  bool change_size = false;
  geometry_msgs::Point min, max;
  min.z = min_z;
  max.z = max_z;
  // Get the x/y values from the 2D costmap
  if (isRolling())
  {
    // If we are rolling, set the origin to zero, as we will shift the limits
    // based on the base position ourselves in updateMap, since the octomap is
    // always originated at (0, 0, 0).
    min.x = 0.0;
    min.y = 0.0;
  }
  else
  {
    // If we are not rolling, use the 2D origin as the minimum.
    min.x = layered_costmap_2d_->getCostmap()->getOriginX();
    min.y = layered_costmap_2d_->getCostmap()->getOriginY();
  }
  max.x = min.x + layered_costmap_2d_->getCostmap()->getSizeInMetersX();
  max.y = min.y + layered_costmap_2d_->getCostmap()->getSizeInMetersY();
  if (min.x != min_point_.x ||
      min.y != min_point_.y ||
      min.z != min_point_.z ||
      max.x != max_point_.x ||
      max.y != max_point_.y ||
      max.z != max_point_.z)
  {
    ROS_DEBUG_STREAM("LayeredCostmap3D: min point " << min << " max point " << max);
    min_point_ = min;
    max_point_ = max;
    change_size = true;
  }
  double resolution = layered_costmap_2d_->getCostmap()->getResolution();
  if (resolution > 0.0 && resolution != resolution_)
  {
    resolution_ = resolution;
    change_size = true;
  }
  if (change_size)
  {
    sizeChange();
  }
}

void LayeredCostmap3D::sizeChange()
{
  size_changed_ = true;
  if (resolution_ > 0.0)
  {
    costmap_.reset(new Costmap3D(resolution_));
  }
  for (auto plugin : plugins_)
  {
    plugin->matchSize(min_point_, max_point_, resolution_);
  }
}

LayeredCostmap3D::LockLayers::LockLayers(LayeredCostmap3D* layered_costmap_3d)
  : layered_costmap_3d_(layered_costmap_3d)
{
}

void LayeredCostmap3D::LockLayers::lock()
{
  for (auto plugin : layered_costmap_3d_->getPlugins())
  {
    plugin->lock();
  }
}

void LayeredCostmap3D::LockLayers::unlock()
{
  for (auto plugin : layered_costmap_3d_->getPlugins())
  {
    plugin->unlock();
  }
}

}  // namespace costmap_3d
