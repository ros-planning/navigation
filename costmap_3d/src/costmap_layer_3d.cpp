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
#include <costmap_3d/costmap_layer_3d.h>
#include <costmap_3d/GenericPluginConfig.h>

namespace costmap_3d
{

CostmapLayer3D::CostmapLayer3D() : super()
{
}

CostmapLayer3D::~CostmapLayer3D()
{
}

void CostmapLayer3D::updateBounds(const geometry_msgs::Pose robot_pose,
                                  const geometry_msgs::Point& rolled_min,
                                  const geometry_msgs::Point& rolled_max,
                                  Costmap3D* bounds_map)
{
  if (costmap_)
  {
    // Don't track deleting out-of-bounds nodes.
    // Note: if we ever publish a debug map, we should probably track them for
    // efficient delta publishing.
    costmap_->deleteAABB(toOctomapPoint(rolled_min),
                         toOctomapPoint(rolled_max),
                         true);

  }
  if (changed_cells_)
  {
    bounds_map->setTreeValues(changed_cells_.get(), true, false);

    // If we were to publish a map for debug, it would be here.

    // Clear out changed cells now that we are finished with it.
    changed_cells_->clear();
  }
}

void CostmapLayer3D::updateCosts(const Costmap3D& bounds_map, Costmap3D* master_map)
{
  if (costmap_ && enabled_)
  {
    switch (combination_method_)
    {
      case GenericPlugin_Overwrite:
        master_map->setTreeValues(static_cast<const Costmap3D*>(costmap_.get()), &bounds_map, false, false);
        break;
      case GenericPlugin_Maximum:
        master_map->setTreeValues(static_cast<const Costmap3D*>(costmap_.get()), &bounds_map, true, false);
        break;
      default:
      case GenericPlugin_Nothing:
        // In case this was a mistake, give info once that we are using NOTHING
        ROS_INFO_STREAM_ONCE("costmap layer '" << name_ << "' combination method NOTHING");
        break;
    }
  }
}

#if 0
void CostmapLayer3D::initialize(LayeredCostmap3D* parent, std::string name, tf2_ros::Buffer *tf)
{
  super::initialize(parent, name, tf);
  // Note that the master costmap's resolution is not set yet.
  // Our setResolution method will be called later once it is set.
}
#endif

void CostmapLayer3D::deactivate()
{
  std::lock_guard<Layer3D> lock(*this);

  // Ensure the next costmap update will not have any information stored in
  // this layer.
  if (costmap_) touch(*costmap_);
}

void CostmapLayer3D::activate()
{
  std::lock_guard<Layer3D> lock(*this);

  // Ensure the next costmap update will have any information stored in
  // this layer.
  if (costmap_) touch(*costmap_);
}

void CostmapLayer3D::reset()
{
  std::lock_guard<Layer3D> lock(*this);
  if (costmap_)
  {
    // Ensure the next costmap update will not have any old information from
    // this layer.
    touch(*costmap_);
    costmap_->clear();
  }
}

void CostmapLayer3D::resetBoundingBox(Costmap3DIndex min, Costmap3DIndex max)
{
  std::lock_guard<Layer3D> lock(*this);
  if (costmap_ && changed_cells_)
  {
    resetBoundingBoxUnlocked(min, max);
  }
};

void CostmapLayer3D::resetBoundingBox(geometry_msgs::Point min_point, geometry_msgs::Point max_point)
{
  std::lock_guard<Layer3D> lock(*this);
  if (costmap_ && changed_cells_)
  {
    Costmap3DIndex min_key, max_key;
    costmap_->coordToKeyClamped(toOctomapPoint(min_point), min_key);
    costmap_->coordToKeyClamped(toOctomapPoint(max_point), max_key);
    resetBoundingBoxUnlocked(min_key, max_key);
  }
}

void CostmapLayer3D::matchSize(const geometry_msgs::Point& min, const geometry_msgs::Point& max, double resolution)
{
  std::lock_guard<Layer3D> lock(*this);
  if ((!costmap_ && resolution > 0.0) || (costmap_ && resolution != costmap_->getResolution()))
  {
    changed_cells_.reset(new Costmap3D(resolution));
    costmap_.reset(new Costmap3D(resolution));
  }
  // If a child class octomap has limits that can be set, such as the one in
  // OctomapServer, they could be set here
}

void CostmapLayer3D::resetBoundingBoxUnlocked(Costmap3DIndex min, Costmap3DIndex max)
{
  // Delete the AABB from the tree, and add any deleted leafs to the changed cells.
  costmap_->deleteAABB(min, max, false,
                       std::bind([](Costmap3D* tree, const Costmap3DIndex& key, unsigned int depth)
                                 {tree->setNodeValueAtDepth(key, depth, LETHAL);},
                                 changed_cells_.get(),
                                 std::placeholders::_3,
                                 std::placeholders::_4));
}

void CostmapLayer3D::touch(const octomap::OcTree& touch_map)
{
  if (changed_cells_)
  {
    changed_cells_->setTreeValues(&touch_map, false, false,
                                  [](const Costmap3D::NodeType*, Costmap3D::NodeType* node, bool, const octomap::OcTreeKey&, unsigned int)
                                  {node->setValue(LETHAL);});
  }
}

void CostmapLayer3D::touch(const Costmap3DIndex& key)
{
  touchKeyAtDepth(key);
}

void CostmapLayer3D::touchKeyAtDepth(const octomap::OcTreeKey& key, unsigned int depth)
{
  if (changed_cells_)
  {
    changed_cells_->setNodeValueAtDepth(key, depth, LETHAL);
  }
}

void CostmapLayer3D::updateCells(octomap::OcTree* value_map, octomap::OcTree* bounds_map,
                                 Cost occupied_threshold)
{
  if (costmap_ && value_map && bounds_map)
  {
    value_map->setTreeDepth(costmap_->getTreeDepth());
    bounds_map->setTreeDepth(costmap_->getTreeDepth());
    updateCells(*value_map, *bounds_map, occupied_threshold);
  }
}

void CostmapLayer3D::updateCells(const octomap::OcTree& value_map, const octomap::OcTree& bounds_map,
                                 Cost occupied_threshold)
{
  if (costmap_)
  {
    // Erase the map in the bounds region then set to either LETHAL or FREE
    // based on the given threshold applied to the value map values.
    costmap_->setTreeValues(&value_map, &bounds_map, false, true,
                            std::bind([](Cost thresh, const Costmap3D::NodeType* value_node, Costmap3D::NodeType* node)
                                      {node->setValue(value_node->getValue() >= thresh ? LETHAL : FREE);},
                                      occupied_threshold,
                                      std::placeholders::_1,
                                      std::placeholders::_2));
  }
  touch(bounds_map);
}

void CostmapLayer3D::updateCell(const geometry_msgs::Point& point, bool mark)
{
  Cost cost = mark ? LETHAL : FREE;
  setCellCost(point, cost);
}

void CostmapLayer3D::updateCell(const octomap::OcTreeKey& key, bool mark)
{
  Cost cost = mark ? LETHAL : FREE;
  setCellCost(key, cost);
}

void CostmapLayer3D::markCell(const geometry_msgs::Point& point)
{
  setCellCost(point, LETHAL);
}

void CostmapLayer3D::markCell(const octomap::OcTreeKey& key)
{
  setCellCost(key, LETHAL);
}

void CostmapLayer3D::clearCell(const geometry_msgs::Point& point)
{
  setCellCost(point, FREE);
}

void CostmapLayer3D::clearCell(const octomap::OcTreeKey& key)
{
  setCellCost(key, FREE);
}

void CostmapLayer3D::markAndClearCells(const Costmap3D& map)
{
  if (costmap_)
  {
    costmap_->setTreeValues(&map);
    touch(map);
  }
}

void CostmapLayer3D::eraseCell(const geometry_msgs::Point& point)
{
  setCellCost(point, UNKNOWN);
}

void CostmapLayer3D::eraseCell(const octomap::OcTreeKey& key)
{
  setCellCost(key, UNKNOWN);
}

void CostmapLayer3D::eraseCells(const Costmap3D& map)
{
  if (costmap_)
  {
    costmap_->setTreeValues(NULL, &map, false, true);
    touch(map);
  }
}

void CostmapLayer3D::setCellCostAtDepth(const octomap::OcTreeKey& key, Cost cost, unsigned int depth)
{
  if (costmap_)
  {
    if (cost >= FREE)
    {
      costmap_->setNodeValueAtDepth(key, depth, cost);
    }
    else
    {
      // A cost not greater or equal to free is unknown (either too negative
      // or its a NAN). Don't bother saving unknown, just delete it.
      costmap_->deleteNode(key, depth);
    }
    touchKeyAtDepth(key, depth);
  }
}

void CostmapLayer3D::setCellCost(const octomap::OcTreeKey& key, Cost cost)
{
  setCellCostAtDepth(key, cost);
}

void CostmapLayer3D::setCellCost(const geometry_msgs::Point& point, Cost cost)
{
  if (costmap_)
  {
    Costmap3DIndex key;
    if (costmap_->coordToKeyChecked(toOctomapPoint(point), key))
      setCellCost(key, cost);
  }
}

}  // namespace costmap_3d
