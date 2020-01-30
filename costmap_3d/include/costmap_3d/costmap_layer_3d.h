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
#ifndef COSTMAP_3D_COSTMAP_LAYER_3D_H_
#define COSTMAP_3D_COSTMAP_LAYER_3D_H_

#include <limits>
#include <costmap_3d/costmap_3d.h>
#include <costmap_3d/layered_costmap_3d.h>
#include <costmap_3d/layer_3d.h>
#include <geometry_msgs/Point.h>

namespace costmap_3d
{

class LayeredCostmap3D;

class CostmapLayer3D : public Layer3D
{
  using super = Layer3D;
public:
  CostmapLayer3D();
  virtual ~CostmapLayer3D();

  /** @brief Add nodes to bounds_map for any cells that have changed
   *
   * The point of updateBounds is to accumulate all the space that has
   * changed in all layers since the last costmap update. For convenience, the
   * changed space is represented as lethal occupied (even though all we are
   * storing here is really a bit that means we should update this space).
   * This layer should call updateNode(node, LETHAL) for every leaf
   * node that has changed since the last update.
   *
   * Also, for rolling maps, layers should check for data that is outside the
   * current rolled min/max that is passed in. For non-rolling maps, layers
   * may assume the bounds only change when matchSize() is called.
   * Note that layers do not need to put out-of-bounds cells into the
   * bounds_map. The layered costmap will delete out-of-bounds cells from the
   * master map and manage tracking their removal for publishing. It will not
   * hurt anything to put the cells in the bounds map, it is just unnecessary.
   */
  virtual void updateBounds(const geometry_msgs::Pose robot_pose,
                            const geometry_msgs::Point& rolled_min,
                            const geometry_msgs::Point& rolled_max,
                            Costmap3D* bounds_map);

  /** @brief Copy the state of this costmap layer into the master layer using
   * the bounds_map to limit what portion is copied.
   *
   * The point of updateCosts is to efficiently update the master layer
   * using the update policy (update with max, or update with overwrite, or
   * true overwrite).
   *
   * The layered costmap using this plugin must have acquired the lock, and
   * must hold it across the call to updateBounds and updateCosts. This will
   * prevent our costmap from changing while copying to the master costmap.
   *
   * The default implementation will work fine as long as the internal costmap
   * is kept up to date. Be sure to prevent the internal costmap_ and
   * changed_cells_ from changing state between the call to updateBounds and
   * updateCosts.
   */
  virtual void updateCosts(const Costmap3D& bounds_map, Costmap3D* master_map);

  /** @brief Deactivate this layer.
   *
   * The default implementation will work fine if the layer is storing its
   * costmap in the costmap_ member.
   */
  virtual void deactivate();

  /** @brief Activate this layer. */
  virtual void activate();

  /** @brief Forget all memory as if restarting */
  virtual void reset();

  /**
   * @brief Forget old sensor data within the given axis-aligned bounding box.
   *
   * For plugins which have no persistent memory of their own but make use of
   * the costmap_, there is no need to override this method.
   */
  virtual void resetBoundingBox(Costmap3DIndex min, Costmap3DIndex max);

  virtual void resetBoundingBox(geometry_msgs::Point min_point, geometry_msgs::Point max_point);

  virtual void matchSize(const geometry_msgs::Point& min, const geometry_msgs::Point& max, double resolution);

protected:
  virtual void resetBoundingBoxUnlocked(Costmap3DIndex min, Costmap3DIndex max);

  /** @brief Touch all the cells present in the given OcTree.
   * This layer must be holding the lock to make this call. */
  virtual void touch(const octomap::OcTree& touch_map);

  /** @brief Touch the given index.
   * This layer must be holding the lock to make this call. */
  virtual void touch(const Costmap3DIndex& key);

  /** @brief Touch the given key at the given depth.
   * This layer must be holding the lock to make this call. */
  virtual void touchKeyAtDepth(const octomap::OcTreeKey& key,
                               unsigned int depth=std::numeric_limits<unsigned int>::max());

  /** @brief Update the cells in binary fashion from the given values and bounds.
   * Note: in this version, the value and bounds trees will have their depth
   * changed to match the costmap */
  virtual void updateCells(octomap::OcTree* value_map, octomap::OcTree* bounds_map,
                           Cost occupied_threshold = LETHAL);

  /** @brief Update the cells in binary fashion from the given values and bounds.
   * Note: if the value_map or bounds_map have a different depth from the
   * internal costmap_, the costmap_ will *not* be updated!
   * To update from maps with different depths, use the updateCells version
   * that takes non-const pointers to the maps, which will alter the depth of
   * the input trees first to match. */
  virtual void updateCells(const octomap::OcTree& value_map, const octomap::OcTree& bounds_map,
                           Cost occupied_threshold = LETHAL);

  /** @brief Update the cell at the given point.
   * If mark is true, mark the cell, otherwise clear it.
   * This layer must be holding the lock to make this call. */
  virtual void updateCell(const geometry_msgs::Point& point, bool mark=false);

  /** @brief Update the cell at the given key.
   * If mark is true, mark the cell, otherwise clear it.
   * This layer must be holding the lock to make this call. */
  virtual void updateCell(const octomap::OcTreeKey& key, bool mark=false);

  /** @brief Mark the cell at the given point.
   * This layer must be holding the lock to make this call. */
  virtual void markCell(const geometry_msgs::Point& point);

  /** @brief Mark the cell at the given key.
   * This layer must be holding the lock to make this call. */
  virtual void markCell(const octomap::OcTreeKey& key);

  /** @brief Clear the cell at the given point.
   * This layer must be holding the lock to make this call. */
  virtual void clearCell(const geometry_msgs::Point& point);

  /** @brief Clear the cell at the given key.
   * This layer must be holding the lock to make this call. */
  virtual void clearCell(const octomap::OcTreeKey& key);

  /** @brief Mark and clear cells from the given Costmap3D.
   *
   * This copies the given Costmap3D into this layer, overwriting any
   * overlapping cells with the costs in the passed in map.
   *
   * This layer must be holding the lock to make this call. */
  virtual void markAndClearCells(const Costmap3D& map);

  /** @brief Erase the cell at the given point.
   * This layer must be holding the lock to make this call. */
  virtual void eraseCell(const geometry_msgs::Point& point);

  /** @brief Erase the cell at the given key.
   * This layer must be holding the lock to make this call. */
  virtual void eraseCell(const octomap::OcTreeKey& key);

  /** @brief Erase the cells from this layer that exist in given map.
   *
   * Erase the cells in this costmap layer that exist in the passed in map.
   *
   * This layer must be holding the lock to make this call. */
  virtual void eraseCells(const Costmap3D& map);

  /** @brief Set the cell cost at the given key and depth.
   * This layer must be holding the lock to make this call. */
  virtual void setCellCostAtDepth(const octomap::OcTreeKey& key, Cost cost,
                                  unsigned int depth=std::numeric_limits<unsigned int>::max());

  /** @brief Set the cell cost at the given key.
   * This layer must be holding the lock to make this call. */
  virtual void setCellCost(const octomap::OcTreeKey& key, Cost cost);

  /** @brief Set the cell cost at the given point.
   * This layer must be holding the lock to make this call. */
  virtual void setCellCost(const geometry_msgs::Point& point, Cost cost);

  Costmap3DPtr costmap_;
  Costmap3DPtr changed_cells_;
  bool enabled_;
  int combination_method_;
};

}  // namespace costmap_3d

#endif  // COSTMAP_3D_COSTMAP_LAYER_3D_H_
