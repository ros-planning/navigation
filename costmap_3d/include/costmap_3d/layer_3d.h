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
#ifndef COSTMAP_3D_LAYER_3D_H_
#define COSTMAP_3D_LAYER_3D_H_

#include <costmap_3d/costmap_3d.h>
#include <costmap_3d/layered_costmap_3d.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <tf2_ros/buffer.h>

namespace costmap_3d
{

class LayeredCostmap3D;

/**
 * Interface that layered costmaps use to access individual costmap layers.
 * All costmap layers must extend this abstract class.
 */
class Layer3D
{
public:
  Layer3D();
  virtual ~Layer3D();

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
                            Costmap3D* bounds_map) = 0;

  /** @brief Copy the state of this costmap layer into the master layer using
   * the bounds_map to limit what portion is copied.
   *
   * The point of updateCosts is to efficiently update the master layer
   * using the update policy (update with max, or update with overwrite, or
   * true overwrite).
   */
  virtual void updateCosts(const Costmap3D& bounds_map, Costmap3D* master_map) = 0;

  /** @brief Initialize this layer. */
  virtual void initialize(LayeredCostmap3D* parent, std::string name, tf2_ros::Buffer *tf);

  /** @brief Deactivate this layer. */
  virtual void deactivate() = 0;

  /** @brief Activate this layer. */
  virtual void activate() = 0;

  /** @brief Forget all memory as if restarting */
  virtual void reset() = 0;

  /** @brief Forget old sensor data within the given axis-aligned bounding box. */
  virtual void resetBoundingBox(geometry_msgs::Point min_point, geometry_msgs::Point max_point) = 0;

  /**
   * @brief Check to make sure all the data in the layer is up to date.
   *        If the layer is not up to date, then it may be unsafe to
   *        plan using the data from this layer, and the planner may
   *        need to know.
   *
   *        A layer's current state should be managed by the protected
   *        variable current_.
   * @return Whether the data in the layer is up to date.
   */
  virtual bool isCurrent() const;

  /** @brief Called when the bounds or resolution changes.
   *
   * Note that in the case of any of these changes that the master costmap
   * will always be reset and it will use a bounds map representing all space,
   * so there is no need to attempt to attempt to track the bounds across a
   * size change.
   * Note also that unlike 2D costmaps, rolling 3D costmaps do not roll min
   * and max. The OcTree stays at the original global origin, but the bounds
   * are adjusted in updateMap to match the current robot pose. So for a
   * rolling map, a plugin can delete any information outside the 2D limits of
   * (max.x - min.x) / 2  or (max.y - min.y) / 2 from the robot pose, or it
   * can independetly track the robot pose to efficiently only ray trace space
   * inside the rolling window. For a non-rolling map, min and max represent
   * the global frame limits (inclusive) for data allowed to be in the
   * costmap.
   * Note that the layered costmap will actively delete any out-of-bounds
   * areas, so it is not possible to put things off the map.
   */
  virtual void matchSize(const geometry_msgs::Point& min, const geometry_msgs::Point& max, double resolution) = 0;

  virtual inline const std::string& getName() const noexcept
  {
    return name_;
  }

  virtual void lock();

  virtual void unlock();

protected:
  LayeredCostmap3D* layered_costmap_3d_;
  bool current_;
  std::string name_;
  tf2_ros::Buffer *tf_;
  std::recursive_mutex mutex_;
};

}  // namespace costmap_3d

#endif  // COSTMAP_3D_LAYER_3D_H_
