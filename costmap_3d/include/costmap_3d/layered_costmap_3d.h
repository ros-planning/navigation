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
#ifndef COSTMAP_3D_LAYERED_COSTMAP_3D_H_
#define COSTMAP_3D_LAYERED_COSTMAP_3D_H_

#include <vector>
#include <string>
#include <limits>
#include <mutex>
#include <map>
#include <functional>
#include <costmap_3d/layer_3d.h>
#include <costmap_3d/costmap_3d.h>
#include <costmap_2d/layered_costmap.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

namespace costmap_3d
{

class Layer3D;

/**
 * @class LayeredCostmap3D
 * @brief Instantiates different 3D layer plugins and aggregates them into one 3D costmap
 */
class LayeredCostmap3D
{
public:
  LayeredCostmap3D(const LayeredCostmap3D&) = delete;
  LayeredCostmap3D& operator=(const LayeredCostmap3D&) = delete;
  /**
   * @brief Constructor for a 3D layered costmap
   */
  LayeredCostmap3D(costmap_2d::LayeredCostmap* layered_costmap_2d);

  /**
   * @brief Destructor
   */
  ~LayeredCostmap3D();

  /**
   * @brief Update the underlying costmap with new data.
   */
  void updateMap(geometry_msgs::Pose robot_pose);

  /**
   * @brief Get the number of times the costmap has been updated.
   * Note: assumes lock is held (not using atomic ops or explicit barriers, so
   * an old value could be returned w/out the lock held)
   */
  unsigned int getNumberOfUpdates() const {return num_updates_;}

  inline const std::string& getGlobalFrameID() const noexcept
  {
    return layered_costmap_2d_->getGlobalFrameID();
  }

  /**
   * @brief Activate each layer (if deactivated).
   */
  void activate();

  /**
   * @brief Activate each layer (if deactivated).
   */
  void deactivate();

  /**
   * @brief Reset the entire costmap, deleting all state.
   */
  void reset();

  bool isCurrent() const;

  // The caller must be holding the lock to keep the costmap from updating on
  // them. It is highly recommended to use RAII lock acquisition, such as
  // std::lock_guard.
  Costmap3DConstPtr getCostmap3D() const;

  /** Returns if this costmap's x/y bounds are relative to the base
   */
  bool isRolling() const;

  const std::vector<boost::shared_ptr<Layer3D>>& getPlugins();

  void addPlugin(boost::shared_ptr<Layer3D> plugin);

  // Lock the master costmap (used by planners to keep the costmap consistent
  // while either planning, or copying the costmap to use for planning).
  void lock();
  void unlock();

  double getResolution() const;

  /// Return true if the given pose is inside the map's coordinate system
  bool isPoseInMap(geometry_msgs::Pose robot_pose);

  void getBounds(geometry_msgs::Point* min, geometry_msgs::Point* max);

  /**
   * @brief Set the Z bounds.
   * The X/Y bounds come from the 2D layered costmap.
   */
  void setBounds(double min_z, double max_z);

  // Pass the complete costmap, the delta map from the last update, and the
  // bounds map on every completion.
  using UpdateCompleteCallback = std::function<void(LayeredCostmap3D* layered_costmap_3d,
                                                    const Costmap3D& delta_map,
                                                    const Costmap3D& bounds_map)>;

  /**
   * @brief Register a callback to be called when a 3D costmap update is complete.
   */
  void registerUpdateCompleteCallback(const std::string callback_id, UpdateCompleteCallback cb);

  /**
   * @brief Unregister a callback to be called when a 3D costmap update is complete.
   */
  void unregisterUpdateCompleteCallback(const std::string callback_id);

  /**
   * @brief Get a pointer to the corresponding 2D layered costmap.
   */
  costmap_2d::LayeredCostmap* getLayeredCostmap2D() {return layered_costmap_2d_;}

private:
  // Check to see if our bounds match the 2D map, and change bounds if necessary.
  // Check to see if our resolution matches the 2D map, and change it if necessary.
  // Lock must be held while calling this internal function
  void matchBoundsAndResolution();

  // Update our Z bounds and also check to see if our bounds match the 2D map,
  // and change bounds if necessary.
  // Check to see if our resolution matches the 2D map, and change it if necessary.
  // Lock must be held while calling this internal function
  void matchBoundsAndResolution(double min_z, double max_z);

  // Lock must be held while calling this internal function
  void sizeChange();

  class LockLayers
  {
  public:
    LockLayers(LayeredCostmap3D* layered_costmap_3d);

    // lock all layers preventing them from updating any state needed to be
    // consistent between updateBounds and updateCosts
    void lock();

    // unlock all layers.
    void unlock();
  private:
    LayeredCostmap3D* layered_costmap_3d_;
  };

  LockLayers lock_layers_;

  // Master 3D cost map.
  // Starts off NULL until our resolution is setup.
  Costmap3DPtr costmap_;
  std::recursive_mutex costmap_mutex_;

  // Pointer to our sibling 2D costmap.
  // We can use this to find our global frame.
  // Also, we can use this pointer to keep the 2D costmap in sync with the 3D layers.
  costmap_2d::LayeredCostmap* layered_costmap_2d_;

  std::vector<boost::shared_ptr<Layer3D>> plugins_;
  std::mutex update_complete_callbacks_mutex_;
  std::map<std::string, UpdateCompleteCallback> update_complete_callbacks_;

  bool size_changed_;
  double resolution_;
  geometry_msgs::Point min_point_, max_point_;
  unsigned int num_updates_;
};

}  // namespace costmap_3d

#endif  // COSTMAP_3D_LAYERED_COSTMAP_3D_H_
