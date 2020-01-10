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
#ifndef COSTMAP_3D_OCTOMAP_SERVER_LAYER_3D_H_
#define COSTMAP_3D_OCTOMAP_SERVER_LAYER_3D_H_

#include <memory>
#include <dynamic_reconfigure/server.h>
#include <costmap_3d/GenericPluginConfig.h>
#include <costmap_3d/costmap_layer_3d.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapUpdate.h>

namespace costmap_3d
{

class OctomapServerLayer3D : public CostmapLayer3D
{
  using super = CostmapLayer3D;
public:
  OctomapServerLayer3D();
  virtual ~OctomapServerLayer3D();

  virtual void initialize(LayeredCostmap3D* parent, std::string name, tf2_ros::Buffer *tf);

  virtual void updateBounds(const geometry_msgs::Pose robot_pose,
                            const geometry_msgs::Point& rolled_min,
                            const geometry_msgs::Point& rolled_max,
                            Costmap3D* bounds_map);

  /** @brief Deactivate this layer, unsubscribing.
   */
  virtual void deactivate();

  /** @brief Activate this layer, subscribing. */
  virtual void activate();

  /** @brief Forward the reset to the octomap server as well. */
  virtual void reset();

  /** @brief Force a full transfer on resolution change. */
  virtual void matchSize(const geometry_msgs::Point& min, const geometry_msgs::Point& max, double resolution);

protected:
  /** @brief Forward the bound box reset to the octomap server as well. */
  virtual void resetBoundingBoxUnlocked(Costmap3DIndex min, Costmap3DIndex max);

  virtual void mapCallback(const octomap_msgs::OctomapConstPtr& map_msg);
  virtual void mapUpdateCallback(const octomap_msgs::OctomapUpdateConstPtr& map_update_msg);

  virtual void mapUpdateInternal(const octomap_msgs::Octomap* map_msg,
                                 const octomap_msgs::Octomap* bounds_msg);

  virtual void reconfigureCallback(costmap_3d::GenericPluginConfig &config, uint32_t level);

  virtual void subscribe();
  virtual void unsubscribe();
  virtual void subscribeUpdatesUnlocked();
  virtual void unsubscribeUpdatesUnlocked();
  virtual void scheduleResubscribeUpdates();
  virtual void resubscribeUpdatesCallback();

  ros::NodeHandle pnh_;
  std::shared_ptr<dynamic_reconfigure::Server<costmap_3d::GenericPluginConfig>> dsrv_;
  std::string map_topic_;
  std::string map_update_topic_;
  std::string reset_srv_name_;
  std::string erase_bbx_srv_name_;
  ros::ServiceClient reset_client_;
  ros::ServiceClient erase_bbx_client_;
  ros::Subscriber map_sub_;
  ros::Subscriber map_update_sub_;
  bool using_updates_;
  bool active_;
  bool first_full_map_update_received_;
  size_t num_map_updates_;
  uint32_t last_seq_;
  ros::Duration data_valid_duration_;
  ros::Time last_update_stamp_;
  ros::Timer resub_timer_;  // one-shot timer used for resubscribing
};

}  // namespace costmap_3d

#endif  // COSTMAP_3D_OCTOMAP_SERVER_LAYER_3D_H_
