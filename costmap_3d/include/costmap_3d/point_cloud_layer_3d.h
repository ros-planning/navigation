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
#ifndef COSTMAP_3D_POINT_CLOUD_LAYER_3D_H_
#define COSTMAP_3D_POINT_CLOUD_LAYER_3D_H_

#include <memory>
#include <dynamic_reconfigure/server.h>
#include <costmap_3d/GenericPluginConfig.h>
#include <costmap_3d/costmap_layer_3d.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace costmap_3d
{

class PointCloudLayer3D : public CostmapLayer3D
{
  using super = CostmapLayer3D;
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudWithIntensity = pcl::PointCloud<pcl::PointXYZI>;
public:
  PointCloudLayer3D();
  virtual ~PointCloudLayer3D();

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

protected:
  template <typename PointType>
  Cost pointIntensityToCost(PointType point);

  template <typename CloudType>
  void pointCloudCallback(const typename CloudType::ConstPtr& cloud_msg);

  virtual void reconfigureCallback(costmap_3d::GenericPluginConfig &config, uint32_t level);

  virtual void subscribe();
  virtual void unsubscribe();

  ros::NodeHandle pnh_;
  std::shared_ptr<dynamic_reconfigure::Server<costmap_3d::GenericPluginConfig>> dsrv_;
  bool active_;
  std::string cloud_topic_;
  ros::Subscriber cloud_sub_;
  bool cloud_has_intensity_;
  // Intensity values at or below free_intensity_ are considred free.
  // Intensity values at or above lethal_intensity_ value are considered lethal.
  // Intensity values between free_intensity_ and lethal_intesnity_ are linearly interpolated.
  float free_intensity_;
  float lethal_intensity_;
  ros::Duration data_valid_duration_;
  ros::Time last_update_stamp_;
};

}  // namespace costmap_3d

#endif  // COSTMAP_3D_POINT_CLOUD_LAYER_3D_H_
