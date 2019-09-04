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
#include <costmap_3d/point_cloud_layer_3d.h>
#include <pluginlib/class_list_macros.h>
#include <tf_conversions/tf_eigen.h>

PLUGINLIB_EXPORT_CLASS(costmap_3d::PointCloudLayer3D, costmap_3d::Layer3D)

namespace costmap_3d
{

PointCloudLayer3D::PointCloudLayer3D() : super()
{
  current_ = false;
}

PointCloudLayer3D::~PointCloudLayer3D()
{
}

void PointCloudLayer3D::initialize(LayeredCostmap3D* parent, std::string name, tf2_ros::Buffer *tf)
{
  super::initialize(parent, name, tf);

  // Get names of topics and servers from parameter server
  pnh_ = ros::NodeHandle("~/" + name);

  cloud_topic_ = pnh_.param("cloud_topic", std::string("cloud"));
  cloud_has_intensity_ = pnh_.param("cloud_has_intensity", false);
  free_intensity_ = pnh_.param("free_intensity", 0.0);
  lethal_intensity_ = pnh_.param("lethal_intensity", 1.0);
  double data_valid_duration = pnh_.param("data_valid_duration", 0.0);
  data_valid_duration_ = ros::Duration(data_valid_duration);

  ROS_INFO_STREAM("PointCloudLayer3D " << name << ": initializing");
  ROS_INFO_STREAM("  cloud_topic: " << cloud_topic_);
  ROS_INFO_STREAM("  cloud_has_intensity: " << cloud_has_intensity_);
  if (cloud_has_intensity_)
  {
    ROS_INFO_STREAM("  free_intensity: " << free_intensity_);
    ROS_INFO_STREAM("  lethal_intensity: " << lethal_intensity_);
  }

  dsrv_.reset(new dynamic_reconfigure::Server<costmap_3d::GenericPluginConfig>(pnh_));
  dsrv_->setCallback(std::bind(&PointCloudLayer3D::reconfigureCallback, this,
                               std::placeholders::_1, std::placeholders::_2));

  activate();
}

void PointCloudLayer3D::reconfigureCallback(costmap_3d::GenericPluginConfig &config, uint32_t level)
{
  std::lock_guard<Layer3D> lock(*this);
  enabled_ = config.enabled;
  combination_method_ = config.combination_method;
}

void PointCloudLayer3D::updateBounds(
    const geometry_msgs::Pose robot_pose,
    const geometry_msgs::Point& rolled_min,
    const geometry_msgs::Point& rolled_max,
    Costmap3D* bounds_map)
{
  std::lock_guard<Layer3D> lock(*this);
  bool current = true;
  if (data_valid_duration_ > ros::Duration(0.0))
  {
    if (ros::Time::now() - last_update_stamp_ > data_valid_duration_)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Layer " << name_ << " point cloud not updated for "
                               << (ros::Time::now() - last_update_stamp_).toSec()
                               << " expected update in at least " <<  data_valid_duration_.toSec());
      current = false;
    }
  }
  current_ = current;
  super::updateBounds(robot_pose, rolled_min, rolled_max, bounds_map);
}


void PointCloudLayer3D::deactivate()
{
  std::lock_guard<Layer3D> lock(*this);
  active_ = false;
  unsubscribe();
  super::deactivate();
}

void PointCloudLayer3D::activate()
{
  std::lock_guard<Layer3D> lock(*this);
  super::activate();
  subscribe();
  active_ = true;
}

void PointCloudLayer3D::subscribe()
{
  if (cloud_has_intensity_)
  {
    cloud_sub_ = pnh_.subscribe<PointCloudWithIntensity>(
        cloud_topic_, 1, std::bind(&PointCloudLayer3D::pointCloudCallback<PointCloudWithIntensity>,
                                   this, std::placeholders::_1));
  }
  else
  {
    cloud_sub_ = pnh_.subscribe<PointCloud>(
        cloud_topic_, 1, std::bind(&PointCloudLayer3D::pointCloudCallback<PointCloud>,
                                   this, std::placeholders::_1));
  }
}

void PointCloudLayer3D::unsubscribe()
{
  cloud_sub_.shutdown();
}

template <typename PointType>
Cost PointCloudLayer3D::pointIntensityToCost(PointType point)
{
  // default is all points are lethal
  return LETHAL;
}

template <>
Cost PointCloudLayer3D::pointIntensityToCost<pcl::PointXYZI>(pcl::PointXYZI point)
{
  const float intensity = point.intensity;
  if (intensity >= lethal_intensity_)
    return LETHAL;
  if (intensity <= free_intensity_)
    return FREE;
  // Linearly interpolate the intensity value to the space between LETHAL and FREE
  return ((intensity - free_intensity_) / (lethal_intensity_ - free_intensity_)) * (LETHAL - FREE) + FREE;
}

template <typename CloudType>
void PointCloudLayer3D::pointCloudCallback(const typename CloudType::ConstPtr& cloud_msg)
{
  // Find the global frame transform.
  ros::Time stamp(pcl_conversions::fromPCL(cloud_msg->header.stamp));
  geometry_msgs::TransformStamped global_frame_transform;
  try
  {
    global_frame_transform = tf_->lookupTransform(layered_costmap_3d_->getGlobalFrameID(),
                                                  cloud_msg->header.frame_id, stamp);
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_INFO_STREAM_THROTTLE(1.0, "unable to transform: " << ex.what());
    return;
  }
  Eigen::Affine3d global_frame_transform_eigen;
  tf2::fromMsg(global_frame_transform, global_frame_transform_eigen);
  typename CloudType::Ptr cloud_ptr(new CloudType());
  pcl::transformPointCloud(*cloud_msg, *cloud_ptr, global_frame_transform_eigen);

  std::lock_guard<Layer3D> lock(*this);
  if (active_ && costmap_)
  {
    last_update_stamp_ = stamp;
    Costmap3D new_cells(costmap_->getResolution());
    for (auto pt : cloud_msg->points)
    {
      Cost pt_cost = pointIntensityToCost(pt);
      geometry_msgs::Point point;
      point.x = pt.x;
      point.y = pt.y;
      point.z = pt.z;
      Costmap3DIndex key;
      if (new_cells.coordToKeyChecked(toOctomapPoint(point), key))
      {
        Costmap3D::NodeType* node = new_cells.search(key);
        // Only set the maximum cost for a given key across the cloud
        if (!node || node->getValue() < pt_cost)
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
}

}  // namespace costmap_3d
