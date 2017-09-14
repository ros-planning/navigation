/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/voxel_layer.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <voxel_grid/basic_clearer.h>
#include <voxel_grid/basic_marker.h>
#include <voxel_grid/plain_clearer.h>
#include <voxel_grid/cache_clearer.h>

#define VOXEL_BITS 16
PLUGINLIB_EXPORT_CLASS(costmap_2d::VoxelLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace costmap_2d
{

void VoxelLayer::onInitialize()
{
  ObstacleLayer::onInitialize();
  ros::NodeHandle private_nh("~/" + name_);

  private_nh.param("publish_voxel_map", publish_voxel_, false);
  if (publish_voxel_)
    voxel_pub_ = private_nh.advertise<costmap_2d::VoxelGrid>("voxel_grid", 1);
  clearing_endpoints_pub_ = private_nh.advertise<sensor_msgs::PointCloud>("clearing_endpoints", 1);

  private_nh.param("clear_corner_cases", clear_corner_cases_, false);

  int accuracy_multiplier_bits = 10;
  private_nh.param("accuracy_multiplier_bits", accuracy_multiplier_bits, 10);
  voxel_grid_.setAccuracyMultiplierBits(accuracy_multiplier_bits);

  private_nh.param("use_cached_updating", use_cached_updating_, false);
  cleared_points_pub_ = private_nh.advertise<sensor_msgs::PointCloud>("cleared_voxels", 1);

  if (clear_corner_cases_)
    use_cached_updating_ = true;
  combination_method_ = 1;
}

void VoxelLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  voxel_dsrv_ = new dynamic_reconfigure::Server<costmap_2d::VoxelPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::VoxelPluginConfig>::CallbackType cb = boost::bind(&VoxelLayer::reconfigureCB,
                                                                                            this, _1, _2);
  voxel_dsrv_->setCallback(cb);
}

VoxelLayer::~VoxelLayer()
{
  if (voxel_dsrv_)
    delete voxel_dsrv_;
}

void VoxelLayer::reconfigureCB(costmap_2d::VoxelPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  footprint_clearing_enabled_ = config.footprint_clearing_enabled;
  max_obstacle_height_ = config.max_obstacle_height;
  size_z_ = config.z_voxels;
  origin_z_ = config.origin_z;
  z_resolution_ = config.z_resolution;

  unknown_threshold_ = config.unknown_threshold + (VOXEL_BITS - size_z_);
  mark_threshold_ = config.mark_threshold;
  combination_method_ = config.combination_method;

  clear_corner_cases_ = config.clear_corner_cases;
  voxel_grid_.setAccuracyMultiplierBits(config.accuracy_multiplier_bits);
  use_cached_updating_ = config.use_cached_updating;

  //we need to use cached updating in case of clearing corners, or else we get overflows
  if (clear_corner_cases_)
    use_cached_updating_ = true;

  matchSize();
}

void VoxelLayer::matchSize()
{
  ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);
  ROS_ASSERT(voxel_grid_.sizeX() == size_x_ && voxel_grid_.sizeY() == size_y_);
}

void VoxelLayer::reset()
{
  deactivate();
  resetMaps();
  voxel_grid_.reset();
  activate();
}

void VoxelLayer::resetMaps()
{
  Costmap2D::resetMaps();
  voxel_grid_.reset();
}

void VoxelLayer::resetGrid()
{
  voxel_grid_.reset();
}

void VoxelLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                              double* max_x, double* max_y)
{
  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!enabled_)
    return;
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  current = getMarkingObservations(observations) && current;

  // get the clearing observations
  current = getClearingObservations(clearing_observations) && current;

  // update the global current status
  current_ = current;

  // raytrace freespace
  clear(clearing_observations,  min_x, min_y, max_x, max_y);

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const Observation& obs = *it;

    const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

    for (unsigned int i = 0; i < cloud.points.size(); ++i)
    {
      // if the obstacle is too high or too far away from the robot we won't add it
      if (cloud.points[i].z > max_obstacle_height_)
        continue;

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (cloud.points[i].x - obs.origin_.x) * (cloud.points[i].x - obs.origin_.x)
          + (cloud.points[i].y - obs.origin_.y) * (cloud.points[i].y - obs.origin_.y)
          + (cloud.points[i].z - obs.origin_.z) * (cloud.points[i].z - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range)
        continue;

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my, mz;
      if (!worldToMap3D(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, mx, my, mz))
      {
        continue;
      }

      // mark the cell in the voxel grid and check if we should also mark it in the costmap
      if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_))
      {
        unsigned int index = getIndex(mx, my);

        costmap_[index] = LETHAL_OBSTACLE;
        touch((double)cloud.points[i].x, (double)cloud.points[i].y, min_x, min_y, max_x, max_y);
      }
    }
  }

  if (publish_voxel_)
  {
    costmap_2d::VoxelGrid grid_msg;
    unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
    grid_msg.size_x = voxel_grid_.sizeX();
    grid_msg.size_y = voxel_grid_.sizeY();
    grid_msg.size_z = voxel_grid_.sizeZ();
    grid_msg.data.resize(size);
    memcpy(&grid_msg.data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

    grid_msg.origin.x = origin_x_;
    grid_msg.origin.y = origin_y_;
    grid_msg.origin.z = origin_z_;

    grid_msg.resolutions.x = resolution_;
    grid_msg.resolutions.y = resolution_;
    grid_msg.resolutions.z = z_resolution_;
    grid_msg.header.frame_id = global_frame_;
    grid_msg.header.stamp = ros::Time::now();
    voxel_pub_.publish(grid_msg);
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void VoxelLayer::clearNonLethal(double wx, double wy, double w_size_x, double w_size_y, bool clear_no_info)
{
  // get the cell coordinates of the center point of the window
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my))
    return;

  // compute the bounds of the window
  double start_x = wx - w_size_x / 2;
  double start_y = wy - w_size_y / 2;
  double end_x = start_x + w_size_x;
  double end_y = start_y + w_size_y;

  // scale the window based on the bounds of the costmap
  start_x = std::max(origin_x_, start_x);
  start_y = std::max(origin_y_, start_y);

  end_x = std::min(origin_x_ + getSizeInMetersX(), end_x);
  end_y = std::min(origin_y_ + getSizeInMetersY(), end_y);

  // get the map coordinates of the bounds of the window
  unsigned int map_sx, map_sy, map_ex, map_ey;

  // check for legality just in case
  if (!worldToMap(start_x, start_y, map_sx, map_sy) || !worldToMap(end_x, end_y, map_ex, map_ey))
    return;

  // we know that we want to clear all non-lethal obstacles in this window to get it ready for inflation
  unsigned int index = getIndex(map_sx, map_sy);
  unsigned char* current = &costmap_[index];
  for (unsigned int j = map_sy; j <= map_ey; ++j)
  {
    for (unsigned int i = map_sx; i <= map_ex; ++i)
    {
      // if the cell is a lethal obstacle... we'll keep it and queue it, otherwise... we'll clear it
      if (*current != LETHAL_OBSTACLE)
      {
        if (clear_no_info || *current != NO_INFORMATION)
        {
          *current = FREE_SPACE;
          voxel_grid_.clearVoxelColumn(index);
        }
      }
      current++;
      index++;
    }
    current += size_x_ - (map_ex - map_sx) - 1;
    index += size_x_ - (map_ex - map_sx) - 1;
  }
}

void VoxelLayer::clear(std::vector<Observation>& observations, double* min_x, double* min_y,
                       double* max_x, double* max_y)
{
  boost::shared_ptr<voxel_grid::AbstractGridUpdater> voxel_clearer;
  unsigned int update_area_center = 0;

  if (use_cached_updating_)
  {
    unsigned int max_raytrace_range_in_cells = std::floor(max_raytrace_range_ / resolution_);

    update_area_center = max_raytrace_range_in_cells + 1; // +1 to have a buffer
    unsigned int cached_update_area_width = update_area_center * 2 + 1; //+1 to have a center point

    voxel_clearer = boost::shared_ptr<voxel_grid::CacheClearer>(
        new voxel_grid::CacheClearer(voxel_grid_.getData(), costmap_, voxel_grid_.sizeX(), voxel_grid_.sizeY(),
				     cached_update_area_width,
				     unknown_threshold_, mark_threshold_,
				     clear_corner_cases_,
				     FREE_SPACE, NO_INFORMATION));
  }
  else
  {
    voxel_clearer = boost::shared_ptr<voxel_grid::PlainClearer>(
        new voxel_grid::PlainClearer(voxel_grid_.getData(), costmap_, unknown_threshold_, mark_threshold_, FREE_SPACE,
                                      NO_INFORMATION));
  }

  bool publish_clearing_points = (clearing_endpoints_pub_.getNumSubscribers() > 0);
  if (publish_clearing_points)
  {
    clearing_endpoints_.points.clear();
  }

  for (unsigned int i = 0; i < observations.size(); ++i)
  {
    raytraceFreespace(observations[i], voxel_clearer, min_x, min_y, max_x, max_y, update_area_center);
  }

  if (use_cached_updating_)
  {
    boost::shared_ptr<voxel_grid::CacheClearer> cached_clearer = boost::static_pointer_cast<voxel_grid::CacheClearer>(
        voxel_clearer);

    cached_clearer->update();

    updated_cells_indices_ = cached_clearer->getClearedCellsIndices();

    bool publish_cleared_points = (cleared_points_pub_.getNumSubscribers() > 0);
    if (publish_cleared_points)
    {
      sensor_msgs::PointCloud cleared_voxels = cached_clearer->getClearedVoxels();
      convertFromMapToWorld(cleared_voxels);

      cleared_voxels.header.frame_id = global_frame_;
      cleared_voxels.header.stamp = ros::Time::now();
      cleared_points_pub_.publish(cleared_voxels);
    }
  }

  if (publish_clearing_points && !observations.empty())
  {
    clearing_endpoints_.header.frame_id = global_frame_;
    //don't know which one to use in case of multiple observations, so just use last one
    clearing_endpoints_.header.stamp = pcl_conversions::fromPCL(observations.back().cloud_->header).stamp;
    clearing_endpoints_.header.seq = observations.back().cloud_->header.seq;

    clearing_endpoints_pub_.publish(clearing_endpoints_);
  }

}

void VoxelLayer::raytraceFreespace(const Observation& clearing_observation, boost::shared_ptr<voxel_grid::AbstractGridUpdater> voxel_clearer,
                                   double* min_x, double* min_y, double* max_x, double* max_y, unsigned int update_area_center)
{
  if (clearing_observation.cloud_->points.size() == 0)
    return;

  double sensor_x, sensor_y, sensor_z;
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  double oz = clearing_observation.origin_.z;

  if (!worldToMap3DFloat(ox, oy, oz, sensor_x, sensor_y, sensor_z))
  {
    ROS_ERROR_THROTTLE(
        1.0,
        "The origin for the sensor at (%.2f, %.2f, %.2f) in voxel layer of %s is out of map bounds. So, the costmap cannot raytrace for it.",
        ox, oy, oz, name_.c_str());

    double mx = ((ox - origin_x_) / resolution_);
    double my = ((oy - origin_y_) / resolution_);
    double mz = ((oz - origin_z_) / z_resolution_);

    ROS_WARN_STREAM("Calculated costmap position of sensor: " << mx << ", " << my << ", " << mz);
    ROS_WARN_STREAM("Map origin: " << origin_x_ << ", " << origin_y_ << ", " << origin_z_);
    ROS_WARN_STREAM("Map size: " << size_x_ << ", " << size_y_ << ", " << size_z_);

    return;
  }

  double start_offset_x;
  double start_offset_y;
  unsigned int cached_update_area_width = update_area_center * 2 + 1;
  bool publish_clearing_points = (clearing_endpoints_pub_.getNumSubscribers() > 0);

  if (use_cached_updating_)
  {
    int offset_x = update_area_center - (int)sensor_x;
    int offset_y = update_area_center - (int)sensor_y;

    //we raytrace in the cache to skip the "slow" transformation between real world and cache coordinates
    //sub-cell accuracy of start point has to be the same, so add the fractional part
    double temp = 0;
    start_offset_x = update_area_center + fabs(modf(sensor_x, &temp));
    start_offset_y = update_area_center + fabs(modf(sensor_y, &temp));

    boost::shared_ptr<voxel_grid::CacheClearer> cached_clearer = boost::static_pointer_cast<voxel_grid::CacheClearer>(
        voxel_clearer);

    cached_clearer->setCostmapOffsets(offset_x, offset_y);
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double map_end_x = origin_x_ + size_x_ * resolution_;
  double map_end_y = origin_y_ + size_y_ * resolution_;
  double epsilon = 0.0001;

  for (unsigned int i = 0; i < clearing_observation.cloud_->points.size(); ++i)
  {
    double wpx = clearing_observation.cloud_->points[i].x;
    double wpy = clearing_observation.cloud_->points[i].y;
    double wpz = clearing_observation.cloud_->points[i].z;

    double dx_global = wpx - origin_x_;
    double dy_global = wpy - origin_y_;
    double dz_global = wpz - origin_z_;

    double point_x = dx_global / resolution_;
    double point_y = dy_global / resolution_;
    double point_z = dz_global / z_resolution_;

    double dx = point_x - sensor_x;
    double dy = point_y - sensor_y;
    double dz = point_z - sensor_z;

    double abs_dx = std::abs(dx);
    double abs_dy = std::abs(dy);
    double abs_dz = std::abs(dz);

    //We might go outside of the voxel grid in any direction. We need to stop before that.
    //Normally raytracing stops in front of the obstacle (does not clear the end point cell / obstacle cell).
    //In the case we go outside, we want to go all the way to the border (and also clear the cell at the border).
    //Because of this we crop at 0.0 - epsilon and size (which is max index +1), "setting the obstacle cell just outside of the border".
    double cropped_distance = 0.0;

    // Initialize scale based on raytrace range.
    // When c++17 is supported, use hypot(abs_dx, abs_dy, abs_dz) to compute observation length
    double obs_len = std::sqrt((abs_dx * resolution_) * (abs_dx * resolution_) +
                               (abs_dy * resolution_) * (abs_dy * resolution_) +
                               (abs_dz * z_resolution_) * (abs_dz * z_resolution_));
    double scaling = std::min(1.0, max_raytrace_range_/obs_len);

    //Check if we go outside, and set the scaling factor accordingly
    if (point_x < 0.0)
    {
      cropped_distance = std::abs(-epsilon - sensor_x);
      scaling = std::min(scaling, cropped_distance / abs_dx);
    }
    else if (point_x > size_x_)
    {
      cropped_distance = std::abs(size_x_ - sensor_x);
      scaling = std::min(scaling, cropped_distance / abs_dx);
    }

    if (point_y < 0.0)
    {
      cropped_distance = std::abs(-epsilon - sensor_y);
      scaling = std::min(scaling, cropped_distance / abs_dy);
    }
    else if (point_y > size_y_)
    {
      cropped_distance = std::abs(size_y_ - sensor_y);
      scaling = std::min(scaling, cropped_distance / abs_dy);
    }

    if (point_z < 0.0)
    {
      cropped_distance = std::abs((origin_z_ / z_resolution_) - epsilon - sensor_z);
      scaling = std::min(scaling, cropped_distance / abs_dz);
    }
    else if (point_z > size_z_)
    {
      cropped_distance = std::abs(max_obstacle_height_ / z_resolution_ - sensor_z);
      scaling = std::min(scaling, cropped_distance / abs_dz);
    }

    dx = dx * scaling;
    dy = dy * scaling;
    dz = dz * scaling;

    if (use_cached_updating_)
    {
      //the line goes from the start point to the end point which is start point + distance
      double x1 = start_offset_x + dx;
      double y1 = start_offset_y + dy;
      double z1 = sensor_z + dz;
      voxel_grid_.clearVoxelLineInMap(start_offset_x, start_offset_y, sensor_z,
				      x1, y1, z1, &(*voxel_clearer),
				      cached_update_area_width);
    }
    else
    {
      double x1 = sensor_x + dx;
      double y1 = sensor_y + dy;
      double z1 = sensor_z + dz;
      voxel_grid_.clearVoxelLineInMap(sensor_x, sensor_y, sensor_z,
				      x1, y1, z1, &(*voxel_clearer),
				      voxel_grid_.sizeX());
    }

    wpx = origin_x_ + (sensor_x + dx) * resolution_;
    wpy = origin_y_ + (sensor_y + dy) * resolution_;
    wpz = origin_z_ + (sensor_z + dz) * z_resolution_;

    updateRaytraceBounds(ox, oy, wpx, wpy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);

    if (publish_clearing_points)
    {
      geometry_msgs::Point32 point;
      point.x = wpx;
      point.y = wpy;
      point.z = wpz;
      clearing_endpoints_.points.push_back(point);
    }
  }
}

void VoxelLayer::convertFromMapToWorld(sensor_msgs::PointCloud& point_cloud)
{
  for (int i = 0; i < point_cloud.points.size(); ++i)
  {
    double x = 0;
    double y = 0;
    double z = 0;

    mapToWorld3D(point_cloud.points[i].x, point_cloud.points[i].y, point_cloud.points[i].z, x, y, z);

    point_cloud.points[i].x = x;
    point_cloud.points[i].y = y;
    point_cloud.points[i].z = z;
  }
}

void VoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = int((new_origin_x - origin_x_) / resolution_);
  cell_oy = int((new_origin_y - origin_y_) / resolution_);

  // compute the associated world coordinates for the origin cell
  // beacuase we want to keep things grid-aligned
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  // To save casting from unsigned int to int a bunch of times
  int size_x = size_x_;
  int size_y = size_y_;

  // we need to compute the overlap of the new and existing windows
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  // we need a map to store the obstacles in the window temporarily
  unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];
  unsigned int* local_voxel_map = new unsigned int[cell_size_x * cell_size_y];
  unsigned int* voxel_map = voxel_grid_.getData();

  // copy the local window in the costmap to the local map
  copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);
  copyMapRegion(voxel_map, lower_left_x, lower_left_y, size_x_, local_voxel_map, 0, 0, cell_size_x, cell_size_x,
                cell_size_y);

  // we'll reset our maps to unknown space if appropriate
  resetMaps();

  // update the origin with the appropriate world coordinates
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  // compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  // now we want to copy the overlapping information back into the map, but in its new location
  copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);
  copyMapRegion(local_voxel_map, 0, 0, cell_size_x, voxel_map, start_x, start_y, size_x_, cell_size_x, cell_size_y);

  // make sure to clean up
  delete[] local_map;
  delete[] local_voxel_map;
}

} // namespace costmap_2d
