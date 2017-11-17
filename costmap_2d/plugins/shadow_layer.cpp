/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017 6 River Systems
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
 *   * Neither the name of 6 River Systems nor the names of its
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
 * Author: Daniel Grieneisen
 *********************************************************************/
#include <costmap_2d/shadow_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

#include <visualization_msgs/Marker.h>


PLUGINLIB_EXPORT_CLASS(costmap_2d::ShadowLayer, costmap_2d::Layer)

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace costmap_2d
{

void ShadowLayer::onInitialize()
{
  ROS_INFO("Initializing shadow layer");
  ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();

  default_value_ = UNKNOWN;

  ShadowLayer::matchSize();
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);

  visualization_publisher_ = nh.advertise<visualization_msgs::Marker> ("shadow_points", 1);

}

void ShadowLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ShadowPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::ShadowPluginConfig>::CallbackType cb = boost::bind(
      &ShadowLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

ShadowLayer::~ShadowLayer()
{
  if (dsrv_) 
  {
    delete dsrv_;      
  }
}

void ShadowLayer::reconfigureCB(costmap_2d::ShadowPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  shadow_scan_range_ = config.shadow_scan_range;
  shadow_scan_angular_resolution_ = config.shadow_scan_angular_resolution;
  shadow_scan_half_angle_ = config.shadow_scan_half_angle;
  min_shadow_size_ = config.min_shadow_size;
  shadow_half_width_ = min_shadow_size_ / resolution_;

  publish_shadow_objects_ = config.publish_shadow_objects;
}


void ShadowLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  ROS_DEBUG("Updating bounds for shadow layer");
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  if (!enabled_) {
    return;
  }

  // Bounds do not need to be updated b/c we are not adding cost to the costmap. yet.
  /// TODO: update bounds when this functionality is added.


  // generate the observation to be used
  generateMaxShadowObservation(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ShadowLayer::generateMaxShadowObservation(double robot_x, double robot_y, double robot_yaw,
                                          double* min_x, double* min_y, double* max_x, double* max_y)
{
  ROS_DEBUG("Generating max shadow observation");
  // Should this be based on the lidar scan?
  // For now, just generate some points.
  shadow_observation_ = std::make_shared<Observation>();
  geometry_msgs::Point origin;
  origin.x = robot_x;
  origin.y = robot_y;
  shadow_observation_->origin_ = origin;
  
  pcl::PointCloud <pcl::PointXYZ> &observation_cloud = *(shadow_observation_->cloud_);

  unsigned int num_points = shadow_scan_half_angle_ / shadow_scan_angular_resolution_ * 2 + 1;
  observation_cloud.reserve(num_points);

  double angle_start = -shadow_scan_half_angle_ + robot_yaw;
  double angle_end = shadow_scan_half_angle_ + robot_yaw;
  for (double angle = angle_start; 
    angle <= angle_end; 
    angle += shadow_scan_angular_resolution_)
  {
    double x = cos(angle) * shadow_scan_range_ + robot_x;
    double y = sin(angle) * shadow_scan_range_ + robot_y;
    observation_cloud.push_back(pcl::PointXYZ(x, y, 0.0)
    );

    touch(x, y, min_x, min_y, max_x, max_y);
  }
  ROS_DEBUG("Resized to %f, %f, %f, %f", *min_x, *min_y, *max_x, *max_y);
}


void ShadowLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  ROS_DEBUG("Updating shadow layer costs");
  if (!enabled_) {
    return;
  }

  calculateShadows(master_grid);
}

void ShadowLayer::calculateShadows(costmap_2d::Costmap2D& master_grid)
{
  ROS_DEBUG("Calculating shadows");
  // First, reset the costmap.
  resetMaps();

  // Raytrace the observations into the map
  raytraceShadowObservation(master_grid);

  // Calculate all of the spots where there could be a shadow.
  calculateShadowedObjects();

  if (publish_shadow_objects_)
  {
    publishShadowObjects();
  }
}


// void ShadowLayer::publishShadowRegions()
// {
//   if (!shadowed_objects_)
//   {
//     return;
//   }

//   ROS_INFO("Publishing display objects");
//   visualization_msgs::Marker sphere_list;
//   sphere_list.header.frame_id= global_frame_;
//   sphere_list.header.stamp= ros::Time::now();
//   sphere_list.ns= "shadows";
//   sphere_list.action= visualization_msgs::Marker::ADD;
//   sphere_list.pose.orientation.w= 1.0;

//   sphere_list.id = 0;

//   sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;


//   // POINTS markers use x and y scale for width/height respectively
//   sphere_list.scale.x = min_shadow_size_;
//   sphere_list.scale.y = min_shadow_size_;
//   sphere_list.scale.z = min_shadow_size_;

//   // Points are green
//   sphere_list.color.r = 1.0f;
//   sphere_list.color.a = 1.0;

//   for (auto pt : (*shadowed_objects_))
//   {
//     sphere_list.points.push_back(pt);
//   }
//   visualization_publisher_.publish(sphere_list);
// }


void ShadowLayer::publishShadowObjects()
{
  if (!shadowed_objects_)
  {
    return;
  }

  ROS_DEBUG("Publishing display objects");
  visualization_msgs::Marker sphere_list;
  sphere_list.header.frame_id= global_frame_;
  sphere_list.header.stamp= ros::Time::now();
  sphere_list.ns= "shadows";
  sphere_list.action= visualization_msgs::Marker::ADD;
  sphere_list.pose.orientation.w= 1.0;

  sphere_list.id = 0;

  sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;


  // POINTS markers use x and y scale for width/height respectively
  sphere_list.scale.x = min_shadow_size_;
  sphere_list.scale.y = min_shadow_size_;
  sphere_list.scale.z = min_shadow_size_;

  // Points are green
  sphere_list.color.r = 1.0f;
  sphere_list.color.a = 1.0;

  for (auto pt : (*shadowed_objects_))
  {
    sphere_list.points.push_back(pt);
  }
  visualization_publisher_.publish(sphere_list);
}

void ShadowLayer::raytraceShadowObservation(costmap_2d::Costmap2D& master_grid)
{
  if (!shadowed_points_)
  {
    shadowed_points_ = std::make_shared<std::vector<unsigned int>>();
  }
  // Clear out the old shadow points
  shadowed_points_->clear();

  // Check if the observation is valid.
  if (!shadow_observation_)
  {
    ROS_WARN_THROTTLE(1.0, "Cannot raytrace in shadow layer because there is no shadow observation.");
    return;
  }

  // 
  double ox = shadow_observation_->origin_.x;
  double oy = shadow_observation_->origin_.y;
  pcl::PointCloud < pcl::PointXYZ > cloud = *(shadow_observation_->cloud_);

  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  if (!worldToMap(ox, oy, x0, y0))
  {
    ROS_WARN_THROTTLE(
        1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
        ox, oy);
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;


  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  for (unsigned int i = 0; i < cloud.points.size(); ++i)
  {
    double wx = cloud.points[i].x;
    double wy = cloud.points[i].y;

    double a = wx - ox;
    double b = wy - oy;
    double ray_length = sqrt(a * a + b * b);


    // calculate raytrace starting point
    // the raytrace range should be (rx, ry) -> (wx, wy)
    double rx = ox;
    double ry = oy;

    // now we need to make sure that the point we're raytracing
    // to isn't off the costmap and scale if necessary
    checkRaytracePoint(origin_x, origin_y, map_end_x, map_end_y, ox, oy, wx, wy);
    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // check for legality just in case
    if (!worldToMap(wx, wy, x1, y1))
      continue;

    // same process for rx and ry
    checkRaytracePoint(origin_x, origin_y, map_end_x, map_end_y, ox, oy, rx, ry);

    unsigned int rx_map, ry_map;

    // check for legality of the raytrace starting point
    if (!worldToMap(rx, ry, rx_map, ry_map)){
      continue;
    }

    ShadowMarker marker(costmap_, master_grid.getCharMap(), shadowed_points_,
      VISIBLE, SHADOW, OBSTACLE);


    // and finally... we can execute our trace to clear obstacles along that line
    raytraceLine(marker, rx_map, ry_map, x1, y1);
  }
}


void ShadowLayer::calculateShadowedObjects()
{

  if (!shadowed_objects_)
  {
    shadowed_objects_ = std::make_shared<std::vector<geometry_msgs::Point>>();
  }
  shadowed_objects_->clear();
  ROS_DEBUG("Checking %d shadowed points", shadowed_points_->size());
  // Iterate over all of the shadowed_points_
  for (auto it : (*shadowed_points_))
  {
    if (checkForShadowedObjectAtIndex(it))
    {
      shadowed_objects_->push_back(createPointFromIndex(it));
    }
  }
  ROS_DEBUG("Returned %d shadowed objects", shadowed_objects_->size());
}

bool ShadowLayer::checkForShadowedObjectAtIndex(unsigned int idx)
{
  unsigned int idx_x, idx_y;
  indexToCells(idx, idx_x, idx_y);

  // calculate the square to check
  unsigned int start_x = idx_x > shadow_half_width_ ? idx_x - shadow_half_width_ : 0;
  unsigned int start_y = idx_y > shadow_half_width_ ? idx_y - shadow_half_width_ : 0;
  unsigned int end_x = idx_x < getSizeInCellsX() - 1 - shadow_half_width_ 
    ? idx_x + shadow_half_width_ : getSizeInCellsX() - 1;
  unsigned int end_y = idx_y < getSizeInCellsY() - 1 - shadow_half_width_ 
    ? idx_y + shadow_half_width_ : getSizeInCellsY() - 1;

  // Iterate over the square and see if there are any CLEAR or OBSTACLE values
  for (unsigned int y_val = start_y; y_val <= end_y; ++y_val)
  {
    for (unsigned int x_val = start_x; x_val <= end_x; ++x_val)
    {
      switch (costmap_[getIndex(x_val, y_val)])
      {
        case VISIBLE:
        case OBSTACLE:
          return false;
        default:
          continue;
      }
    }
  }
  return true;
}

geometry_msgs::Point ShadowLayer::createPointFromIndex(unsigned int idx)
{
  geometry_msgs::Point pt;
  unsigned int mx, my;
  indexToCells(idx, mx, my);
  mapToWorld(mx, my, pt.x, pt.y);
  return pt;
}

void ShadowLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                         double* min_x, double* min_y, double* max_x, double* max_y)
{
  double dx = wx-ox, dy = wy-oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ShadowLayer::reset()
{
    resetMaps();
    current_ = true;
}

}  // namespace costmap_2d
