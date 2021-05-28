/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
 *  Copyright (c) 2017, 6 River Systems, Inc.
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
 *         Daniel Grieneisen
 *********************************************************************/
#include <costmap_2d/static_layer_with_inflation.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <srslib_timing/ScopedTimingSampleRecorder.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_2d::StaticLayerWithInflation, costmap_2d::CostmapLayer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

StaticLayerWithInflation::StaticLayerWithInflation() : dsrv_(NULL),
  inflation_layer_(NULL), secondary_inflation_layer_(NULL), needs_reinflation_(true), timingDataRecorder_("slwi"),
  static_map_(nullptr),
  plugin_loader_("costmap_2d", "costmap_2d::Layer") {}

StaticLayerWithInflation::~StaticLayerWithInflation()
{
  if (dsrv_)
    delete dsrv_;
}

void StaticLayerWithInflation::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("first_map_only", first_map_only_, false);
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);

  nh.param("track_unknown_space", track_unknown_space_, true);
  nh.param("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  nh.param("inflation_layer_type", inflation_layer_type_, std::string("costmap_2d::VoronoiInflationLayer"));
  nh.param("secondary_inflation_layer_type", secondary_inflation_layer_type_, std::string(""));

  nh.param("impassible", impassible_, true);


  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;

  // Create a new inflation layer
  if (inflation_layer_)
  {
    // delete inflation_layer_;
    inflation_layer_->reset();
  }

  // Create a new secondary inflation layer
  if (secondary_inflation_layer_)
  {
    // delete inflation_layer_;
    secondary_inflation_layer_->reset();
  }
  
  // resubscribe if it is unsubscribed or the map topic has changed
  if (!map_sub_ || map_sub_.getTopic() != ros::names::resolve(map_topic))
  {
    // we'll subscribe to the latched topic that the map server uses
    ROS_INFO("Requesting the map on topic %s", map_topic.c_str());
    map_sub_ = g_nh.subscribe(map_topic, 1, &StaticLayerWithInflation::incomingMap, this);
    map_received_ = false;
    has_updated_data_ = false;

    ros::Rate r(10);
    while (!map_received_ && g_nh.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());

    if (subscribe_to_updates_)
    {
      ROS_INFO("Subscribing to updates");
      map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &StaticLayerWithInflation::incomingUpdate, this);

    }
  }
  else
  {
    has_updated_data_ = true;
  }

  if (dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &StaticLayerWithInflation::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  layer_initialized_ = true;
}

void StaticLayerWithInflation::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

void StaticLayerWithInflation::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling())
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }
  // Either way, match the size in the inflation layer so it gets the correct resolution
  if (inflation_layer_)
  {
    inflation_layer_->matchSize();
  }
  else
  {
    ROS_WARN("Could not match size for inflation layer because there isn't one.");
  }
  if (secondary_inflation_layer_)
  {
    secondary_inflation_layer_->matchSize();
  }
}

unsigned char StaticLayerWithInflation::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void StaticLayerWithInflation::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  Costmap2D* master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
      master->getSizeInCellsY() != size_y ||
      master->getResolution() != new_map->info.resolution ||
      master->getOriginX() != new_map->info.origin.position.x ||
      master->getOriginY() != new_map->info.origin.position.y ||
      !layered_costmap_->isSizeLocked()))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y, true);
  }
  else if (size_x_ != size_x || size_y_ != size_y ||
           resolution_ != new_map->info.resolution ||
           origin_x_ != new_map->info.origin.position.x ||
           origin_y_ != new_map->info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer
    ROS_INFO("Resizing static layer to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    resizeMap(size_x, size_y, new_map->info.resolution,
              new_map->info.origin.position.x, new_map->info.origin.position.y);
  }

  if (static_map_)
  {
    delete static_map_;
  }

  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;
  static_map_ = new unsigned char[height_ * width_];

  unsigned int index = 0;
  // initialize the costmap with static data
  for (unsigned int i = 0; i < height_; ++i)
  {
    for (unsigned int j = 0; j < width_; ++j)
    {
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);
      static_map_[index] = interpretValue(value);
      ++index;
    }
  }

  map_frame_ = new_map->header.frame_id;

  if (!inflation_layer_){
    ROS_INFO("Creating inflation layer in static layer.");

    boost::shared_ptr<Layer> plugin = plugin_loader_.createInstance(inflation_layer_type_);
    // inflation_layer_ = boost::static_pointer_cast<VoronoiInflationLayer>(plugin);
    inflation_layer_ = plugin;
    inflation_layer_->initialize(layered_costmap_, name_ + "/inflation", tf_);
  }
  if (!secondary_inflation_layer_ && secondary_inflation_layer_type_.size() > 0){
    ROS_INFO("Creating secondary inflation layer in static layer.");

    boost::shared_ptr<Layer> plugin = plugin_loader_.createInstance(secondary_inflation_layer_type_);
    // inflation_layer_ = boost::static_pointer_cast<VoronoiInflationLayer>(plugin);
    secondary_inflation_layer_ = plugin;
    secondary_inflation_layer_->initialize(layered_costmap_, name_ + "/secondary_inflation", tf_);
  }
  needs_reinflation_ = true;
  // shutdown the map subscrber if firt_map_only_ flag is on
  if (first_map_only_)
  {
    ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
    map_sub_.shutdown();
  }
}

void StaticLayerWithInflation::updateCostmapFromStaticMap()
{
  ROS_DEBUG("Copying static map into slwi costmap");
  memcpy(costmap_, static_map_, sizeof(unsigned char) * height_ * width_);
}

void StaticLayerWithInflation::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height ; y++)
  {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width ; x++)
    {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }
  x_ = update->x;
  y_ = update->y;
  width_ = update->width;
  height_ = update->height;
  has_updated_data_ = true;
}

void StaticLayerWithInflation::activate()
{
  onInitialize();
}

void StaticLayerWithInflation::deactivate()
{
  layer_initialized_ = false;
  map_sub_.shutdown();
  if (subscribe_to_updates_)
    map_update_sub_.shutdown();
}

void StaticLayerWithInflation::reset()
{
  layer_initialized_ = false;
  if (first_map_only_)
  {
    has_updated_data_ = true;
  }
  else
  {
    onInitialize();
  }
}

void StaticLayerWithInflation::reinitialize()
{
  if (first_map_only_)
  {
    has_updated_data_ = true;
  }
  else
  {
    deactivate();
    activate();
  }
}

void StaticLayerWithInflation::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{

  if (inflation_layer_)
  {
    if (inflation_layer_->needsUpdate())
    {
      needs_reinflation_ = true;
    }
    inflation_layer_->updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  }
  if (secondary_inflation_layer_)
  {
    if (secondary_inflation_layer_->needsUpdate())
    {
      needs_reinflation_ = true;
    }
    secondary_inflation_layer_->updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  }

  if( !layered_costmap_->isRolling() ){
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
      return;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;

  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false;
}

double StaticLayerWithInflation::getDistanceFromStaticMap(double px, double py)
{
  if (layered_costmap_->isRolling())
  {
    // If it is rolling, convert the points.
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    tf::StampedTransform transform;
    if (!getTransform(transform, map_frame_, global_frame_))
    {
      return -1.0;
    }

    tf::Point p(px, py, 0);
    p = transform(p);
    ROS_DEBUG("Original: %f, %f - transformed: %f, %f", px, py, p.x(), p.y());
    px = p.x();
    py = p.y();
  }

  unsigned int mx = 0;
  unsigned int my = 0;

  auto distance_map = inflation_layer_->getDistancesFromStaticMap();
  if (distance_map && worldToMap(px, py, mx, my))
  {
    return (*distance_map)[getIndex(mx, my)];
  }
  else
  {
    return -1.0;
  }
}

int StaticLayerWithInflation::getAngleFromStaticMap(double px, double py)
{
  if (layered_costmap_->isRolling())
  {
    // If it is rolling, convert the points.
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    tf::StampedTransform transform;
    if (!getTransform(transform, map_frame_, global_frame_))
    {
      return -1.0;
    }

    tf::Point p(px, py, 0);
    p = transform(p);
    ROS_DEBUG("Original: %f, %f - transformed: %f, %f", px, py, p.x(), p.y());
    px = p.x();
    py = p.y();
  }

  unsigned int mx = 0;
  unsigned int my = 0;

  auto angle_map = inflation_layer_->getAnglesFromStaticMap();
  if (angle_map && worldToMap(px, py, mx, my))
  {
    return (*angle_map)[getIndex(mx, my)];
  }
  else
  {
    return -10;
  }
}


void StaticLayerWithInflation::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  srs::ScopedTimingSampleRecorder stsr_update_costs(timingDataRecorder_.getRecorder("-updateCosts", 1));
  
  if (!map_received_)
    return;

  if (inflation_layer_ && needs_reinflation_)
  {
    updateCostmapFromStaticMap();
    inflation_layer_->updateCosts(*this, x_, y_, width_, height_);

    if (secondary_inflation_layer_) 
    {
      secondary_inflation_layer_->updateCosts(*this, x_, y_, width_, height_);
    }
    needs_reinflation_ = false;
  }

  if (!layered_costmap_->isRolling())
  {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    if (!use_maximum_)
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    else
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  else
  {
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    tf::StampedTransform transform;
    if (!getTransform(transform, map_frame_, global_frame_))
    {
      return;
    }
    // Copy map data given proper transformations
    for (unsigned int i = min_i; i < max_i; ++i)
    {
      for (unsigned int j = min_j; j < max_j; ++j)
      {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        tf::Point p(wx, wy, 0);
        p = transform(p);
        // Set master_grid with cell from map
        if (worldToMap(p.x(), p.y(), mx, my))
        {
          if (!use_maximum_)
            master_grid.setCost(i, j, getCost(mx, my));
          else
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
        }
      }
    }
  }
}

bool StaticLayerWithInflation::getTransform(tf::StampedTransform& transform,
  std::string from_frame_id, std::string to_frame_id)
{
  try
  {
    if (tf_->waitForTransform(from_frame_id, to_frame_id, ros::Time(0), ros::Duration(0.1)))
    {
      tf_->lookupTransform(from_frame_id, to_frame_id, ros::Time(0), transform);

      return true;
    }
    else
    {
      ROS_WARN_THROTTLE(1.0, "Could not find transform from %s to %s", from_frame_id.c_str(), to_frame_id.c_str());
      return false;
    }
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  return true;
}

void StaticLayerWithInflation::onFootprintChanged()
{
  if (inflation_layer_)
  {
    boost::static_pointer_cast<Layer>(inflation_layer_)->onFootprintChanged();
    if (secondary_inflation_layer_)
    {
      boost::static_pointer_cast<Layer>(secondary_inflation_layer_)->onFootprintChanged();
    }
    needs_reinflation_ = true;
  } else {
    ROS_WARN("Don't have an inflation layer to footprint change.");
  }
}

}  // namespace costmap_2d
