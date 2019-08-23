/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
#include <costmap_2d/obstruction_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <srslib_timing/ScopedTimingSampleRecorder.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_2d::ObstructionLayer, costmap_2d::CostmapLayer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace costmap_2d
{

void ObstructionLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();

  bool track_unknown_space;
  nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
  if (track_unknown_space)
    default_value_ = NO_INFORMATION;
  else
    default_value_ = FREE_SPACE;

  ObstructionLayer::matchSize();
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();
  double transform_tolerance;
  nh.param("transform_tolerance", transform_tolerance, 0.2);

  std::string topics_string;
  // get the topics that we'll subscribe to from the parameter server
  nh.param("observation_sources", topics_string, std::string(""));
  ROS_INFO("Subscribed to Topics: %s", topics_string.c_str());

  nh.param<float>("dynamic_kernel_inflation", dynamic_kernel_inflation_, 0.05);

  timingDataRecorder_ = srs::MasterTimingDataRecorder(global_frame_ + "-" + name_);

  // get our tf prefix
  ros::NodeHandle prefix_nh;
  const std::string tf_prefix = tf::getPrefixParam(prefix_nh);

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source)
  {
    ros::NodeHandle source_node(nh, source);

    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;

    source_node.param("topic", topic, source);
    source_node.param("sensor_frame", sensor_frame, std::string(""));
    source_node.param("observation_persistence", observation_keep_time, 0.0);
    source_node.param("expected_update_rate", expected_update_rate, 0.0);
    source_node.param("data_type", data_type, std::string("PointCloud"));
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
    source_node.param("inf_is_valid", inf_is_valid, false);
    source_node.param("clearing", clearing, false);
    source_node.param("marking", marking, true);

    if (!sensor_frame.empty())
    {
      sensor_frame = tf::resolve(tf_prefix, sensor_frame);
    }

    if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
    {
      ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
      throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
    }

    std::string raytrace_range_param_name, min_raytrace_range_param_name, obstacle_range_param_name;

    // get the obstacle range for the sensor
    double obstacle_range = 2.5;
    if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
    {
      source_node.getParam(obstacle_range_param_name, obstacle_range);
    }

    // get the raytrace range for the sensor
    double raytrace_range = 3.0;
    if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
    {
      source_node.getParam(raytrace_range_param_name, raytrace_range);
    }

    // get the minimum raytrace range for the sensor
    double min_raytrace_range = 0.0;
    if (source_node.searchParam("min_raytrace_range", min_raytrace_range_param_name))
    {
      source_node.getParam(min_raytrace_range_param_name, min_raytrace_range);
    }

    ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
              sensor_frame.c_str());

    // create an observation buffer
    observation_buffers_.push_back(
        boost::shared_ptr < ObservationBuffer
            > (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                     max_obstacle_height, obstacle_range, raytrace_range, min_raytrace_range, *tf_, global_frame_,
                                     sensor_frame, transform_tolerance)));

    // check if we'll add this buffer to our marking observation buffers
    if (marking)
      marking_buffers_.push_back(observation_buffers_.back());

    // check if we'll also add this buffer to our clearing observation buffers
    if (clearing)
      clearing_buffers_.push_back(observation_buffers_.back());

    ROS_DEBUG(
        "Created an observation buffer for source %s, topic %s, global frame: %s, "
        "expected update rate: %.2f, observation persistence: %.2f",
        source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

    // create a callback for the topic
    if (data_type == "LaserScan")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
          > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

      boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan>
          > filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50));

      if (inf_is_valid)
      {
        filter->registerCallback(
            boost::bind(&ObstructionLayer::laserScanValidInfCallback, this, _1, observation_buffers_.back()));
      }
      else
      {
        filter->registerCallback(
            boost::bind(&ObstructionLayer::laserScanCallback, this, _1, observation_buffers_.back()));
      }

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);

      observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
    }
    else if (data_type == "PointCloud")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));

      if (inf_is_valid)
      {
       ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud>
          > filter(new tf::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50));
      filter->registerCallback(
          boost::bind(&ObstructionLayer::pointCloudCallback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }
    else
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

      if (inf_is_valid)
      {
       ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud2>
          > filter(new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50));
      filter->registerCallback(
          boost::bind(&ObstructionLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }

    if (sensor_frame != "")
    {
      std::vector < std::string > target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(sensor_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);
    }
  }

  obstruction_publisher_ = nh.advertise<costmap_2d::ObstructionListMsg>("obstructions", 1);

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);
}

void ObstructionLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ObstructionPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::ObstructionPluginConfig>::CallbackType cb = boost::bind(
      &ObstructionLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

ObstructionLayer::~ObstructionLayer()
{
    if (dsrv_)
    {
      delete dsrv_;
    }
}
void ObstructionLayer::reconfigureCB(costmap_2d::ObstructionPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  max_obstacle_height_ = config.max_obstacle_height;
  obstruction_half_life_ = ros::Duration(config.obstruction_half_life);
  num_obstruction_levels_ = config.num_obstruction_levels;
  enable_decay_ = config.enable_decay;

  distance_threshold_ = config.distance_threshold;

  dynamic_inflation_type_ = config.dynamic_inflation_type;
  dynamic_inflation_radius_ = config.dynamic_inflation_radius;
  dynamic_cost_scaling_factor_ = config.dynamic_cost_scaling_factor;

  pseudostatic_inflation_type_ = config.pseudostatic_inflation_type;
  pseudostatic_inflation_radius_ = config.pseudostatic_inflation_radius;
  pseudostatic_cost_scaling_factor_ = config.pseudostatic_cost_scaling_factor;

  dynamic_kernel_inflation_ = config.dynamic_kernel_inflation;

  generateKernels();
}

void ObstructionLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                      const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // project the laser into a point cloud
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message->header;

  // project the scan into a point cloud
  try
  {
    projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
             ex.what());
    projector_.projectLaser(*message, cloud);
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void ObstructionLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
                                              const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // Filter positive infinities ("Inf"s) to max_range.
  float epsilon = 0.0001;  // a tenth of a millimeter
  sensor_msgs::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++)
  {
    float range = message.ranges[ i ];
    if (!std::isfinite(range) && range > 0)
    {
      message.ranges[ i ] = message.range_max - epsilon;
    }
  }

  // project the laser into a point cloud
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message.header;

  // project the scan into a point cloud
  try
  {
    projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s",
             global_frame_.c_str(), ex.what());
    projector_.projectLaser(message, cloud);
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void ObstructionLayer::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                                               const boost::shared_ptr<ObservationBuffer>& buffer)
{
  sensor_msgs::PointCloud2 cloud2;

  if (!sensor_msgs::convertPointCloudToPointCloud2(*message, cloud2))
  {
    ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud2);
  buffer->unlock();
}

void ObstructionLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

void ObstructionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  srs::ScopedTimingSampleRecorder stsr_update_bounds(timingDataRecorder_.getRecorder("-updateBounds", 1));

  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!enabled_)
    return;
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  current = current && getMarkingObservations(observations);

  // get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  // update the global current status
  current_ = current;

  // update the static distance map
  static_distance_map_ = layered_costmap_->getDistancesFromStaticMap();

  // raytrace freespace
  ROS_DEBUG("In update bounds.  Have %zu clearing and %zu marking obs.", clearing_observations.size(), observations.size());
  for (unsigned int i = 0; i < observations.size(); ++i)
  {
    checkObservation(observations[i], min_x, min_y, max_x, max_y);
  }

  for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  // Update obstructions
  updateObstructions(min_x, min_y, max_x, max_y);
  ROS_DEBUG_NAMED("obstruction", "Updating bounds to %f, %f, %f, %f", *min_x, *min_y, *max_x, *max_y);

  /// @todo Add this if we want it?
  // updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}


void ObstructionLayer::updateObstructions(double* min_x, double* min_y, double* max_x, double* max_y)
{
  srs::ScopedTimingSampleRecorder lockwait(timingDataRecorder_.getRecorder("-updateObs", 1));

  // First, make sure the kernels have been created.
  if (kernels_.empty())
  {
    ROS_WARN("No kernels.  Cannot update obstructions.");
    return;
  }

  // Clear the stored obstruction messages
  auto obstruction_msgs = std::make_shared<std::vector<ObstructionMsg>>();

  ObstructionListMsg msg;
  msg.costmap_name = name_;

  boost::unique_lock<mutex_t> lock(*getMutex());

  ros::Time now = ros::Time::now();
  // Iterate over the obstructions
  auto iter = obstruction_list_.begin();
  while (iter != obstruction_list_.end())
  {
    auto obs = *iter;
    // Clear the recently seen flag
    if (obs->seen_this_cycle_)
    {
      obs->seen_this_cycle_ = false;
    }

    obs->radius_ = kernels_[obs->type_][obs->level_]->radius_;
    obs->max_cost_ = kernels_[obs->type_][obs->level_]->max_cost_;

    // Check to see if the obstruction is still on the map - if not it should get cleared.
    unsigned int dummyx, dummyy;
    if (!worldToMap(obs->x_, obs->y_, dummyx, dummyy))
    {
      ROS_DEBUG("Removing obstruction which has fallen off the map");
      obs->cleared_ = true;
    }

    // Update level (and radius) if need be
    ROS_DEBUG("obs sight %f, level %f", obs->last_sighting_time_.toSec(), obs->last_level_time_.toSec());
    if (now - obs->last_level_time_ > obstruction_half_life_)
    {
      ROS_DEBUG("Obstruction level updated");
      obs->level_++;
      obs->last_level_time_= now;
      if (obs->level_ >= num_obstruction_levels_)
      {
        ROS_DEBUG("Clearing out an old observation.");
        obs->cleared_ = true;
      }
      else
      {
        obs->radius_ = std::max(kernels_[obs->type_][obs->level_]->radius_, obs->radius_);
        obs->max_cost_ = kernels_[obs->type_][obs->level_]->max_cost_;

        obs->updated_ = true;
      }
    }
    // If the obstruction has changed to static, clear it.
    if (obs->type_ == ObstructionType::STATIC)
    {
      obs->cleared_ = true;
    }

    // Touch with size extent
    touchWithRadius(obs->x_, obs->y_, obs->radius_, min_x, min_y, max_x, max_y);

    // Collect for publishing
    msg.obstructions.push_back(ObstructionAdapter::obstructionToMsg(*obs));
    obstruction_msgs->push_back(msg.obstructions.back());

    // Remove cleared ones
    if (obs->cleared_)
    {
      ROS_DEBUG("Removing obstruction with radius %f at %f, %f", obs->radius_, obs->x_, obs->y_);
      iter = obstruction_list_.erase(iter);
    }
    else
    {
      ++iter;
    }
  }

  // Publish obstructions
  obstruction_publisher_.publish(msg);

  {
    std::lock_guard<std::mutex> lock(obstruction_lock_);
    obstruction_msgs_ = obstruction_msgs;
  }
}

void ObstructionLayer::checkObservation(const Observation& obs, double* min_x, double* min_y, double* max_x, double* max_y)
{
  const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

  double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

  for (unsigned int i = 0; i < cloud.points.size(); ++i)
  {
    double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

    // if the obstacle is too high or too far away from the robot we won't add it
    if (pz > max_obstacle_height_)
    {
      ROS_DEBUG("The point is too high");
      continue;
    }

    // compute the squared distance from the hitpoint to the pointcloud's origin
    double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
        + (pz - obs.origin_.z) * (pz - obs.origin_.z);

    // if the point is far enough away... we won't consider it
    if (sq_dist >= sq_obstacle_range)
    {
      ROS_DEBUG("The point is too far away");
      continue;
    }

    // now we need to compute the map coordinates for the observation
    unsigned int mx, my;
    if (!worldToMap(px, py, mx, my))
    {
      ROS_DEBUG("Computing map coords failed");
      continue;
    }

    unsigned int index = getIndex(mx, my);

    ObstructionType type = getObstructionType(px, py);
    // Check to see if there is already an obstruction there
    auto obstruction = obstruction_map_[index].lock();
    if (obstruction)
    {
      // Touch it.
      ROS_DEBUG("Touching obstacle at %f, %f, %d", px, py, index);
      obstruction->touch(type);
    }
    else
    {
      // Check to see if it is in the master grid already.
      if (type != ObstructionType::STATIC)
      {
        ROS_DEBUG("Creating new obstacle at %f, %f, %d, type: %d", px, py, index, static_cast<int>(type));
        // Create a new one.  Store in list and map.
        auto obs = std::make_shared<Obstruction>(px, py, type, cloud.header.frame_id, getShortName());
        obstruction_list_.push_back(obs);
        obstruction_map_[index] = obs;
      }
      else
      {
        ROS_DEBUG("Obstacle is static.");
      }
    }
  }
}

ObstructionType ObstructionLayer::getObstructionType(double px, double py)
{
  double distance_from_static = layered_costmap_->getDistanceFromStaticMap(px, py) * resolution_;

  ROS_DEBUG("Getting type at: %f, %f.  distance: %f, thresh: %f",
    px, py, distance_from_static, distance_threshold_);

  if (distance_from_static == 0.0)
  {
    return ObstructionType::STATIC;
  }
  else if (distance_from_static > 0.0 && distance_from_static < distance_threshold_)
  {
    return ObstructionType::PSEUDOSTATIC;
  }
  else
  {
    return ObstructionType::DYNAMIC;
  }
}

void ObstructionLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{
    /// @todo do something if needed...

    // if (!footprint_clearing_enabled_) return;
    // transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    // for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    // {
    //   touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    // }
}

void ObstructionLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  srs::ScopedTimingSampleRecorder stsr_update_costs(timingDataRecorder_.getRecorder("-updateCosts", 1));

  /// @todo Add this?
  // if (footprint_clearing_enabled_)
  // {
  //   setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
  // }

  // Iterate over all of the obstructions
  boost::unique_lock<mutex_t> lock(*getMutex());

  ROS_DEBUG_NAMED("obstruction", "Updating costs");
  if (kernels_.empty())
  {
    ROS_WARN("Cannot apply kernels because none exist.");
    return;
  }

  for (auto obs_ptr : obstruction_list_)
  {
    applyKernelAtLocation(kernels_[obs_ptr->type_][obs_ptr->level_], obs_ptr->x_, obs_ptr->y_, master_grid);
  }
}

void ObstructionLayer::applyKernelAtLocation(std::shared_ptr<Kernel> kernel, float x, float y, Costmap2D& master_grid)
  {
    if (!kernel)
    {
      ROS_WARN("Invalid pointer when trying to apply kernel.");
      return;
    }

    if (std::fabs(kernel->resolution_ - master_grid.getResolution()) > 0.001)
    {
      ROS_WARN("Kernel grid resolution mismatch.  Kernel: %f, grid: %f",
        kernel->resolution_, master_grid.getResolution());
      return;
    }

    // Get the grid location of the point.
    int mx, my;
    master_grid.worldToMapNoBounds(x, y, mx, my);

    unsigned int grid_x_size = master_grid.getSizeInCellsX();
    unsigned int grid_y_size = master_grid.getSizeInCellsY();

    int center_x = kernel->size_x_ / 2 + 1;
    int center_y = kernel->size_y_ / 2 + 1;

    unsigned char* grid = master_grid.getCharMap();

    ////////
    // Calculate yy start and yy end
    int yy_start = 0;
    if (my - center_y < 0)
    {
      yy_start = center_y - my;
    }

    int yy_end = kernel->size_y_;
    if (my + center_y > grid_y_size)
    {
      yy_end = center_y + my - grid_y_size;
    }
    /////
    ////////
    // Calculate xx start and xx end
    int xx_start = 0;
    if (mx - center_x < 0)
    {
      xx_start = center_x - mx;
    }

    int xx_end = kernel->size_x_;
    if (mx + center_x > grid_x_size)
    {
      xx_end = center_x + mx - grid_x_size;
    }
    /////
    // Now try to apply it.
    for (int yy = yy_start; yy < yy_end; ++yy)
    {
      int grid_y = yy - center_y + my;

      unsigned int kern_x_start = kernel->size_x_ * yy;
      unsigned int grid_x_start = grid_x_size * grid_y;

      for (int xx = xx_start; xx < xx_end; ++xx)
      {
        int grid_x = xx - center_x + mx;
        // Point is valid, max it in.
        unsigned int grid_idx = grid_x_start + grid_x;
        unsigned char grid_cost = grid[grid_idx];
        unsigned char kernel_cost = kernel->values_[kern_x_start + xx];

        bool kernel_cost_beyond_obstacle = kernel_cost != INSCRIBED_INFLATED_OBSTACLE
                || kernel_cost != LETHAL_OBSTACLE;
        bool ignore_kernel_cost = kernel->ignore_freespace_
              && grid_cost == FREE_SPACE
              && kernel_cost_beyond_obstacle;

        if (!ignore_kernel_cost && (grid_cost < kernel_cost || grid_cost == NO_INFORMATION))
        {
          grid[grid_idx] = kernel_cost;
        }
      }
    }
  }

void ObstructionLayer::addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.push_back(obs);
  if (clearing)
    static_clearing_observations_.push_back(obs);
}

void ObstructionLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.clear();
  if (clearing)
    static_clearing_observations_.clear();
}

bool ObstructionLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const
{
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
  {
    marking_buffers_[i]->lock();
    marking_buffers_[i]->getObservations(marking_observations);
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  }
  marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

bool ObstructionLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const
{
  bool current = true;
  // get the clearing observations
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
  {
    clearing_buffers_[i]->lock();
    clearing_buffers_[i]->getObservations(clearing_observations);
    current = clearing_buffers_[i]->isCurrent() && current;
    clearing_buffers_[i]->unlock();
  }
  clearing_observations.insert(clearing_observations.end(),
                              static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

void ObstructionLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
                                              double* max_x, double* max_y)
{
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  pcl::PointCloud < pcl::PointXYZ > cloud = *(clearing_observation.cloud_);

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

  double min_raytrace_dist = clearing_observation.min_raytrace_range_;

  touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  for (unsigned int i = 0; i < cloud.points.size(); ++i)
  {
    double wx = cloud.points[i].x;
    double wy = cloud.points[i].y;

    double a = wx - ox;
    double b = wy - oy;
    double ray_length = sqrt(a * a + b * b);

    // skip if the ray length is shorter than min_raytrace_range
    if(ray_length < min_raytrace_dist)
    {
      continue;
    }

    // calculate raytrace starting point
    // the raytrace range should be (rx, ry) -> (wx, wy)
    // if min_raytrace_range is 0.0 or not specified, the range will become (ox, oy) -> (wx, wy)
    double rx = ox + min_raytrace_dist * a / ray_length;
    double ry = oy + min_raytrace_dist * b / ray_length;

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

    unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
    ClearObstructionCell marker(obstruction_map_);
    // and finally... we can execute our trace to clear obstacles along that line
    raytraceLine(marker, rx_map, ry_map, x1, y1, cell_raytrace_range);
  }
}

void ObstructionLayer::activate()
{
  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->subscribe();
  }

  for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
  {
    if (observation_buffers_[i])
      observation_buffers_[i]->resetLastUpdated();
  }
}
void ObstructionLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->unsubscribe();
  }
}

void ObstructionLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                         double* min_x, double* min_y, double* max_x, double* max_y)
{
  double dx = wx-ox, dy = wy-oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ObstructionLayer::reset()
{
  deactivate();
  resetMapsAndClearObstructions();
  current_ = true;
  activate();
}

void ObstructionLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  boost::unique_lock<mutex_t> lock(*getMutex());

  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = int((new_origin_x - origin_x_) / resolution_);
  cell_oy = int((new_origin_y - origin_y_) / resolution_);

  // compute the associated world coordinates for the origin cell
  // because we want to keep things grid-aligned
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
  std::weak_ptr<Obstruction>* local_obs_map = new std::weak_ptr<Obstruction>[cell_size_x * cell_size_y];

  /// @todo Remove map region copying in the future.
  // copy the local window in the costmap to the local map
  copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);
  copyMapRegionElementwise(obstruction_map_, lower_left_x, lower_left_y, size_x_, local_obs_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

  // now we'll set the costmap to be completely unknown if we track unknown space
  resetMaps();

  // update the origin with the appropriate world coordinates
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  // compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  // now we want to copy the overlapping information back into the map, but in its new location
  copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);
  copyMapRegionElementwise(local_obs_map, 0, 0, cell_size_x, obstruction_map_, start_x, start_y, size_x_, cell_size_x, cell_size_y);

  // make sure to clean up
  delete[] local_map;
  delete[] local_obs_map;
}

void ObstructionLayer::initMaps(unsigned int size_x, unsigned int size_y)
{
  boost::unique_lock<mutex_t> lock(*getMutex());
  delete[] costmap_;
  delete[] obstruction_map_;
  costmap_ = new unsigned char[size_x * size_y];
  obstruction_map_ = new std::weak_ptr<Obstruction>[size_x * size_y];
}

void ObstructionLayer::resetMapsAndClearObstructions()
{
  boost::unique_lock<mutex_t> lock(*getMutex());
  memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
  for (size_t k = 0; k < size_x_ * size_y_; ++k)
  {
    auto obs = obstruction_map_[k].lock();
    if (obs)
    {
      obs->cleared_ = true;
      obstruction_map_[k].reset();
    }
  }
}

void ObstructionLayer::resetMaps()
{
  boost::unique_lock<mutex_t> lock(*getMutex());
  memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
  for (size_t k = 0; k < size_x_ * size_y_; ++k)
  {
    auto obs = obstruction_map_[k].lock();
    if (obs)
    {
      obstruction_map_[k].reset();
    }
  }
}

void ObstructionLayer::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
  boost::unique_lock<mutex_t> lock(*getMutex());
  unsigned int len = xn - x0;
  for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_)
  {
    memset(costmap_ + y, default_value_, len * sizeof(unsigned char));
    auto obs = obstruction_map_[y].lock();
    if (obs)
    {
      obs->cleared_ = true;
      obstruction_map_[y].reset();
    }
  }
}

void ObstructionLayer::deleteMaps()
{
  // clean up data
  boost::unique_lock<mutex_t> lock(*getMutex());
  delete[] costmap_;
  delete[] obstruction_map_;
  costmap_ = NULL;
  obstruction_map_ = NULL;
}

void ObstructionLayer::onFootprintChanged()
{
  generateKernels();
  ROS_DEBUG("Got a footprint change in obstruction layer.");
}

void ObstructionLayer::generateKernels()
{
  boost::unique_lock < boost::recursive_mutex > lock(*getMutex());

  // Generate the dynamic obstacle kernels
  generateKernelsByType(ObstructionType::DYNAMIC, dynamic_inflation_radius_,
    dynamic_cost_scaling_factor_, dynamic_inflation_type_);

  // Generate the pseudostatic obstacle kernels
  generateKernelsByType(ObstructionType::PSEUDOSTATIC, pseudostatic_inflation_radius_,
    pseudostatic_cost_scaling_factor_, pseudostatic_inflation_type_);

  // Generate the static kernels (to avoid segfaults)
  generateKernelsByType(ObstructionType::STATIC, 0, 1.0, EXPONENTIAL_INFLATION);
}

void ObstructionLayer::generateKernelsByType(ObstructionType type,
  float inflation_radius, float cost_scaling_factor, int inflation_type)
{
  // Do some checks
  ROS_DEBUG("Generating kerns with: number of levels %d, inflation_radius %f, cost_scaling_factor %f, resolution %f",
    num_obstruction_levels_, inflation_radius, cost_scaling_factor, resolution_);
  if (num_obstruction_levels_ == 0)
  {
    ROS_WARN("Cannot set up kernels without obstruction levels");
    return;
  }

  bool ignore_freespace = (type == ObstructionType::PSEUDOSTATIC);
  double dynamic_kernel_inflation = (type == ObstructionType::DYNAMIC) ? dynamic_kernel_inflation_ : 0.0;

  // Create all the new kernels that are needed.
  kernels_[type] = std::vector<std::shared_ptr<Kernel>>();
  kernels_[type].reserve(num_obstruction_levels_);

  // Start.
  for (unsigned int k = 0; k < num_obstruction_levels_; ++k)
  {
    switch (inflation_type) {
      case EXPONENTIAL_INFLATION:
        kernels_[type].push_back(KernelFactory::generateRadialInflationKernel((LETHAL_OBSTACLE / (k + 1)),
         (INSCRIBED_INFLATED_OBSTACLE / (k + 1)), layered_costmap_->getInscribedRadius(),
         inflation_radius, cost_scaling_factor, resolution_, ignore_freespace, dynamic_kernel_inflation));
        break;
      case TRINOMIAL_INFLATION:
        kernels_[type].push_back(KernelFactory::generateTrinomialRadialInflationKernel((LETHAL_OBSTACLE / (k + 1)),
         (INSCRIBED_INFLATED_OBSTACLE / (k + 1)), layered_costmap_->getInscribedRadius(),
         inflation_radius, cost_scaling_factor, resolution_, ignore_freespace, dynamic_kernel_inflation));
        break;
      default:
        ROS_ERROR("Kernel type unknown for obstruction layer.  Defaulting to exponential inflation");
        kernels_[type].push_back(KernelFactory::generateRadialInflationKernel((LETHAL_OBSTACLE / (k + 1)),
         (INSCRIBED_INFLATED_OBSTACLE / (k + 1)), layered_costmap_->getInscribedRadius(),
         inflation_radius, cost_scaling_factor, resolution_, ignore_freespace, dynamic_kernel_inflation));
        break;
    }
  }
}

void ObstructionLayer::clearGridCell(unsigned int x, unsigned int y)
{
  int index = getIndex(x,y);
  auto obs = obstruction_map_[index].lock();
  if (obs)
  {
    obs->cleared_ = true;
    obstruction_map_[index].reset();
  }
}

}  // namespace costmap_2d
