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
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/axis_aligned_bounding_box.h>
#include <costmap_2d/layer_actions.h>


PLUGINLIB_EXPORT_CLASS(costmap_2d::ObstacleLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace costmap_2d
{

void ObstacleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();

  // These values are read from dynamic reconfigure. To change the default
  // values you can edit costmap_2d/cfg/ObstaclePlugin.cfg
  obstacle_lifespan_ = 0.0;          // seconds
  obstacle_keep_radius_ = 0.0;       // meters
  use_forgetful_version_ = true;     // flag
  last_known_enabled_ = false;
  clear_obstacle_memory_ = false;    // flag
  
  // Initial pose confidence and threshold before we remember new data
  // Threshold default in costmap_2d/cfg/ObstaclePlugin.cfg
  pose_confidence_ = 0;
  pose_confidence_threshold_ = 1;
  
  std::string pose_confidence_topic_name;
  nh.param<std::string>("pose_confidence_topic_name", pose_confidence_topic_name, "slam/localization_score");
  pose_confidence_sub_ = g_nh.subscribe(pose_confidence_topic_name, 1, &ObstacleLayer::poseConfidenceCallback, this);

  bool track_unknown_space;
  nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
  if (track_unknown_space)
    default_value_ = NO_INFORMATION;
  else
    default_value_ = FREE_SPACE;

  ObstacleLayer::matchSize();
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();
  double transform_tolerance;
  nh.param("transform_tolerance", transform_tolerance, 0.2);

  std::string topics_string;
  // get the topics that we'll subscribe to from the parameter server
  nh.param("observation_sources", topics_string, std::string(""));
  ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

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
    bool inf_is_valid, clearing, marking, add_max_range;

    source_node.param("topic", topic, source);
    source_node.param("sensor_frame", sensor_frame, std::string(""));
    source_node.param("observation_persistence", observation_keep_time, 0.0);
    source_node.param("expected_update_rate", expected_update_rate, 0.0);
    source_node.param("data_type", data_type, std::string("PointCloud"));
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
    source_node.param("inf_is_valid", inf_is_valid, false);
    source_node.param("add_max_range", add_max_range, true);
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

    std::string raytrace_range_param_name, obstacle_range_param_name;

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

    ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
              sensor_frame.c_str());

    // create an observation buffer
    observation_buffers_.push_back(
        boost::shared_ptr < ObservationBuffer
            > (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                     max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
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
            boost::bind(&ObstacleLayer::laserScanValidInfCallback, this, _1, observation_buffers_.back()));
      }
      else
      {
        filter->registerCallback(
            boost::bind(&ObstacleLayer::laserScanCallback, this, _1, observation_buffers_.back(), add_max_range));
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
          boost::bind(&ObstacleLayer::pointCloudCallback, this, _1, observation_buffers_.back()));

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
          boost::bind(&ObstacleLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));

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

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);
}

void ObstacleLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>::CallbackType cb = boost::bind(
      &ObstacleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

ObstacleLayer::~ObstacleLayer()
{
    if (dsrv_)
        delete dsrv_;
}

void ObstacleLayer::clearObstacleMemory()
{
  // Set a flag that will cause us to clear the obstacle memory the next time we update
  clear_obstacle_memory_ = true;
}

bool ObstacleLayer::isMemoryEnabled()
{
  return use_forgetful_version_;
}

void ObstacleLayer::setMemoryEnabled(const bool enabled)
{
  // "Forgetful version" means that it remembers obstacles for a time, then discards them. Setting it to false means
  // that it doesn't even try to remember them.
  use_forgetful_version_ = enabled;
}

void ObstacleLayer::reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level)
{
  setEnabled(config.enabled);
  setMemoryEnabled(config.enable_forget);

  footprint_clearing_enabled_ = config.footprint_clearing_enabled;
  max_obstacle_height_ = config.max_obstacle_height;
  combination_method_ = config.combination_method;

  obstacle_lifespan_ = config.obstacle_lifespan;
  obstacle_keep_radius_ = config.obstacle_keep_radius;

  pose_confidence_threshold_ = config.pose_confidence_threshold;
}

void ObstacleLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                      const boost::shared_ptr<ObservationBuffer>& buffer,
                                      const bool add_max_range)
{
  // project the laser into a point cloud
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message->header;

  sensor_msgs::LaserScan new_scan = *message;
  if (add_max_range)
  {
    // This forces the addition of points that are at
    //  the sensors max range. This will allow clearning of cells even
    //  if there is nothing in view. NOTE: obstacle range MUST BE < range_max
    //  or you will add obstacles that do no exist to the costmap
    new_scan.range_max = std::numeric_limits<float>::max();
  }

  // project the scan into a point cloud
  try
  {
    projector_.transformLaserScanToPointCloud(message->header.frame_id, new_scan, cloud, *tf_);
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

void ObstacleLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
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

void ObstacleLayer::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
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

void ObstacleLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

void ObstacleLayer::poseConfidenceCallback(const std_msgs::Float64 &message)
{
  pose_confidence_ = message.data;
}

void ObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  if (last_known_enabled_ != enabled_)
  {
    setMaxRange(min_x, min_y, max_x, max_y);
    last_known_enabled_ = enabled_;
  }
  
  // we are making changes to the local costmap so we want to make sure others don't
  boost::unique_lock<mutex_t> lock(*(getMutex()));

  if (use_forgetful_version_)
    return forgetfulUpdateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!enabled_)
    return;
  useExtraBounds(min_x, min_y, max_x, max_y);

  double layer_min_x = robot_x;
  double layer_max_x = robot_x;
  double layer_min_y = robot_y;
  double layer_max_y = robot_y;

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  current = current && getMarkingObservations(observations);

  // get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  // update the global current status
  current_ = current;

  // raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  {
    raytraceFreespace(clearing_observations[i], &layer_min_x, &layer_min_y, &layer_max_x, &layer_max_y);
  }

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const Observation& obs = *it;

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
      costmap_[index] = LETHAL_OBSTACLE;
      touch(px, py, &layer_min_x, &layer_min_y, &layer_max_x, &layer_max_y);
    }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, &layer_min_x, &layer_min_y, &layer_max_x, &layer_max_y);

  // adding margin so that points on the edge are processed.
  {
    double one_cell_world_coords = 1.0 * resolution_;

    layer_min_x -= one_cell_world_coords;
    layer_min_y -= one_cell_world_coords;
    layer_max_x += one_cell_world_coords;
    layer_max_y += one_cell_world_coords;
  }

  // This layer's contribution in cell coordinates
  worldToMapNoBounds(layer_min_x, layer_min_y, min_x_, min_y_);
  worldToMapNoBounds(layer_max_x, layer_max_y, max_x_, max_y_);
  
  // merge the local change with the global change
  *min_x = std::min(*min_x, layer_min_x);
  *min_y = std::min(*min_y, layer_min_y);
  *max_x = std::max(*max_x, layer_max_x);
  *max_y = std::max(*max_y, layer_max_y);
}

static bool sameTimeWorldPoints(const TimeWorldPoint& p1, const TimeWorldPoint& p2, double tolerance)
{
  const double p1x = p1.get<1>();
  const double p1y = p1.get<2>();

  const double p2x = p2.get<1>();
  const double p2y = p2.get<2>();

  const double dx = p2x - p1x;
  const double dy = p2y - p1y;

  return dx*dx + dy*dy <= tolerance * tolerance;
}

void ObstacleLayer::writeTimeWorldPoint(const TimeWorldPoint &p, unsigned char value,
                                        double *min_x, double *min_y, double *max_x, double *max_y)
{
  const double px = p.get<1>();
  const double py = p.get<2>();

  unsigned int mx, my;
  if (!worldToMap(px, py, mx, my))
  {
    ROS_DEBUG("Computing map coords failed");
    return;
  }

  setCost(mx, my, value);
  touch(px, py, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{
    if (!footprint_clearing_enabled_) return;
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
}

void ObstacleLayer::forgetfulUpdateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!enabled_)
    return;
  useExtraBounds(min_x, min_y, max_x, max_y);

  // Check if we received a request to clear the obstacle memory
  if(clear_obstacle_memory_)
  {
    ROS_INFO("Clearing obstacle memory.");
    time_world_points_.clear();
    clear_obstacle_memory_ = false;
  }

  // clear the costmap, it will get rewritten
  resetMaps();
  
  double layer_min_x = robot_x;
  double layer_max_x = robot_x;
  double layer_min_y = robot_y;
  double layer_max_y = robot_y;

  const double time_now = ros::Time::now().toSec();
  const double obstacle_keep_radius2 = obstacle_keep_radius_ * obstacle_keep_radius_;

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  //get the marking observations
  current = current && getMarkingObservations(observations);

  //get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  //update the global current status
  current_ = current;

  // work around for older standards that don't return an iterator on map.erase
  std::vector< std::pair<unsigned int,unsigned int> > map_pending_erase_;
  
  // clear old observations (unless within keep radius)
  const double earliest_epoch = time_now - obstacle_lifespan_;
  obst_map_t::iterator it;
  for (it = time_world_points_.begin(); it != time_world_points_.end(); ++it)
  {
    const double sample_time = (*it).second.get<0>();
    const double px = (*it).second.get<1>();
    const double py = (*it).second.get<2>();

    const double rx = px - robot_x;
    const double ry = py - robot_y;
    const double radius2 = rx*rx + ry*ry;

    if(radius2 > obstacle_keep_radius2 && sample_time < earliest_epoch)
    {
      map_pending_erase_.push_back(it->first);
    }
  }

  for(unsigned int i=0; i<map_pending_erase_.size(); i++)
  {
    time_world_points_.erase (map_pending_erase_[i]);
  }
  map_pending_erase_.clear();

  // write survivors of memory cull
  for (it = time_world_points_.begin(); it != time_world_points_.end(); ++it)
  {
    writeTimeWorldPoint(it->second, LETHAL_OBSTACLE, &layer_min_x, &layer_min_y, &layer_max_x, &layer_max_y);
  }

  // raytrace current freespace - may overwrite old memories, that's OK
  for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  {
    raytraceFreespace(clearing_observations[i], &layer_min_x, &layer_min_y, &layer_max_x, &layer_max_y);
  }

  // If we are footprint clearing then we should do so now to invalidate points.
  if (footprint_clearing_enabled_)
  {
    setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
  }

  // now we will check our memories to make sure they are not FREE_SPACE
  // They would be free space if they got ray-traced away above.
  // We shouldn't remember things that get invalidated by evidence
  for (it = time_world_points_.begin(); it != time_world_points_.end(); ++it)
  {
    TimeWorldPoint& p = it->second;

    const double px = p.get<1>();
    const double py = p.get<2>();

    unsigned int mx, my;
    if (!worldToMap(px, py, mx, my)) 
    {
      map_pending_erase_.push_back(it->first);
    }
    else
    {
      unsigned int index = getIndex(mx, my);
      if (costmap_[index] == FREE_SPACE)
      {
        map_pending_erase_.push_back(it->first);
      }
    }
  }

  for(unsigned int i=0; i<map_pending_erase_.size(); i++)
  {
    time_world_points_.erase (map_pending_erase_[i]);
  }
  map_pending_erase_.clear();

  
  // mark current observations as usual and remember them
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const Observation& obs = *it;

    const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

    for (unsigned int i = 0; i < cloud.points.size(); ++i)
    {
      double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

      //if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_)
      {
        ROS_DEBUG("The point is too high");
        continue;
      }

      //compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
          + (pz - obs.origin_.z) * (pz - obs.origin_.z);

      //if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range)
      {
        ROS_DEBUG("The point is too far away");
        continue;
      }

      TimeWorldPoint p(time_now, px, py);
      writeTimeWorldPoint(p, LETHAL_OBSTACLE, &layer_min_x, &layer_min_y, &layer_max_x, &layer_max_y);

      // if we have low pose confidence we will make sure this
      // data gets cleared quickly by setting it's "birthday" to
      // far in the past.
      if(pose_confidence_ < pose_confidence_threshold_)
      {
        p.get<0>() = time_now - obstacle_lifespan_;
      }

      // remember this data
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my))
      {
        ROS_DEBUG("Computing map coords failed");
      }
      else
      {
        // remove data at location if it exists
        std::pair<unsigned int, unsigned int> location(mx,my);
        time_world_points_.erase(location);

        // insert new data
        time_world_points_[location] = p;
      }
    }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, &layer_min_x, &layer_min_y, &layer_max_x, &layer_max_y);

  // adding margin so that points on the edge are processed.
  {
    double one_cell_world_coords = 1.0 * resolution_;

    layer_min_x -= one_cell_world_coords;
    layer_min_y -= one_cell_world_coords;
    layer_max_x += one_cell_world_coords;
    layer_max_y += one_cell_world_coords;
  }

  // This layer's contribution in cell coordinates
  worldToMapNoBounds(layer_min_x, layer_min_y, min_x_, min_y_);
  worldToMapNoBounds(layer_max_x, layer_max_y, max_x_, max_y_);

  // merge the local change with the global change
  *min_x = std::min(*min_x, layer_min_x);
  *min_y = std::min(*min_y, layer_min_y);
  *max_x = std::max(*max_x, layer_max_x);
  *max_y = std::max(*max_y, layer_max_y);
}


void ObstacleLayer::updateCosts(LayerActions* layer_actions, costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
  {
    current_ = true; // don't block a waiting process
    return;
  }

  if (footprint_clearing_enabled_)
  {
    setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
  }

  if (combination_method_ == 0)
  {
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
    if (layer_actions)
    {
      layer_actions->addAction(
            AxisAlignedBoundingBox(min_x_, min_y_, max_x_, max_y_),
            this,
            AxisAlignedBoundingBox(min_x_, min_y_, max_x_, max_y_),
            &master_grid,
            LayerActions::OVERWRITE);
    }
  }
  
  if (combination_method_ == 1)
  {
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    if (layer_actions)
    {
      layer_actions->addAction(
            AxisAlignedBoundingBox(min_x_, min_y_, max_x_, max_y_),
            this,
            AxisAlignedBoundingBox(min_x_, min_y_, max_x_, max_y_),
            &master_grid,
            LayerActions::MAX);
    }
  }

  current_ = true; // allow consumers to use this data
}

void ObstacleLayer::updateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  updateCosts(NULL, master_grid, min_i, min_j, max_i, max_j);
}

void ObstacleLayer::addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.push_back(obs);
  if (clearing)
    static_clearing_observations_.push_back(obs);
}

void ObstacleLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.clear();
  if (clearing)
    static_clearing_observations_.clear();
}

bool ObstacleLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const
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

bool ObstacleLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const
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

void ObstacleLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
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


  touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  for (unsigned int i = 0; i < cloud.points.size(); ++i)
  {
    double wx = cloud.points[i].x;
    double wy = cloud.points[i].y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    if (wx < origin_x)
    {
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y)
    {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x)
    {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y)
    {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // check for legality just in case
    if (!worldToMap(wx, wy, x1, y1))
      continue;

    unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
    MarkCell marker(costmap_, FREE_SPACE);
    // and finally... we can execute our trace to clear obstacles along that line
    raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

    updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
  }
}

void ObstacleLayer::activate()
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
void ObstacleLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->unsubscribe();
  }
}

void ObstacleLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                         double* min_x, double* min_y, double* max_x, double* max_y)
{
  double dx = wx-ox, dy = wy-oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::reset()
{
    deactivate();
    resetMaps();
    current_ = true;
    activate();
}

}  // namespace costmap_2d
