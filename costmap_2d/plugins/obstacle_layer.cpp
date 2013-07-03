#include<costmap_2d/obstacle_layer.h>
#include<costmap_2d/costmap_math.h>

#include <pluginlib/class_list_macros.h>
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
  default_value_ = NO_INFORMATION;

  initMaps();
  current_ = true;
  has_been_reset_ = false;

  global_frame_ = layered_costmap_->getGlobalFrameID();
  double transform_tolerance;
  nh.param("transform_tolerance", transform_tolerance, 0.2);

  std::string topics_string;
  //get the topics that we'll subscribe to from the parameter server
  nh.param("observation_sources", topics_string, std::string(""));
  ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

  //now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source)
  {
    ros::NodeHandle source_node(nh, source);

    //get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool clearing, marking;

    source_node.param("topic", topic, source);
    source_node.param("sensor_frame", sensor_frame, std::string(""));
    source_node.param("observation_persistence", observation_keep_time, 0.0);
    source_node.param("expected_update_rate", expected_update_rate, 0.0);
    source_node.param("data_type", data_type, std::string("PointCloud"));
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
    source_node.param("clearing", clearing, false);
    source_node.param("marking", marking, true);

    if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
    {
      ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
      throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
    }

    std::string raytrace_range_param_name, obstacle_range_param_name;

    //get the obstacle range for the sensor
    double obstacle_range = 2.5;
    if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
    {
      source_node.getParam(obstacle_range_param_name, obstacle_range);
    }

    //get the raytrace range for the sensor
    double raytrace_range = 3.0;
    if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
    {
      source_node.getParam(raytrace_range_param_name, raytrace_range);
    }

    ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
              sensor_frame.c_str());

    //create an observation buffer
    observation_buffers_.push_back(
        boost::shared_ptr < ObservationBuffer
            > (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                     max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
                                     sensor_frame, transform_tolerance)));

    //check if we'll add this buffer to our marking observation buffers
    if (marking)
      marking_buffers_.push_back(observation_buffers_.back());

    //check if we'll also add this buffer to our clearing observation buffers
    if (clearing)
      clearing_buffers_.push_back(observation_buffers_.back());

    ROS_DEBUG(
        "Created an observation buffer for source %s, topic %s, global frame: %s, expected update rate: %.2f, observation persistence: %.2f",
        source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

    //create a callback for the topic
    if (data_type == "LaserScan")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
          > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

      boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan>
          > filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50));
      filter->registerCallback(
          boost::bind(&ObstacleLayer::laserScanCallback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);

      observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
    }
    else if (data_type == "PointCloud")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));

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

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>::CallbackType cb = boost::bind(
      &ObstacleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  footprint_layer_.initialize( layered_costmap_, name_ + "_footprint", tf_);
}

void ObstacleLayer::reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  max_obstacle_height_ = config.max_obstacle_height;
}

void ObstacleLayer::initMaps()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void ObstacleLayer::matchSize()
{
  initMaps();
}

void ObstacleLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                              const boost::shared_ptr<ObservationBuffer>& buffer)
{
  //project the laser into a point cloud
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message->header;

  //project the scan into a point cloud
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

  //buffer the point cloud
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

  //buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud2);
  buffer->unlock();
}

void ObstacleLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                const boost::shared_ptr<ObservationBuffer>& buffer)
{
  //buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

void ObstacleLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  if (rolling_window_)
    updateOrigin(origin_x - getSizeInMetersX() / 2, origin_y - getSizeInMetersY() / 2);
  if (!enabled_)
    return;
  if (has_been_reset_)
  {
    *min_x = std::min(reset_min_x_, *min_x);
    *min_y = std::min(reset_min_y_, *min_y);
    *max_x = std::max(reset_max_x_, *max_x);
    *max_y = std::max(reset_max_y_, *max_y);
    reset_min_x_ = 1e6;
    reset_min_y_ = 1e6;
    reset_max_x_ = -1e6;
    reset_max_y_ = -1e6;
    has_been_reset_ = false;
  }

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  //get the marking observations
  current = current && getMarkingObservations(observations);

  //get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  //update the global current status
  current_ = current;

  //raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  //place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const Observation& obs = *it;

    const pcl::PointCloud<pcl::PointXYZ>& cloud = obs.cloud_;

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

      //now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my))
      {
        ROS_DEBUG("Computing map coords failed");
        continue;
      }

      unsigned int index = getIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;
      *min_x = std::min(px, *min_x);
      *min_y = std::min(py, *min_y);
      *max_x = std::max(px, *max_x);
      *max_y = std::max(py, *max_y);
    }
  }

  footprint_layer_.updateBounds(origin_x, origin_y, origin_yaw, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  // The footprint layer clears the footprint in this ObstacleLayer
  // before we merge this obstacle layer into the master_grid.
  footprint_layer_.updateCosts(*this, min_i, min_j, max_i, max_j);

  const unsigned char* master_array = master_grid.getCharMap();
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      unsigned char old_cost = master_array[index];
      if (old_cost == NO_INFORMATION || old_cost < costmap_[index])
        master_grid.setCost(i, j, costmap_[index]);
    }
  }
}

void ObstacleLayer::addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing)
{
  if(marking)
    static_marking_observations_.push_back(obs);
  if(clearing)
    static_clearing_observations_.push_back(obs);
}

bool ObstacleLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const
{
  bool current = true;
  //get the marking observations
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
  //get the clearing observations
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
  pcl::PointCloud < pcl::PointXYZ > cloud = clearing_observation.cloud_;

  //get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  if (!worldToMap(ox, oy, x0, y0))
  {
    ROS_WARN_THROTTLE(
        1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
        ox, oy);
    return;
  }

  //we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;


  *min_x = std::min(ox, *min_x);
  *min_y = std::min(oy, *min_y);
  *max_x = std::max(ox, *max_x);
  *max_y = std::max(oy, *max_y);

  //for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  for (unsigned int i = 0; i < cloud.points.size(); ++i)
  {
    double wx = cloud.points[i].x;
    double wy = cloud.points[i].y;

    //now we also need to make sure that the enpoint we're raytracing
    //to isn't off the costmap and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    //the minimum value to raytrace from is the origin
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

    //the maximum value to raytrace to is the end of the map
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

    //now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    //check for legality just in case
    if (!worldToMap(wx, wy, x1, y1))
      continue;

    *min_x = std::min(wx, *min_x);
    *min_y = std::min(wy, *min_y);
    *max_x = std::max(wx, *max_x);
    *max_y = std::max(wy, *max_y);

    unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
    MarkCell marker(costmap_, FREE_SPACE);
    //and finally... we can execute our trace to clear obstacles along that line
    raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);
  }
}

void ObstacleLayer::activate()
{
  //if we're stopped we need to re-subscribe to topics
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

void ObstacleLayer::onFootprintChanged()
{
  footprint_layer_.onFootprintChanged();
}

} // end namespace costmap_2d
