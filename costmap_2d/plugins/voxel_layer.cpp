#include <costmap_2d/voxel_layer.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>

#define VOXEL_BITS 16
PLUGINLIB_EXPORT_CLASS(costmap_2d::VoxelLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;
using namespace std;

namespace costmap_2d
{

void VoxelLayer::onInitialize()
{
  ObstacleLayer::onInitialize();
  ros::NodeHandle private_nh("~/" + name_);

  private_nh.param("publish_voxel_map", publish_voxel_, false);

  private_nh.param("inflation_radius", inflation_radius_, 0.4);
  ROS_WARN("Inflation radius : %f", inflation_radius_);
  if (publish_voxel_)
    voxel_pub_ = private_nh.advertise < costmap_2d::VoxelGrid > ("voxel_grid", 1);

  clearing_endpoints_pub_ = private_nh.advertise<sensor_msgs::PointCloud>( "clearing_endpoints", 1 );
}

void VoxelLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::VoxelPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::VoxelPluginConfig>::CallbackType cb = boost::bind(
      &VoxelLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

VoxelLayer::~VoxelLayer()
{
  if(dsrv_)
    delete dsrv_;
}

void VoxelLayer::reconfigureCB(costmap_2d::VoxelPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  max_obstacle_height_ = config.max_obstacle_height;
  size_z_ = config.z_voxels;
  origin_z_ = config.origin_z;
  z_resolution_ = config.z_resolution;
  unknown_threshold_ = config.unknown_threshold + (VOXEL_BITS - size_z_);
  mark_threshold_ = config.mark_threshold;
  combination_method_ = config.combination_method;

  inflation_radius_ = config.inflation_radius;
  ROS_WARN("Inflation radius : %f", inflation_radius_);

  matchSize();
}

void VoxelLayer::matchSize()
{
  ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);
  ROS_ASSERT(voxel_grid_.sizeX() == size_x_ && voxel_grid_.sizeY() == size_y_);
  if(clear_old_){
    locations_utime.resize(voxel_grid_.sizeX() * voxel_grid_.sizeY());
  }
}

void VoxelLayer::reset()
{
  deactivate();
  resetMaps();
  voxel_grid_.reset();
  if(clear_old_){
    locations_utime.reset();
  }
  activate();
}

void VoxelLayer::resetOldCosts(double* min_x, double* min_y, 
			       double* max_x, double* max_y){
  //removes any obstacles that were put down based on sensor observation when the timer expires 
  double current_time = ros::Time::now().toSec();

  int total_size = new_obs_list.size();
  int checked_count = 0; 

  int error_count = 0; 
  for(int i=0; i < new_obs_list.size(); i++){
    CostMapList &list = new_obs_list[i];
    std::map<std::string, double>::iterator it; 
    it = observation_timeout.find(list.topic); 

    double obs_persistance = max_obstacle_persistance_;
    if(it != observation_timeout.end()){
      obs_persistance = it->second; 
      ROS_DEBUG("Topic : %s - Timeout : %f\n", list.topic.c_str(), obs_persistance);
    }
    else{
      ROS_ERROR("Asked to clear cost for non-timeout topic\n");
    }

    if((current_time - list.obs_timestamp / 1.0e6) > obs_persistance){
      ROS_DEBUG("Clearing timeout observation : %f\n", (current_time - list.obs_timestamp / 1.0e6));
      int cleared_count = 0;
      checked_count++;
      double *topic_utime =  locations_utime.get_values(list.topic);
      for(int j=0; j < list.indices.size(); j++){
	if(topic_utime[list.indices[j].index] == list.obs_timestamp / 1.0e6){
          //right now this clears everything (irrespective of the height) - this is prob bad if the sensors report different heights 
	  costmap_[list.indices[j].index] = FREE_SPACE; 
	  topic_utime[list.indices[j].index] = -1;
	  //increase the map update bounds 
	  touch(list.indices[j].x, list.indices[j].y, min_x, min_y, max_x, max_y);
	}
      }
    }
    else{
      //remove the costmap_list upto (and not including this)
      if(i > 0){
	new_obs_list.erase(new_obs_list.begin(), new_obs_list.begin() + (i-1));
      }
      break;
    }
  }
}

  //we need to get observations for each sensor - to figure out which ones have timeouts 
  bool VoxelLayer::getMarkingObservations(std::vector<ObservationSet>& observations_set) const
{
  bool current = true;
  //get the marking observations
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
  {
    marking_buffers_[i]->lock();
    ObservationSet obs(marking_buffers_[i]->getTopic());
    marking_buffers_[i]->getObservations(obs.marking_observations);//marking_observations);
    current = marking_buffers_[i]->isCurrent() && current;
    observations_set.push_back(obs);
    marking_buffers_[i]->unlock();
  }

  //what the heck is the static marking observations?? 
  ObservationSet obs("static");
  obs.marking_observations.insert(obs.marking_observations.end(), static_marking_observations_.begin(), static_marking_observations_.end());
  observations_set.push_back(obs);
  return current;
}

bool VoxelLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const
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

void VoxelLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                       double* min_y, double* max_x, double* max_y)
{
  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!enabled_)
    return;
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<ObservationSet> observations;
  std::vector<Observation> clearing_observations;

  if(clear_old_){
    resetOldCosts(min_x, min_y, max_x, max_y);
  }

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
  for (std::vector<ObservationSet>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const ObservationSet& obs_set = *it; 

    //check if this is in the max_timeout list 
    bool max_timeout = false; 
    
    if(observation_timeout.find(obs_set.topic)!= observation_timeout.end()){
      max_timeout = true;
      ROS_DEBUG("Observation set for topic %s - clearing\n", obs_set.topic.c_str());
    }

    for(std::vector<Observation>::const_iterator it_o = obs_set.marking_observations.begin(); 
        it_o != obs_set.marking_observations.end(); it_o++){
      const Observation& obs = *it_o;

      //we should throw out stale observations also 

      CostMapList cm_list; 
      cm_list.topic = obs_set.topic;
      cm_list.obs_timestamp = obs.cloud_->header.stamp; 
      
      double obs_ts = cm_list.obs_timestamp / 1.0e6; 

      const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

      double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

      int count = 0; 

      double *topic_utime =  NULL;
      if(clear_old_){
        topic_utime = locations_utime.get_values(obs_set.topic);
      }
    
      for (unsigned int i = 0; i < cloud.points.size(); ++i)
        {
          //if the obstacle is too high or too far away from the robot we won't add it
          if (cloud.points[i].z > max_obstacle_height_)
            continue;
          /*if(i== 0){
            fprintf(stdout, "Topic : %s - Cloud Height : %f\n", obs_set.topic.c_str(), cloud.points[i].z);
            }*/

          //compute the squared distance from the hitpoint to the pointcloud's origin
          double sq_dist = (cloud.points[i].x - obs.origin_.x) * (cloud.points[i].x - obs.origin_.x)
            + (cloud.points[i].y - obs.origin_.y) * (cloud.points[i].y - obs.origin_.y)
            + (cloud.points[i].z - obs.origin_.z) * (cloud.points[i].z - obs.origin_.z);

          //if the point is far enough away... we won't consider it
          if (sq_dist >= sq_obstacle_range)
            continue;

          //now we need to compute the map coordinates for the observation
          unsigned int mx, my, mz;
          if (cloud.points[i].z < origin_z_)
            {
              if (!worldToMap3D(cloud.points[i].x, cloud.points[i].y, origin_z_, mx, my, mz))
                continue;
            }
          else if (!worldToMap3D(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, mx, my, mz))
            {
              continue;
            }

          //mark the cell in the voxel grid and check if we should also mark it in the costmap
          if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_))
            {
              unsigned int index = getIndex(mx, my);
              //these are the ones set as occupied 
              costmap_[index] = LETHAL_OBSTACLE;
              touch((double)cloud.points[i].x, (double)cloud.points[i].y, min_x, min_y, max_x, max_y);

              //keep track of which indexs we updated 
              if(max_timeout){
                cm_list.indices.push_back(ObstaclePoint(index, (double)cloud.points[i].x, (double)cloud.points[i].y));
                topic_utime[index] = obs_ts; 
                count++;
              }
            }
        }
      if(max_timeout && count > 0){
        new_obs_list.push_back(cm_list);
      }
    }
  }

  if(clear_old_){
    //inflate the bounds - otherwise the inflation layer wont reset the inflated cost 
    *min_x -= inflation_radius_;
    *min_y -= inflation_radius_;
    *max_x += inflation_radius_;
    *max_y += inflation_radius_;
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

  footprint_layer_.updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void VoxelLayer::clearNonLethal(double wx, double wy, double w_size_x, double w_size_y, bool clear_no_info)
{
  //get the cell coordinates of the center point of the window
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my))
    return;

  //compute the bounds of the window
  double start_x = wx - w_size_x / 2;
  double start_y = wy - w_size_y / 2;
  double end_x = start_x + w_size_x;
  double end_y = start_y + w_size_y;

  //scale the window based on the bounds of the costmap
  start_x = std::max(origin_x_, start_x);
  start_y = std::max(origin_y_, start_y);

  end_x = std::min(origin_x_ + getSizeInMetersX(), end_x);
  end_y = std::min(origin_y_ + getSizeInMetersY(), end_y);

  //get the map coordinates of the bounds of the window
  unsigned int map_sx, map_sy, map_ex, map_ey;

  //check for legality just in case
  if (!worldToMap(start_x, start_y, map_sx, map_sy) || !worldToMap(end_x, end_y, map_ex, map_ey))
    return;

  //we know that we want to clear all non-lethal obstacles in this window to get it ready for inflation
  unsigned int index = getIndex(map_sx, map_sy);
  unsigned char* current = &costmap_[index];
  for (unsigned int j = map_sy; j <= map_ey; ++j)
  {
    for (unsigned int i = map_sx; i <= map_ex; ++i)
    {
      //if the cell is a lethal obstacle... we'll keep it and queue it, otherwise... we'll clear it
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

void VoxelLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
                                           double* max_x, double* max_y)
{
  if (clearing_observation.cloud_->points.size() == 0)
    return;

  double sensor_x, sensor_y, sensor_z;
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  double oz = clearing_observation.origin_.z;

  if (!worldToMap3DFloat(ox, oy, oz, sensor_x, sensor_y, sensor_z))
  {
    ROS_WARN_THROTTLE(
        1.0,
        "The origin for the sensor at (%.2f, %.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
        ox, oy, oz);
    return;
  }

  bool publish_clearing_points = (clearing_endpoints_pub_.getNumSubscribers() > 0);
  if( publish_clearing_points )
  {
    clearing_endpoints_.points.clear();
    clearing_endpoints_.points.reserve( clearing_observation.cloud_->points.size() );
  }

  //we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double map_end_x = origin_x_ + getSizeInMetersX();
  double map_end_y = origin_y_ + getSizeInMetersY();

  for (unsigned int i = 0; i < clearing_observation.cloud_->points.size(); ++i)
  {
    double wpx = clearing_observation.cloud_->points[i].x;
    double wpy = clearing_observation.cloud_->points[i].y;
    double wpz = clearing_observation.cloud_->points[i].z;

    double distance = dist(ox, oy, oz, wpx, wpy, wpz);
    double scaling_fact = 1.0;
    scaling_fact = std::max(std::min(scaling_fact, (distance - 2 * resolution_) / distance), 0.0);
    wpx = scaling_fact * (wpx - ox) + ox;
    wpy = scaling_fact * (wpy - oy) + oy;
    wpz = scaling_fact * (wpz - oz) + oz;

    double a = wpx - ox;
    double b = wpy - oy;
    double c = wpz - oz;
    double t = 1.0;

    //we can only raytrace to a maximum z height
    if (wpz > max_obstacle_height_)
    {
      //we know we want the vector's z value to be max_z
      t = std::max(0.0, std::min(t, (max_obstacle_height_ - 0.01 - oz) / c));
    }
    //and we can only raytrace down to the floor
    else if (wpz < origin_z_)
    {
      //we know we want the vector's z value to be 0.0
      t = std::min(t, (origin_z_ - oz) / c);
    }

    //the minimum value to raytrace from is the origin
    if (wpx < origin_x_)
    {
      t = std::min(t, (origin_x_ - ox) / a);
    }
    if (wpy < origin_y_)
    {
      t = std::min(t, (origin_y_ - oy) / b);
    }

    //the maximum value to raytrace to is the end of the map
    if (wpx > map_end_x)
    {
      t = std::min(t, (map_end_x - ox) / a);
    }
    if (wpy > map_end_y)
    {
      t = std::min(t, (map_end_y - oy) / b);
    }

    wpx = ox + a * t;
    wpy = oy + b * t;
    wpz = oz + c * t;

    double point_x, point_y, point_z;
    if (worldToMap3DFloat(wpx, wpy, wpz, point_x, point_y, point_z))
    {
      unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);

      //voxel_grid_.markVoxelLine(sensor_x, sensor_y, sensor_z, point_x, point_y, point_z);
      voxel_grid_.clearVoxelLineInMap(sensor_x, sensor_y, sensor_z, point_x, point_y, point_z, costmap_,
                                      unknown_threshold_, mark_threshold_, FREE_SPACE, NO_INFORMATION,
                                      cell_raytrace_range);

      updateRaytraceBounds(ox, oy, wpx, wpy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);

      if( publish_clearing_points )
      {
        geometry_msgs::Point32 point;
        point.x = wpx;
        point.y = wpy;
        point.z = wpz;
        clearing_endpoints_.points.push_back( point );
      }
    }
  }

  if( publish_clearing_points )
  {
    clearing_endpoints_.header.frame_id = global_frame_;
    clearing_endpoints_.header.stamp = pcl_conversions::fromPCL(clearing_observation.cloud_->header).stamp;
    clearing_endpoints_.header.seq = clearing_observation.cloud_->header.seq;

    clearing_endpoints_pub_.publish( clearing_endpoints_ );
  }
}

void VoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  //project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = int((new_origin_x - origin_x_) / resolution_);
  cell_oy = int((new_origin_y - origin_y_) / resolution_);

  if(cell_ox == 0 && cell_oy == 0){
    return;
  } 

  //compute the associated world coordinates for the origin cell
  //beacuase we want to keep things grid-aligned
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  //To save casting from unsigned int to int a bunch of times
  int size_x = size_x_;
  int size_y = size_y_;

  //we need to compute the overlap of the new and existing windows
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  //we need a map to store the obstacles in the window temporarily
  unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];
  unsigned int* local_voxel_map = new unsigned int[cell_size_x * cell_size_y];
  unsigned int* voxel_map = voxel_grid_.getData();

  //copy the local window in the costmap to the local map
  copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);
  copyMapRegion(voxel_map, lower_left_x, lower_left_y, size_x_, local_voxel_map, 0, 0, cell_size_x, cell_size_x,
                cell_size_y);

  //we'll reset our maps to unknown space if appropriate
  resetMaps();

  //update the origin with the appropriate world coordinates
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  //compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  //now we want to copy the overlapping information back into the map, but in its new location
  copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);
  copyMapRegion(local_voxel_map, 0, 0, cell_size_x, voxel_map, start_x, start_y, size_x_, cell_size_x, cell_size_y);

  //make sure to clean up
  delete[] local_map;
  delete[] local_voxel_map;

}

}
