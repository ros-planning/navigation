#include<costmap_2d/static_layer.h>
#include<costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::StaticLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

StaticLayer::StaticLayer() : dsrv_(NULL) {}

void StaticLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);
  
  nh.param("track_unknown_space", track_unknown_space_, true);
  nh.param("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;
  //we'll subscribe to the latched topic that the map server uses
  ROS_INFO("Requesting the map...");
  map_sub_ = g_nh.subscribe(map_topic, 1, &StaticLayer::incomingMap, this);
  map_received_ = false;
  has_updated_data_ = false;

  ros::Rate r(10);
  while (!map_received_ && g_nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());
  
  if(subscribe_to_updates_)
  {
    ROS_INFO("Subscribing to updates");
    map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &StaticLayer::incomingUpdate, this);
  }

  if(dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &StaticLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void StaticLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
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

void StaticLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

unsigned char StaticLayer::interpretValue(unsigned char value)
{
  //check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void StaticLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  Costmap2D* master = layered_costmap_->getCostmap();
  if (master->getSizeInCellsX() != size_x ||
      master->getSizeInCellsY() != size_y ||
      master->getResolution() != new_map->info.resolution ||
      master->getOriginX() != new_map->info.origin.position.x ||
      master->getOriginY() != new_map->info.origin.position.y ||
      !layered_costmap_->isSizeLocked())
  {
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y, true);
  }else if(size_x_ != size_x || size_y_ != size_y ||
      resolution_ != new_map->info.resolution ||
      origin_x_ != new_map->info.origin.position.x ||
      origin_y_ != new_map->info.origin.position.y){
    matchSize();
  }

  unsigned int index = 0;

  //initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;
}

void StaticLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
    unsigned int di = 0;
    for (unsigned int y = 0; y < update->height ; y++)
    {
        unsigned int index_base = (update->y + y) * update->width;
        for (unsigned int x = 0; x < update->width ; x++)
        {
            unsigned int index = index_base + x + update->x;
            costmap_[index] = interpretValue( update->data[di++] );
        }
    }
    x_ = update->x;
    y_ = update->y;
    width_ = update->width;
    height_ = update->height;
    has_updated_data_ = true;
}

void StaticLayer::activate()
{
    onInitialize();
}

void StaticLayer::deactivate()
{
    map_sub_.shutdown();
    if (subscribe_to_updates_)
        map_update_sub_.shutdown();
}

void StaticLayer::reset()
{
    deactivate();
    activate();
}

void StaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                        double* max_x, double* max_y)
{
  if (!map_received_ || !has_updated_data_)
    return;

  double mx, my;
  
  mapToWorld(x_, y_, mx, my);
  *min_x = std::min(mx, *min_x);
  *min_y = std::min(my, *min_y);
  
  mapToWorld(x_ + width_, y_ + height_, mx, my);
  *max_x = std::max(mx, *max_x);
  *max_y = std::max(my, *max_y);
  
  has_updated_data_ = false;

}

void StaticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_received_)
    return;
  if(!use_maximum_)
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
  else
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}
