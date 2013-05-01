#include<costmap_2d/static_costmap_plugin.h>
#include<costmap_2d/costmap_math.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(common_costmap_plugins::StaticCostmapPlugin, costmap_2d::CostmapPluginROS)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace common_costmap_plugins
{
void StaticCostmapPlugin::initialize(costmap_2d::LayeredCostmap* costmap, std::string name)
{
  ros::NodeHandle nh("~/" + name), g_nh;
  layered_costmap_ = costmap;
  name_ = name;
  current_ = true;

  global_frame_ = costmap->getGlobalFrameID();

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("track_unknown_space", track_unknown_space_, true);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;
  //we'll subscribe to the latched topic that the map server uses
  ROS_INFO("Requesting the map...");
  map_sub_ = g_nh.subscribe(map_topic, 1, &StaticCostmapPlugin::incomingMap, this);
  map_recieved_ = false;

  ros::Rate r(10);
  while (!map_recieved_ && g_nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  map_initialized_ = false;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &StaticCostmapPlugin::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void StaticCostmapPlugin::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    map_initialized_ = false;
  }
}

void StaticCostmapPlugin::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getGlobalFrameID(), master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void StaticCostmapPlugin::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_INFO("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                              new_map->info.origin.position.y, true);
  unsigned int index = 0;

  //initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];
      //check if the static value is above the unknown or lethal thresholds
      if (track_unknown_space_ && value == unknown_cost_value_)
        costmap_[index] = NO_INFORMATION;
      else if (value >= lethal_threshold_)
        costmap_[index] = LETHAL_OBSTACLE;
      else
        costmap_[index] = FREE_SPACE;

      ++index;
    }
  }
  map_recieved_ = true;
}

void StaticCostmapPlugin::update_bounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y,
                                        double* max_x, double* max_y)
{
  if (!map_recieved_ || map_initialized_)
    return;

  mapToWorld(0, 0, *min_x, *min_y);
  mapToWorld(size_x_, size_y_, *max_x, *max_y);
  map_initialized_ = true;

}

void StaticCostmapPlugin::update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_initialized_)
    return;
  if (!enabled_)
    return;
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      master_grid.setCost(i, j, costmap_[index]);
    }
  }
}

}
