#include<costmap_2d/footprint_costmap_plugin.h>
#include<costmap_2d/footprint.h>
#include<string>
#include<sstream>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(common_costmap_plugins::FootprintCostmapPlugin, costmap_2d::CostmapPluginROS)

namespace common_costmap_plugins
{
  void FootprintCostmapPlugin::initialize(costmap_2d::LayeredCostmap* costmap, std::string name)
  {
    ros::NodeHandle nh("~/" + name);
    ros::NodeHandle g_nh;
    layered_costmap_ = costmap;
    name_ = name;
    footprint_.header.frame_id = layered_costmap_->getGlobalFrameID();
    current_ = false;

    footprint_pub_ = nh.advertise<geometry_msgs::PolygonStamped>( "footprint_stamped", 1 );

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&FootprintCostmapPlugin::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    current_ = true;
  }

  void FootprintCostmapPlugin::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
  }

  void FootprintCostmapPlugin::update_bounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
  {
    if(!enabled_) return;
    //update transformed polygon
    footprint_.header.stamp = ros::Time::now();
    footprint_.polygon.points.clear();
    double cos_th = cos(origin_yaw);
    double sin_th = sin(origin_yaw);
    const std::vector<geometry_msgs::Point>& footprint_spec = getFootprint();
    for(unsigned int i = 0; i < footprint_spec.size(); ++i)
    {
      geometry_msgs::Point32 new_pt;
      new_pt.x = origin_x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
      new_pt.y = origin_y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
      footprint_.polygon.points.push_back(new_pt);
    }

    for(unsigned int i=0; i < footprint_.polygon.points.size(); i++)
    {
      double px = footprint_.polygon.points[i].x, py = footprint_.polygon.points[i].y;
      *min_x = std::min(px, *min_x);
      *min_y = std::min(py, *min_y);
      *max_x = std::max(px, *max_x);
      *max_y = std::max(py, *max_y);
    }
    footprint_pub_.publish( footprint_ );
  }

  void FootprintCostmapPlugin::update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    if(!enabled_) return;
    master_grid.setConvexPolygonCost(costmap_2d::toPointVector(footprint_.polygon), costmap_2d::FREE_SPACE);
  }
}

