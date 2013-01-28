#ifndef FOOTPRINT_COSTMAP_PLUGIN_H_
#define FOOTPRINT_COSTMAP_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/plugin_base.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

namespace common_costmap_plugins
{
  class FootprintCostmapPlugin : public costmap_2d::CostmapPlugin
  {
    public:
      FootprintCostmapPlugin() { layered_costmap_ = NULL; }

      void initialize(costmap_2d::LayeredCostmap* costmap, std::string name);
      void update_bounds(double* min_x, double* min_y, double* max_x, double* max_y);
      void update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

      void activate() {}
      void deactivate() {}

    private:
      void publishFootprint();

      geometry_msgs::Polygon footprint_spec_;
      geometry_msgs::PolygonStamped footprint_;
      bool circular_;
      ros::Publisher footprint_pub_;
      double inscribed_radius_, circumscribed_radius_;
      double min_x_, min_y_, max_x_, max_y_;

  };
};
#endif

