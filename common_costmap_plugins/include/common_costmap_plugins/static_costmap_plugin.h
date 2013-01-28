#ifndef STATIC_COSTMAP_PLUGIN_H_
#define STATIC_COSTMAP_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/plugin_base.h>
#include <costmap_2d/layered_costmap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

namespace common_costmap_plugins
{
  class StaticCostmapPlugin : public costmap_2d::CostmapPlugin, public costmap_2d::Costmap2D
  {
    public:
      StaticCostmapPlugin() { layered_costmap_ = NULL; }

      void initialize(costmap_2d::LayeredCostmap* costmap, std::string name);
      void update_bounds(double* min_x, double* min_y, double* max_x, double* max_y);
      void update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

      void activate() {}
      void deactivate() {}    
      bool isDiscretized() { return true; }
      void matchSize(); 

    private:
      /**
       * @brief  Callback to update the costmap's map from the map_server
       * @param new_map The map to put into the costmap. The origin of the new
       * map along with its size will determine what parts of the costmap's
       * static map are overwritten.
       */
      void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);

      std::string global_frame_; ///< @brief The global frame for the costmap
      bool map_recieved_, map_initialized_;
      bool track_unknown_space_;
      ros::Subscriber map_sub_;

      unsigned char lethal_threshold_, unknown_cost_value_;

      mutable boost::recursive_mutex lock_;
  };
};
#endif

