#ifndef COSTMAP_PLUGIN_BASE_H_
#define COSTMAP_PLUGIN_BASE_H_
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <string>

namespace costmap_2d {
  class LayeredCostmap;
  class CostmapPlugin {
    public:
      virtual void initialize(LayeredCostmap* costmap, std::string name) = 0;
      virtual void update_bounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y) = 0;
      virtual void update_costs(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) = 0;

      virtual void deactivate() = 0;   // stop publishers
      virtual void activate() = 0;     // restart publishers if they've been stopped
      virtual ~CostmapPlugin() {}

      bool isCurrent() { return current_; }
      virtual void matchSize() {}  // matches the size of the parent costmap

    protected:
      CostmapPlugin() {}
      LayeredCostmap* layered_costmap_;
      bool current_;
  };
};  // namespace layered_costmap
#endif
