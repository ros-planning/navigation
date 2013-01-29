#ifndef INFLATION_COSTMAP_PLUGIN_H_
#define INFLATION_COSTMAP_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/plugin_base.h>
#include <costmap_2d/layered_costmap.h>
#include <queue>

namespace common_costmap_plugins
{
  /**
   * @class CellData
   * @brief Storage for cell information used during obstacle inflation
   */
  class CellData {
    public:
      /**
       * @brief  Constructor for a CellData objects
       * @param  d The distance to the nearest obstacle, used for ordering in the priority queue
       * @param  i The index of the cell in the cost map
       * @param  x The x coordinate of the cell in the cost map
       * @param  y The y coordinate of the cell in the cost map
       * @param  sx The x coordinate of the closest obstacle cell in the costmap
       * @param  sy The y coordinate of the closest obstacle cell in the costmap
       * @return 
       */
      CellData(double d, double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) : distance_(d), 
      index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy) {}
      double distance_;
      unsigned int index_;
      unsigned int x_, y_;
      unsigned int src_x_, src_y_;
  };

  /**
   * @brief Provide an ordering between CellData objects in the priority queue 
   * @return We want the lowest distance to have the highest priority... so this returns true if a has higher priority than b
   */
  inline bool operator<(const CellData &a, const CellData &b){
    return a.distance_ > b.distance_;
  }

  class InflationCostmapPlugin : public costmap_2d::CostmapPlugin
  {
    public:
      InflationCostmapPlugin() { layered_costmap_ = NULL; }
      ~InflationCostmapPlugin() { deleteKernels(); }

      void initialize(costmap_2d::LayeredCostmap* costmap, std::string name);
      void update_bounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
      void update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
      virtual bool isDiscretized() { return true; } 
      virtual void matchSize();
      void activate() {}
      void deactivate() {}

    private:
      /**
       * @brief  Lookup pre-computed distances
       * @param mx The x coordinate of the current cell 
       * @param my The y coordinate of the current cell 
       * @param src_x The x coordinate of the source cell 
       * @param src_y The y coordinate of the source cell 
       * @return 
       */
      inline double distanceLookup(int mx, int my, int src_x, int src_y){
        unsigned int dx = abs(mx - src_x);
        unsigned int dy = abs(my - src_y);
        return cached_distances_[dx][dy];
      }

      /**
       * @brief  Lookup pre-computed costs
       * @param mx The x coordinate of the current cell 
       * @param my The y coordinate of the current cell 
       * @param src_x The x coordinate of the source cell 
       * @param src_y The y coordinate of the source cell 
       * @return 
       */
      inline unsigned char costLookup(int mx, int my, int src_x, int src_y){
        unsigned int dx = abs(mx - src_x);
        unsigned int dy = abs(my - src_y);
        return cached_costs_[dx][dy];
      }
      
      

  void computeCaches();
  void deleteKernels();
        inline unsigned char computeCost(double distance) const;
        void inflate_area(int min_i, int min_j, int max_i, int max_j, unsigned char* master_grid);

        unsigned int cellDistance(double world_dist) { return layered_costmap_->getCostmap()->cellDistance(world_dist); }

        inline void enqueue(unsigned char* grid, unsigned int index, unsigned int mx, unsigned int my,  unsigned int src_x, unsigned int src_y);

        double inflation_radius_, inscribed_radius_, weight_;
        unsigned int cell_inflation_radius_;
        std::priority_queue<CellData> inflation_queue_;
        
        double resolution_;
        
        bool* seen_;

        unsigned char** cached_costs_;
        double** cached_distances_;

  };
};
#endif

