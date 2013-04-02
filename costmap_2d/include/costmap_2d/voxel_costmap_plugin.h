#ifndef VOXEL_COSTMAP_PLUGIN_H_
#define VOXEL_COSTMAP_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/plugin_ros.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/VoxelGrid.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/VoxelPluginConfig.h>
#include <costmap_2d/obstacle_costmap_plugin.h>
#include <voxel_grid/voxel_grid.h>
namespace common_costmap_plugins
{
  class VoxelCostmapPlugin : public common_costmap_plugins::ObstacleCostmapPlugin {
    public:
      VoxelCostmapPlugin() : voxel_grid_(0,0,0) { }

      void initialize(costmap_2d::LayeredCostmap* costmap, std::string name);
      void update_bounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
      void update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    void updateOrigin(double new_origin_x, double new_origin_y);
      bool isDiscretized() { return true; }
      void matchSize(); 

    private:
      void reconfigureCB(costmap_2d::VoxelPluginConfig &config, uint32_t level);
      void clearNonLethal(double wx, double wy, double w_size_x, double w_size_y, bool clear_no_info);
            virtual void raytraceFreespace(const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y, double* max_x, double* max_y);
      void initMaps();
      
      dynamic_reconfigure::Server<costmap_2d::VoxelPluginConfig> *dsrv_;
        
      bool publish_voxel_;
      ros::Publisher voxel_pub_;
      voxel_grid::VoxelGrid voxel_grid_;
      double xy_resolution_, z_resolution_, origin_z_;
      unsigned int unknown_threshold_, mark_threshold_, size_z_;


inline bool worldToMap3DFloat(double wx, double wy, double wz, double& mx, double& my, double& mz){
        if(wx < origin_x_ || wy < origin_y_ || wz < origin_z_)
          return false;
        mx = ((wx - origin_x_) / xy_resolution_);
        my = ((wy - origin_y_) / xy_resolution_);
        mz = ((wz - origin_z_) / z_resolution_);

        if(mx < size_x_ && my < size_y_ && mz < size_z_)
          return true;

        return false;
      }

      inline bool worldToMap3D(double wx, double wy, double wz, unsigned int& mx, unsigned int& my, unsigned int& mz){
        if(wx < origin_x_ || wy < origin_y_ || wz < origin_z_)
          return false;

        mx = (int) ((wx - origin_x_) / xy_resolution_);
        my = (int) ((wy - origin_y_) / xy_resolution_);
        mz = (int) ((wz - origin_z_) / z_resolution_);

        if(mx < size_x_ && my < size_y_ && mz < size_z_)
          return true;

        return false;
      }

      inline void mapToWorld3D(unsigned int mx, unsigned int my, unsigned int mz, double& wx, double& wy, double& wz){
        //returns the center point of the cell
        wx = origin_x_ + (mx + 0.5) * xy_resolution_;
        wy = origin_y_ + (my + 0.5) * xy_resolution_;
        wz = origin_z_ + (mz + 0.5) * z_resolution_;
      }

      inline double dist(double x0, double y0, double z0, double x1, double y1, double z1){
        return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
      }

  };
};
#endif

