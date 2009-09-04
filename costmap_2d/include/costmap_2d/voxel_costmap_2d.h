/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef COSTMAP_VOXEL_COSTMAP_2D_H_
#define COSTMAP_VOXEL_COSTMAP_2D_H_

#include <vector>
#include <queue>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/observation.h>
#include <costmap_2d/cell_data.h>
#include <costmap_2d/cost_values.h>
#include <voxel_grid/voxel_grid.h>
#include <costmap_2d/VoxelGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <boost/thread.hpp>

namespace costmap_2d {
  /**
   * @class VoxelCostmap2D
   * @brief A 2D costmap provides a mapping between points in the world and their associated "costs".
   */
  class VoxelCostmap2D : public Costmap2D {
    public:
      /**
       * @brief  Constructor for a voxel grid based costmap
       * @param  cells_size_x The x size of the map in cells
       * @param  cells_size_y The y size of the map in cells
       * @param  cells_size_z The z size of the map in cells... up to 32 cells max
       * @param  xy_resolution The horizontal resolution of the map in meters/cell
       * @param  z_resolution The vertical resolution of the map in meters/cell
       * @param  origin_x The x origin of the map
       * @param  origin_y The y origin of the map
       * @param  origin_z The z origin of the map
       * @param  inscribed_radius The inscribed radius of the robot
       * @param  circumscribed_radius The circumscribed radius of the robot
       * @param  inflation_radius How far out to inflate obstacles
       * @param  obstacle_range The maximum range at which obstacles will be put into the costmap
       * @param  raytrace_range The maximum distance we'll raytrace out to
       * @param  weight The scaling factor for the cost function. Should be 0 < weight <= 1. Lower values reduce effective cost.
       * @param  static_data Data used to initialize the costmap
       * @param  lethal_threshold The cost threshold at which a point in the static data is considered a lethal obstacle
       * @param  unknown_threshold The maximum number of unknown voxel cells that can exist in a column considered as free space
       * @param  mark_threshold The maximum number of marked voxel cells that can exist in a column considered as free space
       */
      VoxelCostmap2D(unsigned int cells_size_x, unsigned int cells_size_y, unsigned int cells_size_z,
          double xy_resolution, double z_resolution, double origin_x, double origin_y, double origin_z = 0.0, double inscribed_radius = 0.0,
          double circumscribed_radius = 0.0, double inflation_radius = 0.0, double obstacle_range = 0.0,
          double raytrace_range = 0.0, double weight = 25.0,
          const std::vector<unsigned char>& static_data = std::vector<unsigned char>(0), unsigned char lethal_threshold = 0,
          unsigned int unknown_threshold = 0, unsigned int mark_threshold = 0);

      /**
       * @brief  Destructor
       */
      ~VoxelCostmap2D();

      /**
       * @brief  Revert to the static map outside of a specified window centered at a world coordinate
       * @param wx The x coordinate of the center point of the window in world space (meters)
       * @param wy The y coordinate of the center point of the window in world space (meters)
       * @param w_size_x The x size of the window in meters
       * @param w_size_y The y size of the window in meters
       */
      void resetMapOutsideWindow(double wx, double wy, double w_size_x, double w_size_y);

      /**
       * @brief  Move the origin of the costmap to a new location.... keeping data when it can
       * @param  new_origin_x The x coordinate of the new origin
       * @param  new_origin_y The y coordinate of the new origin
       */
      void updateOrigin(double new_origin_x, double new_origin_y);

      /**
       * @brief Returns a point cloud for visualizing the voxel grid
       * @param cloud The point cloud to fill
       */
      void getPoints(sensor_msgs::PointCloud& cloud);

      void getVoxelGridMessage(VoxelGrid& grid);

      static inline void mapToWorld3D(const unsigned int mx, const unsigned int my, const unsigned int mz,
                                      const double origin_x, const double origin_y, const double origin_z,
                                      const double x_resolution, const double y_resolution, const double z_resolution,
                                      double& wx, double& wy, double& wz){
        //returns the center point of the cell
        wx = origin_x + (mx + 0.5) * x_resolution;
        wy = origin_y + (my + 0.5) * y_resolution;
        wz = origin_z + (mz + 0.5) * z_resolution;
      }

    private:
      /**
       * @brief  Insert new obstacles into the cost map
       * @param obstacles The point clouds of obstacles to insert into the map
       * @param inflation_queue The queue to place the obstacles into for inflation
       */
      void updateObstacles(const std::vector<Observation>& observations, std::priority_queue<CellData>& inflation_queue);

      /**
       * @brief  Clear freespace from an observation
       * @param clearing_observation The observation used to raytrace
       */
      void raytraceFreespace(const Observation& clearing_observation);

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

    protected:
      voxel_grid::VoxelGrid voxel_grid_;
      double xy_resolution_, z_resolution_, origin_z_;
      unsigned int unknown_threshold_, mark_threshold_, size_z_;

  };
};

#endif
