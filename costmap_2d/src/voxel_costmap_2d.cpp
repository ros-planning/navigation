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
#include <costmap_2d/voxel_costmap_2d.h>

#define VOXEL_BITS 16

using namespace std;

namespace costmap_2d{
  VoxelCostmap2D::VoxelCostmap2D(unsigned int cells_size_x, unsigned int cells_size_y, unsigned int cells_size_z,
      double xy_resolution, double z_resolution, double origin_x, double origin_y, double origin_z, double inscribed_radius,
      double circumscribed_radius, double inflation_radius, double obstacle_range,
      double raytrace_range, double weight,
      const std::vector<unsigned char>& static_data, unsigned char lethal_threshold, unsigned int unknown_threshold, unsigned int mark_threshold, unsigned char unknown_cost_value)
    : Costmap2D(cells_size_x, cells_size_y, xy_resolution, origin_x, origin_y, inscribed_radius, circumscribed_radius,
        inflation_radius, obstacle_range, cells_size_z * z_resolution + origin_z, raytrace_range, weight, static_data, lethal_threshold, unknown_threshold < cells_size_z, unknown_cost_value),
    voxel_grid_(cells_size_x, cells_size_y, cells_size_z), xy_resolution_(xy_resolution), z_resolution_(z_resolution),
    origin_z_(origin_z), unknown_threshold_(unknown_threshold + (VOXEL_BITS - cells_size_z)), mark_threshold_(mark_threshold), size_z_(cells_size_z)
  {
  }

  VoxelCostmap2D::VoxelCostmap2D(costmap_2d::Costmap2D& costmap, 
      double z_resolution, unsigned int cells_size_z, double origin_z, 
      unsigned int mark_threshold, unsigned int unknown_threshold)
    : Costmap2D(costmap), voxel_grid_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY(), cells_size_z), 
        xy_resolution_(costmap.getResolution()), z_resolution_(z_resolution), origin_z_(origin_z), 
        unknown_threshold_(unknown_threshold + (VOXEL_BITS - cells_size_z)), mark_threshold_(mark_threshold), size_z_(cells_size_z)
  {
  }

  VoxelCostmap2D::~VoxelCostmap2D(){}

  void VoxelCostmap2D::initMaps(unsigned int size_x, unsigned int size_y){
    Costmap2D::initMaps(size_x, size_y);
    voxel_grid_.resize(size_x, size_y, size_z_);
    ROS_ASSERT(voxel_grid_.sizeX() == size_x_ && voxel_grid_.sizeY() == size_y_);
  }

  void VoxelCostmap2D::resetMaps(){
    Costmap2D::resetMaps();
    voxel_grid_.reset();
  }

  void VoxelCostmap2D::resetMapOutsideWindow(double wx, double wy, double w_size_x, double w_size_y){
    ROS_ASSERT_MSG(w_size_x >= 0 && w_size_y >= 0, "You cannot specify a negative size window");

    double start_point_x = wx - w_size_x / 2;
    double start_point_y = wy - w_size_y / 2;
    double end_point_x = start_point_x + w_size_x;
    double end_point_y = start_point_y + w_size_y;

    //check start bounds
    start_point_x = max(origin_x_, start_point_x);
    start_point_y = max(origin_y_, start_point_y);

    //check end bounds
    end_point_x = min(origin_x_ + getSizeInMetersX(), end_point_x);
    end_point_y = min(origin_y_ + getSizeInMetersY(), end_point_y);

    unsigned int start_x, start_y, end_x, end_y;

    //check for legality just in case
    if(!worldToMap(start_point_x, start_point_y, start_x, start_y) || !worldToMap(end_point_x, end_point_y, end_x, end_y))
      return;

    ROS_ASSERT(end_x > start_x && end_y > start_y);
    unsigned int cell_size_x = end_x - start_x;
    unsigned int cell_size_y = end_y - start_y;

    //we need a map to store the obstacles in the window temporarily
    unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];
    unsigned int* local_voxel_map = new unsigned int[cell_size_x * cell_size_y];
    unsigned int* voxel_map = voxel_grid_.getData();

    //copy the local window in the costmap to the local map
    copyMapRegion(costmap_, start_x, start_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);
    copyMapRegion(voxel_map, start_x, start_y, size_x_, local_voxel_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

    //now we'll reset the costmap to the static map
    memcpy(costmap_, static_map_, size_x_ * size_y_ * sizeof(unsigned char));

    //the voxel grid will just go back to being unknown
    voxel_grid_.reset();

    //now we want to copy the local map back into the costmap
    copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);
    copyMapRegion(local_voxel_map, 0, 0, cell_size_x, voxel_map, start_x, start_y, size_x_, cell_size_x, cell_size_y);

    //clean up
    delete[] local_map;
    delete[] local_voxel_map;
  }

  void VoxelCostmap2D::updateObstacles(const vector<Observation>& observations, priority_queue<CellData>& inflation_queue){
    //place the new obstacles into a priority queue... each with a priority of zero to begin with
    for(vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it){
      const Observation& obs = *it;

      const pcl::PointCloud<pcl::PointXYZ>& cloud =obs.cloud_;

      double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

      for(unsigned int i = 0; i < cloud.points.size(); ++i){
        //if the obstacle is too high or too far away from the robot we won't add it
        if(cloud.points[i].z > max_obstacle_height_)
          continue;

        //compute the squared distance from the hitpoint to the pointcloud's origin
        double sq_dist = (cloud.points[i].x - obs.origin_.x) * (cloud.points[i].x - obs.origin_.x)
          + (cloud.points[i].y - obs.origin_.y) * (cloud.points[i].y - obs.origin_.y)
          + (cloud.points[i].z - obs.origin_.z) * (cloud.points[i].z - obs.origin_.z);

        //if the point is far enough away... we won't consider it
        if(sq_dist >= sq_obstacle_range)
          continue;

        //now we need to compute the map coordinates for the observation
        unsigned int mx, my, mz;
        if(cloud.points[i].z < origin_z_){
          if(!worldToMap3D(cloud.points[i].x, cloud.points[i].y, origin_z_, mx, my, mz))
            continue;
        }
        else if(!worldToMap3D(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, mx, my, mz)){
          continue;
        }

        //mark the cell in the voxel grid and check if we should also mark it in the costmap
        if(voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_)){
          unsigned int index = getIndex(mx, my);

          //push the relevant cell index back onto the inflation queue
          enqueue(index, mx, my, mx, my, inflation_queue);
        }
      }
    }
  }

  void VoxelCostmap2D::raytraceFreespace(const Observation& clearing_observation){
    if(clearing_observation.cloud_.points.size() == 0)
      return;


    double sensor_x, sensor_y, sensor_z;
    double ox = clearing_observation.origin_.x;
    double oy = clearing_observation.origin_.y;
    double oz = clearing_observation.origin_.z;

    if(!worldToMap3DFloat(ox, oy, oz, sensor_x, sensor_y, sensor_z)){
      ROS_WARN_THROTTLE(1.0, "The origin for the sensor at (%.2f, %.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.", ox, oy, oz);
      return;
    }

    //we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
    double map_end_x = origin_x_ + getSizeInMetersX();
    double map_end_y = origin_y_ + getSizeInMetersY();

    for(unsigned int i = 0; i < clearing_observation.cloud_.points.size(); ++i){
      double wpx = clearing_observation.cloud_.points[i].x;
      double wpy = clearing_observation.cloud_.points[i].y;
      double wpz = clearing_observation.cloud_.points[i].z;

      double distance = dist(ox, oy, oz, wpx, wpy, wpz);
      double scaling_fact = 1.0;
      scaling_fact = std::max(std::min(scaling_fact, (distance -  2 * xy_resolution_) / distance), 0.0);
      wpx = scaling_fact * (wpx - ox) + ox;
      wpy = scaling_fact * (wpy - oy) + oy;
      wpz = scaling_fact * (wpz - oz) + oz;

      double a = wpx - ox;
      double b = wpy - oy;
      double c = wpz - oz;
      double t = 1.0;

      //we can only raytrace to a maximum z height
      if(wpz > max_obstacle_height_){
        //we know we want the vector's z value to be max_z
        t = std::min(t, (max_obstacle_height_ - 0.01 - oz) / c);
      }
      //and we can only raytrace down to the floor
      else if(wpz < origin_z_){
        //we know we want the vector's z value to be 0.0
        t = std::min(t, (origin_z_ - oz) / c);
      }

      //the minimum value to raytrace from is the origin
      if(wpx < origin_x_){
        t = std::min(t, (origin_x_ - ox) / a);
      }
      if(wpy < origin_y_){
        t = std::min(t, (origin_y_ - oy) / b);
      }

      //the maximum value to raytrace to is the end of the map
      if(wpx > map_end_x){
        t = std::min(t, (map_end_x - ox) / a);
      }
      if(wpy > map_end_y){
        t = std::min(t, (map_end_y - oy) / b);
      }

      wpx = ox + a * t;
      wpy = oy + b * t;
      wpz = oz + c * t;

      double point_x, point_y, point_z;
      if(worldToMap3DFloat(wpx, wpy, wpz, point_x, point_y, point_z)){
        unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
        
        //voxel_grid_.markVoxelLine(sensor_x, sensor_y, sensor_z, point_x, point_y, point_z);
        voxel_grid_.clearVoxelLineInMap(sensor_x, sensor_y, sensor_z, point_x, point_y, point_z, costmap_, 
            unknown_threshold_, mark_threshold_, FREE_SPACE, NO_INFORMATION, cell_raytrace_range);
      }
    }
  }


  void VoxelCostmap2D::updateOrigin(double new_origin_x, double new_origin_y){
    //project the new origin into the grid
    int cell_ox, cell_oy;
    cell_ox = int((new_origin_x - origin_x_) / resolution_);
    cell_oy = int((new_origin_y - origin_y_) / resolution_);

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
    lower_left_x = min(max(cell_ox, 0), size_x);
    lower_left_y = min(max(cell_oy, 0), size_y);
    upper_right_x = min(max(cell_ox + size_x, 0), size_x);
    upper_right_y = min(max(cell_oy + size_y, 0), size_y);

    unsigned int cell_size_x = upper_right_x - lower_left_x;
    unsigned int cell_size_y = upper_right_y - lower_left_y;

    //we need a map to store the obstacles in the window temporarily
    unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];
    unsigned int* local_voxel_map = new unsigned int[cell_size_x * cell_size_y];
    unsigned int* voxel_map = voxel_grid_.getData(); 

    //copy the local window in the costmap to the local map
    copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);
    copyMapRegion(voxel_map, lower_left_x, lower_left_y, size_x_, local_voxel_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

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

  void VoxelCostmap2D::clearNonLethal(double wx, double wy, double w_size_x, double w_size_y, bool clear_no_info){
    //get the cell coordinates of the center point of the window
    unsigned int mx, my;
    if(!worldToMap(wx, wy, mx, my))
      return;

    //compute the bounds of the window
    double start_x = wx - w_size_x / 2;
    double start_y = wy - w_size_y / 2;
    double end_x = start_x + w_size_x;
    double end_y = start_y + w_size_y;

    //scale the window based on the bounds of the costmap
    start_x = max(origin_x_, start_x);
    start_y = max(origin_y_, start_y);

    end_x = min(origin_x_ + getSizeInMetersX(), end_x);
    end_y = min(origin_y_ + getSizeInMetersY(), end_y);

    //get the map coordinates of the bounds of the window
    unsigned int map_sx, map_sy, map_ex, map_ey;

    //check for legality just in case
    if(!worldToMap(start_x, start_y, map_sx, map_sy) || !worldToMap(end_x, end_y, map_ex, map_ey))
      return;

    //we know that we want to clear all non-lethal obstacles in this window to get it ready for inflation
    unsigned int index = getIndex(map_sx, map_sy);
    unsigned char* current = &costmap_[index];
    for(unsigned int j = map_sy; j <= map_ey; ++j){
      for(unsigned int i = map_sx; i <= map_ex; ++i){
        //if the cell is a lethal obstacle... we'll keep it and queue it, otherwise... we'll clear it
        if(*current != LETHAL_OBSTACLE){
          if(clear_no_info || *current != NO_INFORMATION){
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

  void VoxelCostmap2D::getVoxelGridMessage(VoxelGrid& grid){
    unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
    grid.size_x = voxel_grid_.sizeX();
    grid.size_y = voxel_grid_.sizeY();
    grid.size_z = voxel_grid_.sizeZ();
    grid.data.resize(size);
    memcpy(&grid.data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

    grid.origin.x = origin_x_;
    grid.origin.y = origin_y_;
    grid.origin.z = origin_z_;

    grid.resolutions.x = xy_resolution_;
    grid.resolutions.y = xy_resolution_;
    grid.resolutions.z = z_resolution_;
  }

  void VoxelCostmap2D::getPoints(pcl::PointCloud<pcl::PointXYZ>& cloud){
    for(unsigned int i = 0; i < voxel_grid_.sizeX(); ++i){
      for(unsigned int j = 0; j < voxel_grid_.sizeY(); ++j){
        for(unsigned int k = 0; k < voxel_grid_.sizeZ(); ++k){
          if(voxel_grid_.getVoxel(i, j, k) == voxel_grid::MARKED){
            double wx, wy, wz;
            mapToWorld3D(i, j, k, wx, wy, wz);
            pcl::PointXYZ pt;
            pt.x = wx;
            pt.y = wy;
            pt.z = wz;
            cloud.points.push_back(pt);
          }
        }
      }
    }
  }

  void VoxelCostmap2D::finishConfiguration(costmap_2d::Costmap2DConfig &config) {
    unknown_threshold_ = config.unknown_threshold + (VOXEL_BITS - size_z_);
    mark_threshold_ = config.mark_threshold;
  }

};
