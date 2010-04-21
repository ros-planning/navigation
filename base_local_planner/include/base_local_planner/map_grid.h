/*********************************************************************
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
 *********************************************************************/
#ifndef TRAJECTORY_ROLLOUT_MAP_GRID_H_
#define TRAJECTORY_ROLLOUT_MAP_GRID_H_

#include <vector>
#include <iostream>
#include <base_local_planner/trajectory_inc.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <base_local_planner/map_cell.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

namespace base_local_planner{
  /**
   * @class MapGrid
   * @brief A grid of MapCell cells that is used to propagate path and goal distances for the trajectory controller.
   */
  class MapGrid{
    public:
      /**
       * @brief  Creates a map of size_x by size_y
       * @param size_x The width of the map 
       * @param size_y The height of the map 
       */
      MapGrid(unsigned int size_x, unsigned int size_y);

      /**
       * @brief  Creates a map of size_x by size_y with the desired scale and origin
       * @param size_x The width of the map 
       * @param size_y The height of the map 
       * @param scale The resolution of each MapCell
       * @param x The x coordinate of the origin of the map
       * @param y The y coordinate of the origin of the map
       */
      MapGrid(unsigned int size_x, unsigned int size_y, double scale, double x, double y);

      /**
       * @brief  Returns a map cell accessed by (col, row)
       * @param x The x coordinate of the cell 
       * @param y The y coordinate of the cell 
       * @return A reference to the desired cell
       */
      inline MapCell& operator() (unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
      }

      /**
       * @brief  Returns a map cell accessed by (col, row)
       * @param x The x coordinate of the cell 
       * @param y The y coordinate of the cell 
       * @return A copy of the desired cell
       */
      inline MapCell operator() (unsigned int x, unsigned int y) const {
        return map_[size_x_ * y + x];
      }

      inline MapCell& getCell(unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
      }

      /**
       * @brief  Destructor for a MapGrid
       */
      ~MapGrid(){}

      /**
       * @brief  Copy constructor for a MapGrid
       * @param mg The MapGrid to copy 
       */
      MapGrid(const MapGrid& mg);

      /**
       * @brief  Assignment operator for a MapGrid
       * @param mg The MapGrid to assign from 
       */
      MapGrid& operator= (const MapGrid& mg);

      /**
       * @brief reset path distance fields for all cells
       */
      void resetPathDist();

      /**
       * @brief  check if we need to resize
       * @param size_x The desired width
       * @param size_y The desired height
       * @param o_x the desired x coordinate of the origin
       * @param o_y the desired y coordinate of the origin
       */
      void sizeCheck(unsigned int size_x, unsigned int size_y, double o_x, double o_y);

      /**
       * @brief Utility to share initialization code across constructors
       */
      void commonInit();

      /**
       * @brief  Returns a 1D index into the MapCell array for a 2D index
       * @param x The desired x coordinate
       * @param y The desired y coordinate
       * @return The associated 1D index 
       */
      size_t getIndex(int x, int y);

      /**
       * @brief  Used to update the distance of a cell in path distance computation
       * @param  current_cell The cell we're currently in 
       * @param  check_cell The cell to be updated
       */
      inline void updatePathCell(MapCell* current_cell, MapCell* check_cell, 
          std::queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap){
        //mark the cell as visisted
        check_cell->path_mark = true;

        //if the cell is an obstacle set the max path distance
        unsigned char cost = costmap.getCost(check_cell->cx, check_cell->cy);
        if(!getCell(check_cell->cx, check_cell->cy).within_robot && (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost == costmap_2d::NO_INFORMATION)){
          check_cell->path_dist = map_.size();
          return;
        }

        double new_path_dist = current_cell->path_dist + 1;
        if(new_path_dist < check_cell->path_dist)
          check_cell->path_dist = new_path_dist;

        dist_queue.push(check_cell);
      }

      /**
       * @brief  Used to update the distance of a cell in goal distance computation
       * @param  current_cell The cell we're currently in 
       * @param  check_cell The cell to be updated
       */
      inline void updateGoalCell(MapCell* current_cell, MapCell* check_cell, 
          std::queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap ){
        ///mark the cell as visited
        check_cell->goal_mark = true;

        //if the cell is an obstacle set the max goal distance
        unsigned char cost = costmap.getCost(check_cell->cx, check_cell->cy);
        if(!getCell(check_cell->cx, check_cell->cy).within_robot && (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost == costmap_2d::NO_INFORMATION)){
          check_cell->goal_dist = map_.size();
          return;
        }

        double new_goal_dist = current_cell->goal_dist + 1;
        if(new_goal_dist < check_cell->goal_dist)
          check_cell->goal_dist = new_goal_dist;

        dist_queue.push(check_cell);
      }

      /**
       * @brief  Compute the distance from each cell in the local map grid to the planned path
       * @param dist_queue A queue of the initial cells on the path 
       */
      void computePathDistance(std::queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap);

      /**
       * @brief  Compute the distance from each cell in the local map grid to the local goal point
       * @param goal_queue A queue containing the local goal cell 
       */
      void computeGoalDistance(std::queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap);

      /**
       * @brief Update what cells are considered path based on the global plan 
       */
      void setPathCells(const costmap_2d::Costmap2D& costmap, const std::vector<geometry_msgs::PoseStamped>& global_plan);

      unsigned int size_x_, size_y_; ///< @brief The dimensions of the grid
      std::vector<MapCell> map_; ///< @brief Storage for the MapCells

      double scale; ///< @brief grid scale in meters/cell

      double goal_x_, goal_y_; /**< @brief The goal distance was last computed from */

      double origin_x, origin_y; ///< @brief lower left corner of grid in world space
  };
};

#endif
