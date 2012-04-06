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
#include <base_local_planner/map_grid.h>

using namespace std;

namespace base_local_planner{

  MapGrid::MapGrid()
    : size_x_(0), size_y_(0)
  {
  }

  MapGrid::MapGrid(unsigned int size_x, unsigned int size_y) 
    : size_x_(size_x), size_y_(size_y)
  {
    commonInit();
  }

  MapGrid::MapGrid(unsigned int size_x, unsigned int size_y, double s, double x, double y)
    : size_x_(size_x), size_y_(size_y), scale(s), origin_x(x), origin_y(y)
  {
    commonInit();
  }

  MapGrid::MapGrid(const MapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
  }

  void MapGrid::commonInit(){
    //don't allow construction of zero size grid
    ROS_ASSERT(size_y_ != 0 && size_x_ != 0);

    map_.resize(size_y_ * size_x_);

    //make each cell aware of its location in the grid
    for(unsigned int i = 0; i < size_y_; ++i){
      for(unsigned int j = 0; j < size_x_; ++j){
        unsigned int id = size_x_ * i + j;
        map_[id].cx = j;
        map_[id].cy = i;
      }
    }
  }

  size_t MapGrid::getIndex(int x, int y){
    return size_x_ * y + x;
  }

  MapGrid& MapGrid::operator= (const MapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
    return *this;
  }

  void MapGrid::sizeCheck(unsigned int size_x, unsigned int size_y, double o_x, double o_y){
    if(map_.size() != size_x * size_y)
      map_.resize(size_x * size_y);

    if(size_x_ != size_x || size_y_ != size_y){
      size_x_ = size_x;
      size_y_ = size_y;

      for(unsigned int i = 0; i < size_y_; ++i){
        for(unsigned int j = 0; j < size_x_; ++j){
          int index = size_x_ * i + j;
          map_[index].cx = j;
          map_[index].cy = i;
        }
      }
    }
    origin_x = o_x;
    origin_y = o_y;
  }

  //reset the path_dist and goal_dist fields for all cells
  void MapGrid::resetPathDist(){
    for(unsigned int i = 0; i < map_.size(); ++i){
      map_[i].path_dist = unreachableCellCosts();
      map_[i].goal_dist = unreachableCellCosts();
      map_[i].path_mark = false;
      map_[i].goal_mark = false;
      map_[i].within_robot = false;
    }
  }

  //update what map cells are considered path based on the global_plan
  void MapGrid::setPathCells(const costmap_2d::Costmap2D& costmap, const std::vector<geometry_msgs::PoseStamped>& global_plan){
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY(), costmap.getOriginX(), costmap.getOriginY());
    int local_goal_x = -1;
    int local_goal_y = -1;
    bool started_path = false;
    queue<MapCell*> path_dist_queue;
    queue<MapCell*> goal_dist_queue;
    for (unsigned int i = 0; i < global_plan.size(); ++i) {
      double g_x = global_plan[i].pose.position.x;
      double g_y = global_plan[i].pose.position.y;
      unsigned int map_x, map_y;
      if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) {
        MapCell& current = getCell(map_x, map_y);
        current.path_dist = 0.0;
        current.path_mark = true;
        path_dist_queue.push(&current);
        local_goal_x = map_x;
        local_goal_y = map_y;
        started_path = true;
      } else {
        if(started_path) {
          break;
        }
      }
    }

    if (local_goal_x >= 0 && local_goal_y >= 0) {
      MapCell& current = getCell(local_goal_x, local_goal_y);
      costmap.mapToWorld(local_goal_x, local_goal_y, goal_x_, goal_y_);
      current.goal_dist = 0.0;
      current.goal_mark = true;
      goal_dist_queue.push(&current);
    }
    //compute our distances
    computePathDistance(path_dist_queue, costmap);
    computeGoalDistance(goal_dist_queue, costmap);
  }

  void MapGrid::computePathDistance(queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap){
    MapCell* current_cell;
    MapCell* check_cell;
    unsigned int last_col = size_x_ - 1;
    unsigned int last_row = size_y_ - 1;
    while(!dist_queue.empty()){
      current_cell = dist_queue.front();
      check_cell = current_cell;
      dist_queue.pop();

      if(current_cell->cx > 0){
        check_cell = current_cell - 1;
        if(!check_cell->path_mark){
          updatePathCell(current_cell, check_cell, dist_queue, costmap);
        }
      }

      if(current_cell->cx < last_col){
        check_cell = current_cell + 1;
        if(!check_cell->path_mark){
          updatePathCell(current_cell, check_cell, dist_queue, costmap);
        }
      }

      if(current_cell->cy > 0){
        check_cell = current_cell - size_x_;
        if(!check_cell->path_mark){
          updatePathCell(current_cell, check_cell, dist_queue, costmap);
        }
      }

      if(current_cell->cy < last_row){
        check_cell = current_cell + size_x_;
        if(!check_cell->path_mark){
          updatePathCell(current_cell, check_cell, dist_queue, costmap);
        }
      }
    }
  }

  void MapGrid::computeGoalDistance(queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap){
    MapCell* current_cell;
    MapCell* check_cell;
    unsigned int last_col = size_x_ - 1;
    unsigned int last_row = size_y_ - 1;
    while(!dist_queue.empty()){
      current_cell = dist_queue.front();
      current_cell->goal_mark = true;
      check_cell = current_cell;
      dist_queue.pop();

      if(current_cell->cx > 0){
        check_cell = current_cell - 1;
        if(!check_cell->goal_mark){
          updateGoalCell(current_cell, check_cell, dist_queue, costmap);
        }
      }

      if(current_cell->cx < last_col){
        check_cell = current_cell + 1;
        if(!check_cell->goal_mark){
          updateGoalCell(current_cell, check_cell, dist_queue, costmap);
        }
      }

      if(current_cell->cy > 0){
        check_cell = current_cell - size_x_;
        if(!check_cell->goal_mark){
          updateGoalCell(current_cell, check_cell, dist_queue, costmap);
        }
      }

      if(current_cell->cy < last_row){
        check_cell = current_cell + size_x_;
        if(!check_cell->goal_mark){
          updateGoalCell(current_cell, check_cell, dist_queue, costmap);
        }
      }
    }
  }


};
