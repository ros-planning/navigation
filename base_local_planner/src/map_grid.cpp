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
#include <costmap_2d/cost_values.h>
#include <mbf_msgs/ExePathAction.h>

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

  void MapGrid::sizeCheck(unsigned int size_x, unsigned int size_y){
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
  }


  inline bool MapGrid::updatePathCell(MapCell* current_cell, MapCell* check_cell,
      const costmap_2d::Costmap2D& costmap){

    //if the cell is an obstacle set the max path distance
    unsigned char cost = costmap.getCost(check_cell->cx, check_cell->cy);
    if(! getCell(check_cell->cx, check_cell->cy).within_robot &&
        (cost == costmap_2d::LETHAL_OBSTACLE ||
         cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
         cost == costmap_2d::NO_INFORMATION)){
      check_cell->target_dist = obstacleCosts();
      return false;
    }

    double new_target_dist = current_cell->target_dist + 1;
    if (new_target_dist < check_cell->target_dist) {
      check_cell->target_dist = new_target_dist;
    }
    return true;
  }


  //reset the path_dist and goal_dist fields for all cells
  void MapGrid::resetPathDist(){
    for(unsigned int i = 0; i < map_.size(); ++i) {
      map_[i].target_dist = unreachableCellCosts();
      map_[i].target_mark = false;
      map_[i].within_robot = false;
    }
  }

  void MapGrid::adjustPlanResolution(const std::vector<geometry_msgs::PoseStamped>& global_plan_in,
      std::vector<geometry_msgs::PoseStamped>& global_plan_out, double resolution) {
    if (global_plan_in.size() == 0) {
      return;
    }
    double last_x = global_plan_in[0].pose.position.x;
    double last_y = global_plan_in[0].pose.position.y;
    global_plan_out.push_back(global_plan_in[0]);

    double min_sq_resolution = resolution * resolution;

    for (unsigned int i = 1; i < global_plan_in.size(); ++i) {
      double loop_x = global_plan_in[i].pose.position.x;
      double loop_y = global_plan_in[i].pose.position.y;
      double sqdist = (loop_x - last_x) * (loop_x - last_x) + (loop_y - last_y) * (loop_y - last_y);
      if (sqdist > min_sq_resolution) {
        int steps = ceil((sqrt(sqdist)) / resolution);
        // add a points in-between
        double deltax = (loop_x - last_x) / steps;
        double deltay = (loop_y - last_y) / steps;
        // TODO: Interpolate orientation
        for (int j = 1; j < steps; ++j) {
          geometry_msgs::PoseStamped pose;
          pose.pose.position.x = last_x + j * deltax;
          pose.pose.position.y = last_y + j * deltay;
          pose.pose.position.z = global_plan_in[i].pose.position.z;
          pose.pose.orientation = global_plan_in[i].pose.orientation;
          pose.header = global_plan_in[i].header;
          global_plan_out.push_back(pose);
        }
      }
      global_plan_out.push_back(global_plan_in[i]);
      last_x = loop_x;
      last_y = loop_y;
    }
  }

  //update what map cells are considered path based on the global_plan
  mbf_msgs::ExePathResult::_outcome_type MapGrid::setTargetCells(const costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::PoseStamped>& global_plan) {
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    bool started_path = false;

    queue<MapCell*> path_dist_queue;

    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan, costmap.getResolution());
    if (adjusted_global_plan.size() != global_plan.size()) {
      ROS_DEBUG("Adjusted global plan resolution, added %zu points", adjusted_global_plan.size() - global_plan.size());
    }
    unsigned int i;
    // put global path points into local map until we reach the border of the local map
    for (i = 0; i < adjusted_global_plan.size(); ++i) {
      double g_x = adjusted_global_plan[i].pose.position.x;
      double g_y = adjusted_global_plan[i].pose.position.y;
      unsigned int map_x, map_y;
      if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) {
        MapCell& current = getCell(map_x, map_y);
        current.target_dist = 0.0;
        current.target_mark = true;
        path_dist_queue.push(&current);
        started_path = true;
      } else if (started_path) {
          break;
      }
    }
    if (!started_path) {
      ROS_ERROR("None of the %d first of %zu (%zu) points of the global plan were in the local costmap and free",
          i, adjusted_global_plan.size(), global_plan.size());
      return mbf_msgs::ExePathResult::OUT_OF_MAP;
    }

    computeTargetDistance(path_dist_queue, costmap);
    return mbf_msgs::ExePathResult::SUCCESS;
  }

  //mark the point of the costmap as local goal where global_plan first leaves the area (or its last point)
  mbf_msgs::ExePathResult::_outcome_type MapGrid::setLocalGoal(const costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::PoseStamped>& global_plan) {
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    int local_goal_x = -1;
    int local_goal_y = -1;
    bool started_path = false;

    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan, costmap.getResolution());

    auto dist_squared_to_start = [&adjusted_global_plan](const double x, const double y) {
      const double dx = x - adjusted_global_plan.front().pose.position.x;
      const double dy = y - adjusted_global_plan.front().pose.position.y;
      return dx * dx + dy * dy;
    };
    bool reached_point_away_from_start = false;
    bool local_goal_in_obstacle = true;

    // In the following loop, we select a point on the global path as the local goal based on which
    // we compute the "distance to the goal". The following criteria are chosen:
    // - local goal needs to be on the map (otherwise, goal does not correspond to a cell)
    // - local goal needs to be on a cell where the cost is not NO_INFORMATION
    //   (as NO_INFORMATION is treated as obstacle in updatePathCell())
    //
    // Additionally, cells with INSCRIBED_INFLATED_OBSTACLE and LETHAL_OBSTACLE cost are also treated as
    // an obstacle in updatePathCell()
    // (This is such that narrow pathways are discarded as potential paths to the goal)
    // So if the neighboring cells of the local goal cell all have cost of INSCRIBED_INFLATED_OBSTACLE
    // or LETHAL_OBSTACLE, the goal will be considered unreachable (the local planner will reject all trajectories).
    // In this case, it is however unreasonable for the local planner to give up already when still far away
    // from the local goal, as the obstacle on it might disappear or the global plan might get updated.
    // Hence, once we reached a path point significantly far away from the beginning of the path and we
    // encounter a cell with INSCRIBED_INFLATED_OBSTACLE / LETHAL_OBSTACLE cost, we can stop searching if
    // a local goal significantly far away from the beginning of the path was already found that is not in an obstacle.
    for (unsigned int i = 0; i < adjusted_global_plan.size(); ++i) {
      double g_x = adjusted_global_plan[i].pose.position.x;
      double g_y = adjusted_global_plan[i].pose.position.y;
      unsigned int map_x, map_y;
      reached_point_away_from_start = reached_point_away_from_start || dist_squared_to_start(g_x, g_y) > 1;
      if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) {
        const bool in_obstacle = costmap.getCost(map_x, map_y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        if (reached_point_away_from_start && in_obstacle && !local_goal_in_obstacle) {
          break;
        }
        local_goal_x = map_x;
        local_goal_y = map_y;
        started_path = true;
        local_goal_in_obstacle = in_obstacle;
      } else {
        if (started_path) {
          ROS_WARN_COND(local_goal_in_obstacle, "local goal in obstacle");
          break;
        }// else we might have a non pruned path, so we just continue
      }
    }
    if (!started_path) {
      ROS_ERROR("None of the points of the global plan were in the local costmap, global plan points too far from robot");
      return mbf_msgs::ExePathResult::OUT_OF_MAP;
    }

    if (local_goal_in_obstacle) {
      ROS_WARN("None of the points of the global plan sufficiently far away from the start were in free space");
      return mbf_msgs::ExePathResult::BLOCKED_PATH;
    }

    queue<MapCell*> path_dist_queue;
    if (local_goal_x >= 0 && local_goal_y >= 0) {
      MapCell& current = getCell(local_goal_x, local_goal_y);
      costmap.mapToWorld(local_goal_x, local_goal_y, goal_x_, goal_y_);
      current.target_dist = 0.0;
      current.target_mark = true;
      path_dist_queue.push(&current);
    }

    computeTargetDistance(path_dist_queue, costmap);
    return mbf_msgs::ExePathResult::SUCCESS;
  }



  void MapGrid::computeTargetDistance(queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap){
    MapCell* current_cell;
    MapCell* check_cell;
    unsigned int last_col = size_x_ - 1;
    unsigned int last_row = size_y_ - 1;
    while(!dist_queue.empty()){
      current_cell = dist_queue.front();


      dist_queue.pop();

      if(current_cell->cx > 0){
        check_cell = current_cell - 1;
        if(!check_cell->target_mark){
          //mark the cell as visisted
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cx < last_col){
        check_cell = current_cell + 1;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cy > 0){
        check_cell = current_cell - size_x_;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cy < last_row){
        check_cell = current_cell + size_x_;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }
    }
  }

};
