/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Eric Perko
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
 *   * Neither the name of Eric Perko nor the names of its
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
#include <base_local_planner/map_grid_visualizer.h>
#include <base_local_planner/map_cell.h>
#include <vector>

namespace base_local_planner {
  MapGridVisualizer::MapGridVisualizer() {}


  void MapGridVisualizer::initialize(const std::string& name, boost::function<bool (int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost)> cost_function) {
    name_ = name;
    cost_function_ = cost_function;

    ns_nh_ = ros::NodeHandle("~/" + name_);
    ns_nh_.param("global_frame_id", frame_id_, std::string("odom"));

    cost_cloud_.header.frame_id = frame_id_;
    pub_.advertise(ns_nh_, "cost_cloud", 1);
  }

  void MapGridVisualizer::publishCostCloud(const costmap_2d::Costmap2D* costmap_p_) {
    unsigned int x_size = costmap_p_->getSizeInCellsX();
    unsigned int y_size = costmap_p_->getSizeInCellsY();
    double z_coord = 0.0;
    double x_coord, y_coord;
    MapGridCostPoint pt;
    cost_cloud_.points.clear();
    cost_cloud_.header.stamp = ros::Time::now();
    float path_cost, goal_cost, occ_cost, total_cost;
    for (unsigned int cx = 0; cx < x_size; cx++) {
      for (unsigned int cy = 0; cy < y_size; cy++) {
        costmap_p_->mapToWorld(cx, cy, x_coord, y_coord);
        if (cost_function_(cx, cy, path_cost, goal_cost, occ_cost, total_cost)) {
          pt.x = x_coord;
          pt.y = y_coord;
          pt.z = z_coord;
          pt.path_cost = path_cost;
          pt.goal_cost = goal_cost;
          pt.occ_cost = occ_cost;
          pt.total_cost = total_cost;
          cost_cloud_.push_back(pt);
        }
      }
    }
    pub_.publish(cost_cloud_);
    ROS_DEBUG("Cost PointCloud published");
  }
};
