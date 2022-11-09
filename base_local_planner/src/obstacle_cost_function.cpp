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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/common.h>
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>
#include <iostream>
#include <fstream>

namespace base_local_planner {

ObstacleCostFunction::ObstacleCostFunction(costmap_2d::Costmap2D* costmap) 
    : costmap_(costmap), sum_scores_(false) {
  if (costmap != NULL) {
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
  }
}

ObstacleCostFunction::~ObstacleCostFunction() {
  if (world_model_ != NULL) {
    delete world_model_;
  }
}


void ObstacleCostFunction::setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed) {
  // TODO: move this to prepare if possible
  max_trans_vel_ = max_trans_vel;
  max_scaling_factor_ = max_scaling_factor;
  scaling_speed_ = scaling_speed;
}

void ObstacleCostFunction::setFootprint(std::vector<geometry_msgs::Point> footprint_spec) {
  footprint_spec_ = footprint_spec;
}

void ObstacleCostFunction::setCargoAngle(double cargo_angle)
{
  this->cargo_angle_ = cargo_angle;
}

void ObstacleCostFunction::setCargoEnabled(bool is_cargo_enabled)
{
  this->is_cargo_enabled_ = is_cargo_enabled;
}

bool ObstacleCostFunction::prepare() {
  return true;
}

double ObstacleCostFunction::calc_distance(double x0, double y0, double x1, double y1, double th1)
{
  double x2 = x1 + std::cos(th1);
  double y2 = y1 + std::sin(th1);

  // distance from point to line
  double f1 = (x2 - x1) * (y1 - y0) - (y2 - y1) * (x1 - x0);
  double r2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
  return std::sqrt((f1 * f1) / r2);
}

double ObstacleCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0;
  double scale = getScalingFactor(traj, scaling_speed_, max_trans_vel_, max_scaling_factor_);
  double px, py, pth;
  if (footprint_spec_.size() == 0) {
    // Bug, should never happen
    ROS_ERROR("Footprint spec is empty, maybe missing call to setFootprint?");
    return -9;
  }
  double robot_x, robot_y, robot_th;
  if (0 < traj.getPointsSize())
  {
    traj.getPoint(0, robot_x, robot_y, robot_th);
  }
  double cargo_global = base_local_planner::normalize(robot_th + this->cargo_angle_);
  double cargo_rear_x = robot_x + std::cos(cargo_global) * 0.95;
  double cargo_rear_y = robot_y + std::sin(cargo_global) * 0.95;
  const char *fileName = "/home/lexxauto/LexxAuto/log.txt";
  std::ofstream ofs(fileName, std::ios::app);

  // ROS_WARN_STREAM("######## ------------------------------------------");
  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, px, py, pth);
    ofs << i << "," << 0 << "," << px << "," << py << "," << pth << "," << this->cargo_angle_ << std::endl;
    if (this->is_cargo_enabled_)
    {

        double angle_a = cargo_global;

        // Calculate angle formed by vector A and B.
        double a1 = px - cargo_rear_x;
        double a2 = py - cargo_rear_y;
        double b1 = robot_x - cargo_rear_x;
        double b2 = robot_y - cargo_rear_y;

        double cos_th = a1 * b1 + a2 * b2 /
            (std::sqrt(a1 * a1 + a2 * a2) * std::sqrt(b1 * b1 + b2 * b2));
        
        // angle formed by A and B become new_cargo_angle
        double angle_b = std::acos(cos_th);
        double angle_c = base_local_planner::normalize(angle_a + angle_b);
        double new_cargo_global = base_local_planner::normalize(angle_c);
        double cargo_delta = base_local_planner::normalize(cargo_global - new_cargo_global);

        // ROS_WARN_STREAM(i << "," << px << "," << py << "," << pth << "," << robot_x << "," << robot_y << "," << cargo_rear_x << "," << cargo_rear_y << "," << this->cargo_angle_ << "," << cargo_global << "," << cargo_delta << "," << new_cargo_global);

        // difference of cargo_angle in global coordinates
        // is rotation amount of the footprint.
        pth = base_local_planner::normalize(robot_th + cargo_delta);

        robot_x = px;
        robot_y = py;
        robot_th = pth;
        cargo_global = new_cargo_global;
        cargo_rear_x = robot_x + std::cos(cargo_global) * 0.95;
        cargo_rear_y = robot_y + std::sin(cargo_global) * 0.95;
    }

    double cos_th = cos(pth);
    double sin_th = sin(pth);

    std::vector<geometry_msgs::Point> oriented_footprint;
    for(unsigned int j = 0; j < footprint_spec_.size(); ++j){
      geometry_msgs::Point new_pt;
      new_pt.x = px + (footprint_spec_[j].x * cos_th - footprint_spec_[j].y * sin_th);
      new_pt.y = py + (footprint_spec_[j].x * sin_th + footprint_spec_[j].y * cos_th);
      oriented_footprint.push_back(new_pt);
    }

    for (size_t j = 0; j < oriented_footprint.size(); j++)
    {
      auto pt = oriented_footprint[j];
      ofs << i << "," << (j + 1) << "," << pt.x << "," << pt.y << std::endl;
    }

    double f_cost = footprintCost(px, py, pth,
        scale, footprint_spec_,
        costmap_, world_model_);

    if(f_cost < 0){
        return f_cost;
    }

    if(sum_scores_)
        cost +=  f_cost;
    else
        cost = std::max(cost, f_cost);
  }
  return cost;
}

double ObstacleCostFunction::getScalingFactor(Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor) {
  double vmag = hypot(traj.xv_, traj.yv_);

  //if we're over a certain speed threshold, we'll scale the robot's
  //footprint to make it either slow down or stay further from walls
  double scale = 1.0;
  if (vmag > scaling_speed) {
    //scale up to the max scaling factor linearly... this could be changed later
    double ratio = (vmag - scaling_speed) / (max_trans_vel - scaling_speed);
    scale = max_scaling_factor * ratio + 1.0;
  }
  return scale;
}

double ObstacleCostFunction::footprintCost (
    const double& x,
    const double& y,
    const double& th,
    double scale,
    std::vector<geometry_msgs::Point> footprint_spec,
    costmap_2d::Costmap2D* costmap,
    base_local_planner::WorldModel* world_model) {

  //check if the footprint is legal
  // TODO: Cache inscribed radius
  double footprint_cost = world_model->footprintCost(x, y, th, footprint_spec);

  if (footprint_cost < 0) {
    return -6.0;
  }
  unsigned int cell_x, cell_y;

  //we won't allow trajectories that go off the map... shouldn't happen that often anyways
  if ( ! costmap->worldToMap(x, y, cell_x, cell_y)) {
    return -7.0;
  }

  double occ_cost = std::max(std::max(0.0, footprint_cost), double(costmap->getCost(cell_x, cell_y)));

  return occ_cost;
}

} /* namespace base_local_planner */
