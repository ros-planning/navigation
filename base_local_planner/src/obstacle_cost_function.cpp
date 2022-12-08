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
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>
#include <std_msgs/Float32.h>

namespace base_local_planner {

ObstacleCostFunction::ObstacleCostFunction(costmap_2d::Costmap2D* costmap)
    : costmap_(costmap), sum_scores_(false), sideward_inflation_scale_(1.0) {
  if (costmap != NULL) {
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
  }

  ros::NodeHandle pnh("~");
  sideward_inflation_scale_sub_ = pnh.subscribe<std_msgs::Float32>("sideward_inflation_scale", 1, [&](const std_msgs::Float32ConstPtr& msg){
    sideward_inflation_scale_ = msg->data;
  });
}

ObstacleCostFunction::~ObstacleCostFunction() {
  if (world_model_ != NULL) {
    delete world_model_;
  }
}


void ObstacleCostFunction::setParams(double max_trans_vel, double max_forward_inflation, double max_sideward_inflation, double scaling_speed, bool occdist_use_footprint) {
  // TODO: move this to prepare if possible
  max_trans_vel_ = max_trans_vel;
  max_forward_inflation_ = max_forward_inflation;
  max_sideward_inflation_ = max_sideward_inflation;
  scaling_speed_ = scaling_speed;
  occdist_use_footprint_ = occdist_use_footprint;
}

void ObstacleCostFunction::setFootprint(std::vector<geometry_msgs::Point> footprint_spec) {
  footprint_spec_ = footprint_spec;
}

std::vector<geometry_msgs::Point> ObstacleCostFunction::getScaledFootprint(const Trajectory& traj) const {
  std::vector<geometry_msgs::Point> scaled_footprint = footprint_spec_;
  const double scale = getScalingFactor(traj, scaling_speed_, max_trans_vel_);
  if (scale != 0.0) {
    const bool fwd = traj.xv_ > 0;
    const double forward_inflation = scale * max_forward_inflation_;
    const double sideward_inflation = scale * sideward_inflation_scale_ * max_sideward_inflation_;
    for (unsigned int i = 0; i < scaled_footprint.size(); ++i) {
      if (fwd == (scaled_footprint[i].x > 0)) {
        // assumes no sideward motion
        scaled_footprint[i].x += std::copysign(forward_inflation, scaled_footprint[i].x);
        // assumes the widest part of the robot is not behind the footprint's origin
        scaled_footprint[i].y += std::copysign(sideward_inflation, scaled_footprint[i].y);
      }
    }
  }
  return scaled_footprint;
}

mbf_msgs::ExePathResult::_outcome_type ObstacleCostFunction::prepare() {
  return mbf_msgs::ExePathResult::SUCCESS;
}

double ObstacleCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0;
  double px, py, pth;
  if (footprint_spec_.size() == 0) {
    // Bug, should never happen
    ROS_ERROR("Footprint spec is empty, maybe missing call to setFootprint?");
    return -9;
  }

  std::vector<geometry_msgs::Point> scaled_footprint = getScaledFootprint(traj);

   const unsigned int point_size = traj.getPointsSize();
  // ignore first trajectory point since it is the same for all trajectories,
  // unless trajectory only contains one point
  for (unsigned int i = point_size > 1 ? 1 : 0; i < point_size; ++i) {
    traj.getPoint(i, px, py, pth);
    double f_cost = footprintCost(px, py, pth,
        scaled_footprint,
        costmap_, world_model_);

    if(f_cost < 0){
        return f_cost;
    }

    if(sum_scores_)
        cost +=  f_cost;
    else
        cost = f_cost;
  }
  return cost;
}

double ObstacleCostFunction::getScalingFactor(const Trajectory &traj, double scaling_speed, double max_trans_vel) {
  double vmag = hypot(traj.xv_, traj.yv_);

  //if we're over a certain speed threshold, we'll scale the robot's
  //footprint to make it either slow down or stay further from walls
  //scale up to 1 linearly... this could be changed later
  const double scale = vmag <= scaling_speed ? 0.0 : (vmag - scaling_speed) / (max_trans_vel - scaling_speed);
  return std::max(0.0, std::min(1.0, scale));
}

double ObstacleCostFunction::footprintCost (
    const double& x,
    const double& y,
    const double& th,
    const std::vector<geometry_msgs::Point>& scaled_footprint,
    costmap_2d::Costmap2D* costmap,
    base_local_planner::WorldModel* world_model) {

  //check if the footprint is legal
  // TODO: Cache inscribed radius
  double footprint_cost = world_model->footprintCost(x, y, th, scaled_footprint);

  if (footprint_cost < 0) {
    return -6.0;
  }
  unsigned int cell_x, cell_y;

  //we won't allow trajectories that go off the map... shouldn't happen that often anyways
  if ( ! costmap->worldToMap(x, y, cell_x, cell_y)) {
    return -7.0;
  }

  double occ_cost = double(costmap->getCost(cell_x, cell_y));
  if (occdist_use_footprint_)
  {
    occ_cost = std::max(std::max(0.0, footprint_cost), occ_cost);
  }

  return occ_cost;
}

} /* namespace base_local_planner */
