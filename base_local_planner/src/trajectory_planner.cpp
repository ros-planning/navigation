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

#include <base_local_planner/trajectory_planner.h>

using namespace std;
using namespace costmap_2d;

namespace base_local_planner{
  TrajectoryPlanner::TrajectoryPlanner(WorldModel& world_model, 
      const Costmap2D& costmap, 
      std::vector<geometry_msgs::Point> footprint_spec,
      double inscribed_radius, double circumscribed_radius,
      double acc_lim_x, double acc_lim_y, double acc_lim_theta,
      double sim_time, double sim_granularity, 
      int vx_samples, int vtheta_samples,
      double pdist_scale, double gdist_scale, double occdist_scale,
      double heading_lookahead, double oscillation_reset_dist,
      double escape_reset_dist, double escape_reset_theta,
      bool holonomic_robot,
      double max_vel_x, double min_vel_x,
      double max_vel_th, double min_vel_th, double min_in_place_vel_th,
      double backup_vel,
      bool dwa, bool heading_scoring, double heading_scoring_timestep, bool simple_attractor,
      vector<double> y_vels)
    : map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()), costmap_(costmap), 
    world_model_(world_model), footprint_spec_(footprint_spec),
    inscribed_radius_(inscribed_radius), circumscribed_radius_(circumscribed_radius),
    goal_x_(0), goal_y_(0),
    sim_time_(sim_time), sim_granularity_(sim_granularity), 
    vx_samples_(vx_samples), vtheta_samples_(vtheta_samples),
    pdist_scale_(pdist_scale), gdist_scale_(gdist_scale), occdist_scale_(occdist_scale),
    acc_lim_x_(acc_lim_x), acc_lim_y_(acc_lim_y), acc_lim_theta_(acc_lim_theta),
    prev_x_(0), prev_y_(0), escape_x_(0), escape_y_(0), escape_theta_(0), heading_lookahead_(heading_lookahead), 
    oscillation_reset_dist_(oscillation_reset_dist), escape_reset_dist_(escape_reset_dist), 
    escape_reset_theta_(escape_reset_theta), holonomic_robot_(holonomic_robot),
    max_vel_x_(max_vel_x), min_vel_x_(min_vel_x), 
    max_vel_th_(max_vel_th), min_vel_th_(min_vel_th), min_in_place_vel_th_(min_in_place_vel_th),
    backup_vel_(backup_vel),
    dwa_(dwa), heading_scoring_(heading_scoring), heading_scoring_timestep_(heading_scoring_timestep),
    simple_attractor_(simple_attractor), y_vels_(y_vels)
  {
    //the robot is not stuck to begin with
    stuck_left = false;
    stuck_right = false;
    stuck_left_strafe = false;
    stuck_right_strafe = false;
    rotating_left = false;
    rotating_right = false;
    strafe_left = false;
    strafe_right = false;

    escaping_ = false;

    //if no y velocities are specified... we'll fill in default values
    if(y_vels_.empty()){
      y_vels_.push_back(-0.3);
      y_vels_.push_back(-0.1);
      y_vels_.push_back(0.1);
      y_vels_.push_back(0.3);
    }
  }

  TrajectoryPlanner::~TrajectoryPlanner(){}

  //update what map cells are considered path based on the global_plan
  void TrajectoryPlanner::setPathCells(){
    int local_goal_x = -1;
    int local_goal_y = -1;
    bool started_path = false;
    queue<MapCell*> path_dist_queue;
    queue<MapCell*> goal_dist_queue;
    for(unsigned int i = 0; i < global_plan_.size(); ++i){
      double g_x = global_plan_[i].pose.position.x;
      double g_y = global_plan_[i].pose.position.y;
      unsigned int map_x, map_y;
      if(costmap_.worldToMap(g_x, g_y, map_x, map_y) && costmap_.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION){
        MapCell& current = map_(map_x, map_y);
        current.path_dist = 0.0;
        current.path_mark = true;
        path_dist_queue.push(&current);
        local_goal_x = map_x;
        local_goal_y = map_y;
        started_path = true;
      }
      else{
        if(started_path)
          break;
      }
    }

    if(local_goal_x >= 0 && local_goal_y >= 0){
      MapCell& current = map_(local_goal_x, local_goal_y);
      costmap_.mapToWorld(local_goal_x, local_goal_y, goal_x_, goal_y_);
      current.goal_dist = 0.0;
      current.goal_mark = true;
      goal_dist_queue.push(&current);
    }
    //compute our distances
    computePathDistance(path_dist_queue);
    computeGoalDistance(goal_dist_queue);
  }

  void TrajectoryPlanner::computePathDistance(queue<MapCell*>& dist_queue){
    MapCell* current_cell;
    MapCell* check_cell;
    unsigned int last_col = map_.size_x_ - 1;
    unsigned int last_row = map_.size_y_ - 1;
    while(!dist_queue.empty()){
      current_cell = dist_queue.front();
      check_cell = current_cell;
      dist_queue.pop();

      if(current_cell->cx > 0){
        check_cell = current_cell - 1;
        if(!check_cell->path_mark){
          updatePathCell(current_cell, check_cell, dist_queue);
        }
      }

      if(current_cell->cx < last_col){
        check_cell = current_cell + 1;
        if(!check_cell->path_mark){
          updatePathCell(current_cell, check_cell, dist_queue);
        }
      }

      if(current_cell->cy > 0){
        check_cell = current_cell - map_.size_x_;
        if(!check_cell->path_mark){
          updatePathCell(current_cell, check_cell, dist_queue);
        }
      }

      if(current_cell->cy < last_row){
        check_cell = current_cell + map_.size_x_;
        if(!check_cell->path_mark){
          updatePathCell(current_cell, check_cell, dist_queue);
        }
      }
    }
  }

  void TrajectoryPlanner::computeGoalDistance(queue<MapCell*>& dist_queue){
    MapCell* current_cell;
    MapCell* check_cell;
    unsigned int last_col = map_.size_x_ - 1;
    unsigned int last_row = map_.size_y_ - 1;
    while(!dist_queue.empty()){
      current_cell = dist_queue.front();
      current_cell->goal_mark = true;
      check_cell = current_cell;
      dist_queue.pop();

      if(current_cell->cx > 0){
        check_cell = current_cell - 1;
        if(!check_cell->goal_mark){
          updateGoalCell(current_cell, check_cell, dist_queue);
        }
      }

      if(current_cell->cx < last_col){
        check_cell = current_cell + 1;
        if(!check_cell->goal_mark){
          updateGoalCell(current_cell, check_cell, dist_queue);
        }
      }

      if(current_cell->cy > 0){
        check_cell = current_cell - map_.size_x_;
        if(!check_cell->goal_mark){
          updateGoalCell(current_cell, check_cell, dist_queue);
        }
      }

      if(current_cell->cy < last_row){
        check_cell = current_cell + map_.size_x_;
        if(!check_cell->goal_mark){
          updateGoalCell(current_cell, check_cell, dist_queue);
        }
      }
    }
  }

  //create and score a trajectory given the current pose of the robot and selected velocities
  void TrajectoryPlanner::generateTrajectory(double x, double y, double theta, double vx, double vy, 
      double vtheta, double vx_samp, double vy_samp, double vtheta_samp, 
      double acc_x, double acc_y, double acc_theta, double impossible_cost,
      Trajectory& traj){
    double x_i = x;
    double y_i = y;
    double theta_i = theta;

    double vx_i, vy_i, vtheta_i;

    vx_i = vx;
    vy_i = vy;
    vtheta_i = vtheta;

    //compute the magnitude of the velocities
    double vmag = sqrt(vx_samp * vx_samp + vy_samp * vy_samp);

    //compute the number of steps we must take along this trajectory to be "safe"
    int num_steps;
    if(!heading_scoring_)
      num_steps = int(max((vmag * sim_time_) / sim_granularity_, abs(vtheta_samp) / sim_granularity_) + 0.5);
    else
      num_steps = int(sim_time_ / sim_granularity_ + 0.5);

    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //create a potential trajectory
    traj.resetPoints();
    traj.xv_ = vx_samp; 
    traj.yv_ = vy_samp; 
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    if(num_steps == 0)
      return;

    //initialize the costs for the trajectory
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;
    double heading_diff = 0.0;

    for(int i = 0; i < num_steps; ++i){
      //get map coordinates of a point
      unsigned int cell_x, cell_y;

      //we don't want a path that goes off the know map
      if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
        traj.cost_ = -1.0;
        return;
      }

      //check the point on the trajectory for legality
      double footprint_cost = footprintCost(x_i, y_i, theta_i);

      //if the footprint hits an obstacle this trajectory is invalid
      if(footprint_cost < 0){
        traj.cost_ = -1.0;
        return;
      }

      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));

      double cell_pdist = map_(cell_x, cell_y).path_dist;
      double cell_gdist = map_(cell_x, cell_y).goal_dist;

      //update path and goal distances
      if(!heading_scoring_){
        path_dist = cell_pdist;
        goal_dist = cell_gdist;
      }
      else if(time >= heading_scoring_timestep_ && time < heading_scoring_timestep_ + dt){
        heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
        //update path and goal distances
        path_dist = cell_pdist;
        goal_dist = cell_gdist;
      }

      time += dt;

      //do we want to follow blindly
      if(simple_attractor_){
        goal_dist = (x_i - global_plan_[global_plan_.size() -1].pose.position.x) * 
          (x_i - global_plan_[global_plan_.size() -1].pose.position.x) + 
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y) * 
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y);
        path_dist = 0.0;
      }
      else{
        //if a point on this trajectory has no clear path to goal it is invalid
        if(impossible_cost <= goal_dist || impossible_cost <= path_dist){
          ROS_DEBUG("No path to goal with goal distance = %f, path_distance = %f and max cost = %f", 
              goal_dist, path_dist, impossible_cost);
          traj.cost_ = -2.0;
          return;
        }
      }


      //the point is legal... add it to the trajectory
      traj.addPoint(x_i, y_i, theta_i);

      //calculate velocities
      vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
      vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
      vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

      //calculate positions
      x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
      y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
      theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

    }

    //ROS_INFO("OccCost: %f, vx: %.2f, vy: %.2f, vtheta: %.2f", occ_cost, vx_samp, vy_samp, vtheta_samp);
    double cost = -1.0;
    if(!heading_scoring_)
      cost = pdist_scale_ * path_dist + goal_dist * gdist_scale_ + occdist_scale_ * occ_cost;
    else
      cost = occdist_scale_ * occ_cost + pdist_scale_ * path_dist + 0.3 * heading_diff + goal_dist * gdist_scale_;

    traj.cost_ = cost;
  }

  double TrajectoryPlanner::headingDiff(int cell_x, int cell_y, double x, double y, double heading){
    double heading_diff = DBL_MAX;
    unsigned int goal_cell_x, goal_cell_y;
    //find a clear line of sight from the robot's cell to a point on the path
    for(int i = global_plan_.size() - 1; i >=0; --i){
      if(costmap_.worldToMap(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y, goal_cell_x, goal_cell_y)){
        if(lineCost(cell_x, goal_cell_x, cell_y, goal_cell_y) >= 0){
          double gx, gy;
          costmap_.mapToWorld(goal_cell_x, goal_cell_y, gx, gy);
          double v1_x = gx - x;
          double v1_y = gy - y;
          double v2_x = cos(heading);
          double v2_y = sin(heading);

          double perp_dot = v1_x * v2_y - v1_y * v2_x;
          double dot = v1_x * v2_x + v1_y * v2_y;

          //get the signed angle
          double vector_angle = atan2(perp_dot, dot);

          heading_diff = abs(vector_angle);
          return heading_diff;

        }
      }
    }
    return heading_diff;
  }

  //calculate the cost of a ray-traced line
  double TrajectoryPlanner::lineCost(int x0, int x1, 
      int y0, int y1){
    //Bresenham Ray-Tracing
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;

    double line_cost = 0.0;
    double point_cost = -1.0;

    if (x1 >= x0)                 // The x-values are increasing
    {
      xinc1 = 1;
      xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1 = -1;
      xinc2 = -1;
    }

    if (y1 >= y0)                 // The y-values are increasing
    {
      yinc1 = 1;
      yinc2 = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1 = -1;
      yinc2 = -1;
    }

    if (deltax >= deltay)         // There is at least one x-value for every y-value
    {
      xinc1 = 0;                  // Don't change the x when numerator >= denominator
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    }
    else                          // There is at least one y-value for every x-value
    {
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }

    for (int curpixel = 0; curpixel <= numpixels; curpixel++)
    {
      point_cost = pointCost(x, y); //Score the current point

      if(point_cost < 0)
        return -1;

      if(line_cost < point_cost)
        line_cost = point_cost;

      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den)             // Check if numerator >= denominator
      {
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }
      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }

    return line_cost;
  }

  double TrajectoryPlanner::pointCost(int x, int y){
    unsigned char cost = costmap_.getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION){
      return -1;
    }

    return cost;
  }

  void TrajectoryPlanner::updatePlan(const vector<geometry_msgs::PoseStamped>& new_plan){
    global_plan_.resize(new_plan.size());
    for(unsigned int i = 0; i < new_plan.size(); ++i){
      global_plan_[i] = new_plan[i];
    }
  }

  bool TrajectoryPlanner::checkTrajectory(double x, double y, double theta, double vx, double vy, 
      double vtheta, double vx_samp, double vy_samp, double vtheta_samp){
    Trajectory t; 
    double impossible_cost = map_.map_.size();
    generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
        acc_lim_x_, acc_lim_y_, acc_lim_theta_, impossible_cost, t);

    //if the trajectory is a legal one... the check passes
    if(t.cost_ >= 0)
      return true;

    //otherwise the check fails
    return false;
  }

  //create the trajectories we wish to score
  Trajectory TrajectoryPlanner::createTrajectories(double x, double y, double theta, 
      double vx, double vy, double vtheta,
      double acc_x, double acc_y, double acc_theta){
    //compute feasible velocity limits in robot space
    double max_vel_x, max_vel_theta;
    double min_vel_x, min_vel_theta;

    //should we use the dynamic window approach?
    if(dwa_){
      max_vel_x = min(max_vel_x_, vx + acc_x * .1);
      min_vel_x = max(min_vel_x_, vx - acc_x * .1);

      max_vel_theta = min(max_vel_th_, vtheta + acc_theta * .1);
      min_vel_theta = max(min_vel_th_, vtheta - acc_theta * .1);
    }
    else{
      max_vel_x = min(max_vel_x_, vx + acc_x * sim_time_);
      min_vel_x = max(min_vel_x_, vx - acc_x * sim_time_);

      max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_time_);
      min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_time_);
    }


    //we want to sample the velocity space regularly
    double dvx = (max_vel_x - min_vel_x) / (vx_samples_ - 1);
    double dvtheta = (max_vel_theta - min_vel_theta) / (vtheta_samples_ - 1);

    double vx_samp = min_vel_x;
    double vtheta_samp = min_vel_theta;
    double vy_samp = 0.0;

    //keep track of the best trajectory seen so far
    Trajectory* best_traj = &traj_one;
    best_traj->cost_ = -1.0;

    Trajectory* comp_traj = &traj_two;
    comp_traj->cost_ = -1.0;

    Trajectory* swap = NULL;

    //any cell with a cost greater than the size of the map is impossible
    double impossible_cost = map_.map_.size();

    //if we're performing an escape we won't allow moving forward
    if(!escaping_){
      //loop through all x velocities
      for(int i = 0; i < vx_samples_; ++i){
        vtheta_samp = 0;
        //first sample the straight trajectory
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
            acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }

        vtheta_samp = min_vel_theta;
        //next sample all theta trajectories
        for(int j = 0; j < vtheta_samples_ - 1; ++j){
          generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
              acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

          //if the new trajectory is better... let's take it
          if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
            swap = best_traj;
            best_traj = comp_traj;
            comp_traj = swap;
          }
          vtheta_samp += dvtheta;
        }
        vx_samp += dvx;
      }

      //only explore y velocities with holonomic robots
      if(holonomic_robot_){
        //explore trajectories that move forward but also strafe slightly
        vx_samp = 0.1;
        vy_samp = 0.1;
        vtheta_samp = 0.0;
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
            acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }

        vx_samp = 0.1;
        vy_samp = -0.1;
        vtheta_samp = 0.0;
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
            acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }
      }
    }

    //next we want to generate trajectories for rotating in place
    vtheta_samp = min_vel_theta;
    vx_samp = 0.0;
    vy_samp = 0.0;

    //let's try to rotate toward open space
    double heading_dist = DBL_MAX;

    for(int i = 0; i < vtheta_samples_; ++i){
      //enforce a minimum rotational velocity because the base can't handle small in-place rotations
      double vtheta_samp_limited = vtheta_samp > 0 ? max(vtheta_samp, min_in_place_vel_th_) 
        : min(vtheta_samp, -1.0 * min_in_place_vel_th_);

      generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp_limited, 
          acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

      //if the new trajectory is better... let's take it... 
      //note if we can legally rotate in place we prefer to do that rather than move with y velocity
      if(comp_traj->cost_ >= 0 
          && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0 || best_traj->yv_ != 0.0) 
          && (vtheta_samp > dvtheta || vtheta_samp < -1 * dvtheta)){
        double x_r, y_r, th_r;
        comp_traj->getEndpoint(x_r, y_r, th_r);
        x_r += heading_lookahead_ * cos(th_r);
        y_r += heading_lookahead_ * sin(th_r);
        unsigned int cell_x, cell_y;

        //make sure that we'll be looking at a legal cell
        if(costmap_.worldToMap(x_r, y_r, cell_x, cell_y)){
          double ahead_gdist = map_(cell_x, cell_y).goal_dist;
          if(ahead_gdist < heading_dist){
            //if we haven't already tried rotating left since we've moved forward
            if(vtheta_samp < 0 && !stuck_left){
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
            //if we haven't already tried rotating right since we've moved forward
            else if(vtheta_samp > 0 && !stuck_right){
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
          }
        }
      }

      vtheta_samp += dvtheta;
    }

    //do we have a legal trajectory
    if(best_traj->cost_ >= 0){
      if(!(best_traj->xv_ > 0)){
        if(best_traj->thetav_ < 0){
          if(rotating_right){
            stuck_right = true;
          }
          rotating_left = true;
        }
        else if(best_traj->thetav_ > 0){
          if(rotating_left){
            stuck_left = true;
          }
          rotating_right = true;
        }
        else if(best_traj->yv_ > 0){
          if(strafe_right){
            stuck_right_strafe = true;
          }
          strafe_left = true;
        }
        else if(best_traj->yv_ < 0){
          if(strafe_left){
            stuck_left_strafe = true;
          }
          strafe_right = true;
        }

        //set the position we must move a certain distance away from
        prev_x_ = x;
        prev_y_ = y;
      }

      double dist = sqrt((x - prev_x_) * (x - prev_x_) + (y - prev_y_) * (y - prev_y_));
      if(dist > oscillation_reset_dist_){
        rotating_left = false;
        rotating_right = false;
        strafe_left = false;
        strafe_right = false;
        stuck_left = false;
        stuck_right = false;
        stuck_left_strafe = false;
        stuck_right_strafe = false;
      }

      dist = sqrt((x - escape_x_) * (x - escape_x_) + (y - escape_y_) * (y - escape_y_));
      if(dist > escape_reset_dist_ || fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_){
        escaping_ = false;
      }

      return *best_traj;
    }



    //only explore y velocities with holonomic robots
    if(holonomic_robot_){
      //if we can't rotate in place or move forward... maybe we can move sideways and rotate
      vtheta_samp = min_vel_theta;
      vx_samp = 0.0;

      //loop through all y velocities
      for(unsigned int i = 0; i < y_vels_.size(); ++i){
        vtheta_samp = 0;
        vy_samp = y_vels_[i];
        //sample completely horizontal trajectories
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
            acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0)){
          double x_r, y_r, th_r;
          comp_traj->getEndpoint(x_r, y_r, th_r);
          x_r += heading_lookahead_ * cos(th_r);
          y_r += heading_lookahead_ * sin(th_r);
          unsigned int cell_x, cell_y;

          //make sure that we'll be looking at a legal cell
          if(costmap_.worldToMap(x_r, y_r, cell_x, cell_y)){
            double ahead_gdist = map_(cell_x, cell_y).goal_dist;
            if(ahead_gdist < heading_dist){
              //if we haven't already tried strafing left since we've moved forward
              if(vy_samp > 0 && !stuck_left_strafe){
                swap = best_traj;
                best_traj = comp_traj;
                comp_traj = swap;
                heading_dist = ahead_gdist;
              }
              //if we haven't already tried rotating right since we've moved forward
              else if(vy_samp < 0 && !stuck_right_strafe){
                swap = best_traj;
                best_traj = comp_traj;
                comp_traj = swap;
                heading_dist = ahead_gdist;
              }
            }
          }
        }
      }
    }

    //do we have a legal trajectory
    if(best_traj->cost_ >= 0){
      if(!(best_traj->xv_ > 0)){
        if(best_traj->thetav_ < 0){
          if(rotating_right){
            stuck_right = true;
          }
          rotating_left = true;
        }
        else if(best_traj->thetav_ > 0){
          if(rotating_left){
            stuck_left = true;
          }
          rotating_right = true;
        }
        else if(best_traj->yv_ > 0){
          if(strafe_right){
            stuck_right_strafe = true;
          }
          strafe_left = true;
        }
        else if(best_traj->yv_ < 0){
          if(strafe_left){
            stuck_left_strafe = true;
          }
          strafe_right = true;
        }

        //set the position we must move a certain distance away from
        prev_x_ = x;
        prev_y_ = y;

      }

      double dist = sqrt((x - prev_x_) * (x - prev_x_) + (y - prev_y_) * (y - prev_y_));
      if(dist > oscillation_reset_dist_){
        rotating_left = false;
        rotating_right = false;
        strafe_left = false;
        strafe_right = false;
        stuck_left = false;
        stuck_right = false;
        stuck_left_strafe = false;
        stuck_right_strafe = false;
      }

      dist = sqrt((x - escape_x_) * (x - escape_x_) + (y - escape_y_) * (y - escape_y_));
      if(dist > escape_reset_dist_ || fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_){
        escaping_ = false;
      }

      return *best_traj;
    }

    //and finally, if we can't do anything else, we want to generate trajectories that move backwards slowly
    vtheta_samp = 0.0;
    vx_samp = backup_vel_;
    vy_samp = 0.0;
    generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
        acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

    //if the new trajectory is better... let's take it
    /*
       if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
       swap = best_traj;
       best_traj = comp_traj;
       comp_traj = swap;
       }
       */

    //we'll allow moving backwards slowly even when the static map shows it as blocked
    swap = best_traj;
    best_traj = comp_traj;
    comp_traj = swap;
    
    double dist = sqrt((x - prev_x_) * (x - prev_x_) + (y - prev_y_) * (y - prev_y_));
    if(dist > oscillation_reset_dist_){
      rotating_left = false;
      rotating_right = false;
      strafe_left = false;
      strafe_right = false;
      stuck_left = false;
      stuck_right = false;
      stuck_left_strafe = false;
      stuck_right_strafe = false;
    }

    //only enter escape mode when the planner has given a valid goal point
    if(!escaping_ && best_traj->cost_ > -2.0){
      escape_x_ = x;
      escape_y_ = y;
      escape_theta_ = theta;
      escaping_ = true;
    }

    dist = sqrt((x - escape_x_) * (x - escape_x_) + (y - escape_y_) * (y - escape_y_));

    if(dist > escape_reset_dist_ || fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_){
      escaping_ = false;
    }


    //if the trajectory failed because the footprint hits something, we're still going to back up
    if(best_traj->cost_ == -1.0)
      best_traj->cost_ = 1.0;

    return *best_traj;

  }

  //given the current state of the robot, find a good trajectory
  Trajectory TrajectoryPlanner::findBestPath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel, 
      tf::Stamped<tf::Pose>& drive_velocities){

    double uselessPitch, uselessRoll, yaw, velYaw;
    global_pose.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
    global_vel.getBasis().getEulerZYX(velYaw, uselessPitch, uselessRoll);

    double x = global_pose.getOrigin().getX();
    double y = global_pose.getOrigin().getY();
    double theta = yaw;

    double vx = global_vel.getOrigin().getX();
    double vy = global_vel.getOrigin().getY();
    double vtheta = velYaw;

    //reset the map for new operations
    map_.resetPathDist();

    //temporarily remove obstacles that are within the footprint of the robot
    vector<base_local_planner::Position2DInt> footprint_list = getFootprintCells(x, y, theta, true);

    //mark cells within the initial footprint of the robot
    for(unsigned int i = 0; i < footprint_list.size(); ++i){
      map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
    }

    //make sure that we update our path based on the global plan and compute costs
    setPathCells();
    ROS_DEBUG("Path/Goal distance computed");

    //rollout trajectories and find the minimum cost one
    Trajectory best = createTrajectories(x, y, theta, 
        vx, vy, vtheta, 
        acc_lim_x_, acc_lim_y_, acc_lim_theta_);
    ROS_DEBUG("Trajectories created");

    /*
    //If we want to print a ppm file to draw goal dist
    char buf[4096];
    sprintf(buf, "base_local_planner.ppm");
    FILE *fp = fopen(buf, "w");
    if(fp){
      fprintf(fp, "P3\n");
      fprintf(fp, "%d %d\n", map_.size_x_, map_.size_y_);
      fprintf(fp, "255\n");
      for(int j = map_.size_y_ - 1; j >= 0; --j){
        for(unsigned int i = 0; i < map_.size_x_; ++i){
          int g_dist = 255 - int(map_(i, j).goal_dist);
          int p_dist = 255 - int(map_(i, j).path_dist);
          if(g_dist < 0)
            g_dist = 0;
          if(p_dist < 0)
            p_dist = 0;
          fprintf(fp, "%d 0 %d ", g_dist, 0);
        }
        fprintf(fp, "\n");
      }
      fclose(fp);
    }
    */

    if(best.cost_ < 0){
      drive_velocities.setIdentity();
    }
    else{
      btVector3 start(best.xv_, best.yv_, 0);
      drive_velocities.setOrigin(start);
      btMatrix3x3 matrix;
      matrix.setRotation(btQuaternion(best.thetav_, 0, 0));
      drive_velocities.setBasis(matrix);
    }

    return best;
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double TrajectoryPlanner::footprintCost(double x_i, double y_i, double theta_i){
    //if we have no footprint... do nothing
    if(footprint_spec_.size() < 3)
      return -1.0;

    //build the oriented footprint
    double cos_th = cos(theta_i);
    double sin_th = sin(theta_i);
    vector<geometry_msgs::Point> oriented_footprint;
    for(unsigned int i = 0; i < footprint_spec_.size(); ++i){
      geometry_msgs::Point new_pt;
      new_pt.x = x_i + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
      new_pt.y = y_i + (footprint_spec_[i].x * sin_th + footprint_spec_[i].y * cos_th);
      oriented_footprint.push_back(new_pt);
    }

    geometry_msgs::Point robot_position;
    robot_position.x = x_i;
    robot_position.y = y_i;

    //check if the footprint is legal
    double footprint_cost = world_model_.footprintCost(robot_position, oriented_footprint, inscribed_radius_, circumscribed_radius_);

    return footprint_cost;
  }

  void TrajectoryPlanner::getLineCells(int x0, int x1, int y0, int y1, vector<base_local_planner::Position2DInt>& pts){
    //Bresenham Ray-Tracing
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;

    base_local_planner::Position2DInt pt;

    if (x1 >= x0)                 // The x-values are increasing
    {
      xinc1 = 1;
      xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1 = -1;
      xinc2 = -1;
    }

    if (y1 >= y0)                 // The y-values are increasing
    {
      yinc1 = 1;
      yinc2 = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1 = -1;
      yinc2 = -1;
    }

    if (deltax >= deltay)         // There is at least one x-value for every y-value
    {
      xinc1 = 0;                  // Don't change the x when numerator >= denominator
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    }
    else                          // There is at least one y-value for every x-value
    {
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }

    for (int curpixel = 0; curpixel <= numpixels; curpixel++)
    {
      pt.x = x;      //Draw the current pixel
      pt.y = y;
      pts.push_back(pt);

      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den)             // Check if numerator >= denominator
      {
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }
      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }
  }

  //its nice to be able to draw a footprint for a particular point for debugging info
  vector<geometry_msgs::Point> TrajectoryPlanner::drawFootprint(double x_i, double y_i, double theta_i){
    vector<base_local_planner::Position2DInt> footprint_cells = getFootprintCells(x_i, y_i, theta_i, false);
    vector<geometry_msgs::Point> footprint_pts;
    geometry_msgs::Point pt;
    for(unsigned int i = 0; i < footprint_cells.size(); ++i){
      double pt_x, pt_y;
      costmap_.mapToWorld(footprint_cells[i].x, footprint_cells[i].y, pt_x, pt_y);
      pt.x = pt_x;
      pt.y = pt_y;
      footprint_pts.push_back(pt);
    }
    return footprint_pts;
  }

  //get the cellsof a footprint at a given position
  vector<base_local_planner::Position2DInt> TrajectoryPlanner::getFootprintCells(double x_i, double y_i, double theta_i, bool fill){
    vector<base_local_planner::Position2DInt> footprint_cells;

    //if we have no footprint... do nothing
    if(footprint_spec_.size() <= 1)
      return footprint_cells;

    //pre-compute cos and sin values
    double cos_th = cos(theta_i);
    double sin_th = sin(theta_i);
    double new_x, new_y;
    unsigned int x0, y0, x1, y1;
    unsigned int last_index = footprint_spec_.size() - 1;

    for(unsigned int i = 0; i < last_index; ++i){
      //find the cell coordinates of the first segment point
      new_x = x_i + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
      new_y = y_i + (footprint_spec_[i].x * sin_th + footprint_spec_[i].y * cos_th);
      if(!costmap_.worldToMap(new_x, new_y, x0, y0))
        return footprint_cells;

      //find the cell coordinates of the second segment point
      new_x = x_i + (footprint_spec_[i + 1].x * cos_th - footprint_spec_[i + 1].y * sin_th);
      new_y = y_i + (footprint_spec_[i + 1].x * sin_th + footprint_spec_[i + 1].y * cos_th);
      if(!costmap_.worldToMap(new_x, new_y, x1, y1))
        return footprint_cells;

      getLineCells(x0, x1, y0, y1, footprint_cells);
    }

    //we need to close the loop, so we also have to raytrace from the last pt to first pt
    new_x = x_i + (footprint_spec_[last_index].x * cos_th - footprint_spec_[last_index].y * sin_th);
    new_y = y_i + (footprint_spec_[last_index].x * sin_th + footprint_spec_[last_index].y * cos_th);
    if(!costmap_.worldToMap(new_x, new_y, x0, y0))
      return footprint_cells;

    new_x = x_i + (footprint_spec_[0].x * cos_th - footprint_spec_[0].y * sin_th);
    new_y = y_i + (footprint_spec_[0].x * sin_th + footprint_spec_[0].y * cos_th);
    if(!costmap_.worldToMap(new_x, new_y, x1, y1))
      return footprint_cells;

    getLineCells(x0, x1, y0, y1, footprint_cells);

    if(fill)
      getFillCells(footprint_cells);

    return footprint_cells;
  }

  void TrajectoryPlanner::getFillCells(vector<base_local_planner::Position2DInt>& footprint){
    //quick bubble sort to sort pts by x
    base_local_planner::Position2DInt swap, pt;
    unsigned int i = 0;
    while(i < footprint.size() - 1){
      if(footprint[i].x > footprint[i + 1].x){
        swap = footprint[i];
        footprint[i] = footprint[i + 1];
        footprint[i + 1] = swap;
        if(i > 0)
          --i;
      }
      else
        ++i;
    }

    i = 0;
    base_local_planner::Position2DInt min_pt;
    base_local_planner::Position2DInt max_pt;
    unsigned int min_x = footprint[0].x;
    unsigned int max_x = footprint[footprint.size() -1].x;
    //walk through each column and mark cells inside the footprint
    for(unsigned int x = min_x; x <= max_x; ++x){
      if(i >= footprint.size() - 1)
        break;

      if(footprint[i].y < footprint[i + 1].y){
        min_pt = footprint[i];
        max_pt = footprint[i + 1];
      }
      else{
        min_pt = footprint[i + 1];
        max_pt = footprint[i];
      }

      i += 2;
      while(i < footprint.size() && footprint[i].x == x){
        if(footprint[i].y < min_pt.y)
          min_pt = footprint[i];
        else if(footprint[i].y > max_pt.y)
          max_pt = footprint[i];
        ++i;
      }

      //loop though cells in the column
      for(unsigned int y = min_pt.y; y < max_pt.y; ++y){
        pt.x = x;
        pt.y = y;
        footprint.push_back(pt);
      }
    }
  }

  void TrajectoryPlanner::getLocalGoal(double& x, double& y){
    x = goal_x_;
    y = goal_y_;
  }

};


