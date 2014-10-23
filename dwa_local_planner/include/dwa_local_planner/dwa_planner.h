/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_H_

#include <vector>
#include <Eigen/Core>

#include "dwa_local_planner/DWAPlannerConfig.h"
#include "dwa_local_planner/visualization.h"

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/occupancy_velocity_cost_function.h>
#include <base_local_planner/alignment_cost_function.h>
#include <base_local_planner/cmd_vel_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <nav_msgs/Path.h>

namespace dwa_local_planner {

  /** Describes the state of the planner. This may influence cost functions and generator
  * Default:  normal operating conditions
  * Arrive:   the robot is close (with switch dist) of its goal
  * Align:    there is a large orientation error between robot and path */
  enum LocalPlannerState {Default, Arrive, Align, None};
  static const char * StateName[] = { "Default", "Arrive", "Align" };

  /**
   * @class DWAPlanner
   * @brief A class implementing a local planner using the Dynamic Window Approach
   */
  class DWAPlanner {
    public:
      /**
       * @brief  Constructor for the planner
       * @param name The name of the planner 
       * @param costmap_ros A pointer to the costmap instance the planner should use
       * @param global_frame the frame id of the tf frame to use
       */
      DWAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util);

      /**
       * @brief  Destructor for the planner
       */
      ~DWAPlanner() {}

      /**
       * @brief Reconfigures the trajectory planner
       */
      boost::mutex configuration_mutex_;
      void reconfigure(DWAPlannerConfig &cfg);

      /**
       * @brief  Take in a new global plan for the local planner to follow, and adjust local costmaps
       * @param  new_plan The new global plan
       */
      void updatePlanAndLocalCosts(tf::Stamped<tf::Pose> robot_pose, const std::vector<geometry_msgs::PoseStamped>& local_plan, const std::vector<geometry_msgs::Point>& footprint_spec);

      /**
       * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
       * @param robot_pose The current position of the robot
       * @param robot_vel The current velocity of the robot
       * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
       */
      base_local_planner::Trajectory findBestPath(tf::Stamped<tf::Pose> robot_pose, tf::Stamped<tf::Pose> robot_vel, tf::Stamped<tf::Pose> goal_pose);

      inline double getSimPeriod() { return sim_period_; }
      inline double getSimTime() { return sim_time_; }

    private:

      //! Pointer to planner util
      base_local_planner::LocalPlannerUtil *planner_util_;

      //! Switches which determine the state of the DWA Planner
      LocalPlannerState determineState(double yaw_error, double plan_distance, double goal_distance);
      double switch_yaw_error_;
      double switch_plan_distance_;
      double switch_goal_distance_;

      //! Trajectory generation
      base_local_planner::SimpleTrajectoryGenerator generator_;
      Eigen::Vector3f vsamples_;
      double sim_period_, sim_time_;

      //! Cost functions with parameters
      //base_local_planner::ObstacleCostFunction obstacle_costs_; /// <@brief discards trajectories that move into obstacles
      base_local_planner::OccupancyVelocityCostFunction occ_vel_costs_; /// <@brief discards trajectories that on which the velocity is not allowed

      base_local_planner::MapGridCostFunction plan_costs_; /// <@brief prefers trajectories on plan
      double align_plan_scale_;
      double default_plan_scale_;
      double arrive_plan_scale_;

      base_local_planner::MapGridCostFunction goal_costs_; /// <@brief prefers trajectories that go towards (local) goal, based on wave propagation
      double align_goal_scale_;
      double default_goal_scale_;
      double arrive_goal_scale_;

      base_local_planner::AlignmentCostFunction alignment_costs_;  /// <@brief prefers trajectories that align with plan
      double align_align_scale_;
      double default_align_scale_;
      double arrive_align_scale_;

      //! Scored sampling planner which evaluates the trajectories generation by the SimpleTrajectoryGenerator with use of costfunctions
      base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;

      //! Visualization
      Visualization vis_;
  };
}
#endif
