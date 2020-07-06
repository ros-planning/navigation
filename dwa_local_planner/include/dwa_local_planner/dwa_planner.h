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


#include <dwa_local_planner/DWAPlannerConfig.h>

//for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>

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
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <nav_msgs/Path.h>

namespace dwa_local_planner {
  /** Describes the state of the planner. This may influence cost functions and generator
    * Default:  normal operating conditions
    * Arrive:   the robot is close (with switch dist) of its goal
    * Align:    there is a large orientation error between robot and path
  */
  enum LocalPlannerState {Default, Arrive, Align, NotMoving, None};
  static const char * StateName[] = { "Default", "Arrive", "Align", "NotMoving" };

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
       * @brief Reconfigures the trajectory planner
       */
      void reconfigure(DWAPlannerConfig &cfg);

      /**
       * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
       * @param global_pose The current position of the robot
       * @param global_vel The current velocity of the robot
       * @param goal_pose The velocities to send to the robot base
       * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
       */
      base_local_planner::Trajectory findBestPath(
          const geometry_msgs::PoseStamped& global_pose,
          const geometry_msgs::PoseStamped& global_vel,
          const geometry_msgs::PoseStamped& goal_pose);

      /**
       * @brief  Update the cost functions before planning
       * @param  robot_pose The robot's current pose
       * @param  local_plan The new global plan
       * @param  lookahead distance
       * @param  footprint_spec The robot's footprint
       *
       * The obstacle cost function gets the footprint.
       * The path and goal cost functions get the global_plan
       * The alignment cost functions get a version of the global plan
       *   that is modified based on the global_pose 
       */
      void updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& robot_pose,
                                   const std::vector<geometry_msgs::PoseStamped>& local_plan,
                                   double lookahead,
                                   const std::vector<geometry_msgs::Point>& footprint_spec);

      /**
       * @brief Get the period at which the local planner is expected to run
       * @return The simulation period
       */
      double getSimPeriod() { return sim_period_; }
      double getSimTime() { return sim_time_; }

      /**
       * @brief Compute the components and total cost for a map grid cell
       * @param cx The x coordinate of the cell in the map grid
       * @param cy The y coordinate of the cell in the map grid
       * @param path_cost Will be set to the path distance component of the cost function
       * @param goal_cost Will be set to the goal distance component of the cost function
       * @param occ_cost Will be set to the costmap value of the cell
       * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
       * @return True if the cell is traversible and therefore a legal location for the robot to move to
       */
      bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

      /**
       * @brief Sets the stamp of the last motion to the current time. This is used to determine whether the robot is and should be moving.
       */
      void resetMotionStamp() { stamp_last_motion_ = ros::Time::now(); }

    private:

      /**
       * @brief determine the state of the DWA Planner
       * @param robot_pose
       * @param yaw_error
       * @param plan_distance
       * @param goal_distance
       * @return The resulting state of the planner
       */
      LocalPlannerState determineState(const geometry_msgs::PoseStamped& robot_pose, double yaw_error, double plan_distance, double goal_distance);

      /**
       * @brief Checks if the robot is moving. Although somewhat arbitrary:
       * if it hasn't moved at least 10 cm in the past 10 seconds,
       * it is assumed the robot is not moving.
       * @param robot_pose current pose of the robot
       */
      bool isMoving(const geometry_msgs::PoseStamped& robot_pose);

      double switch_yaw_error_;
      double switch_plan_distance_;
      double switch_goal_distance_;

      base_local_planner::LocalPlannerUtil *planner_util_;

      double stop_time_buffer_; ///< @brief How long before hitting something we're going to enforce that the robot stop
      double path_distance_bias_, goal_distance_bias_, occdist_scale_;
      Eigen::Vector3f vsamples_;

      double sim_period_;///< @brief The number of seconds to use to compute max/min vels for dwa
      double sim_time_;

      boost::mutex configuration_mutex_;
      std::string frame_id_;
      ros::Publisher traj_cloud_pub_;
      bool publish_cost_grid_pc_; ///< @brief Whether or not to build and publish a PointCloud
      bool publish_traj_pc_;

      base_local_planner::MapGridVisualizer map_viz_; ///< @brief The map grid visualizer for outputting the potential field generated by the cost function

      // see constructor body for explanations
      base_local_planner::SimpleTrajectoryGenerator generator_;

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

      base_local_planner::CmdVelCostFunction cmd_vel_costs_; /// <@brief penalizes directions to achieve certain behaviors
      double align_cmd_scale_;
      double default_cmd_scale_;
      double arrive_cmd_scale_;
      double align_cmd_px_, align_cmd_nx_, align_cmd_py_, align_cmd_ny_, align_cmd_pth_, align_cmd_nth_;
      double default_cmd_px_, default_cmd_nx_, default_cmd_py_, default_cmd_ny_, default_cmd_pth_, default_cmd_nth_;
      double arrive_cmd_px_, arrive_cmd_nx_, arrive_cmd_py_, arrive_cmd_ny_, arrive_cmd_pth_, arrive_cmd_nth_;

      base_local_planner::ObstacleCostFunction obstacle_costs_; /// <@brief penalizes trajectories close to obstacles
      double align_obstacle_scale_;
      double default_obstacle_scale_;
      double arrive_obstacle_scale_;

      base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;

      ros::Time stamp_last_motion_; /// <@briefROS time when the robot was last moving
  };
};
#endif
