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
#include <pcl_ros/publisher.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/alignment_cost_function.h>
#include <base_local_planner/cmd_vel_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <nav_msgs/Path.h>

namespace dwa_local_planner {

  /** Describes the state of the planner. This may influence cost functions and generator
  * Default:  normal operating conditions
  * Arrive:   the robot is close (with switch dist) of its goal
  * Align:    there is a large orientation error between robot and path */
  enum LocalPlannerState {Default, Arrive, Align};
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
      void reconfigure(DWAPlannerConfig &cfg);

      /**
       * @brief  Take in a new global plan for the local planner to follow, and adjust local costmaps
       * @param  new_plan The new global plan
       */
      void updatePlanAndLocalCosts(tf::Stamped<tf::Pose> robot_pose, const std::vector<geometry_msgs::PoseStamped>& new_plan, const std::vector<geometry_msgs::Point>& footprint_spec);

      /**
       * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
       * @param robot_pose The current position of the robot
       * @param robot_vel The current velocity of the robot
       * @param drive_velocities The velocities to send to the robot base
       * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
       */
      base_local_planner::Trajectory findBestPath(
          tf::Stamped<tf::Pose> robot_pose,
          tf::Stamped<tf::Pose> robot_vel,
          tf::Stamped<tf::Pose>& drive_velocities);

      /**
       * @brief Get the period at which the local planner is expected to run
       * @return The simulation period
       */
      double getSimPeriod() { return sim_period_; }

    private:

      base_local_planner::LocalPlannerUtil *planner_util_;

      double stop_time_buffer_; ///< @brief How long before hitting something we're going to enforce that the robot stop
      double pdist_scale_, gdist_scale_, occdist_scale_;
      Eigen::Vector3f vsamples_;

      double sim_period_;///< @brief The number of seconds to use to compute max/min vels for dwa
      base_local_planner::Trajectory result_traj_;

      //double forward_point_distance_;
      /** Distance along the global plan where the local goal is place */
      double lookahead_dist_;

      /** If the distance along the global path is smaller than the switch distance, different parameters are selected */
      double switch_dist_;

      std::vector<geometry_msgs::PoseStamped> plan_;

      boost::mutex configuration_mutex_;

      pcl_ros::Publisher<base_local_planner::MapGridCostPoint> traj_cloud_pub_;
      void publishTrajectoryCloud(const std::vector<base_local_planner::Trajectory>& trajectories);
      bool publish_cost_grid_pc_; ///< @brief Whether or not to build and publish a PointCloud

      double cheat_factor_;

      LocalPlannerState determineState(double angle_error, double path_distance, double goal_distance);

      // see constructor body for explanations
      base_local_planner::SimpleTrajectoryGenerator generator_;
//      base_local_planner::OscillationCostFunction oscillation_costs_;
      base_local_planner::ObstacleCostFunction obstacle_costs_;
      base_local_planner::MapGridCostFunction path_costs_;
      base_local_planner::MapGridCostFunction goal_costs_;
//      base_local_planner::MapGridCostFunction goal_front_costs_;
//      base_local_planner::MapGridCostFunction alignment_costs_;
      base_local_planner::AlignmentCostFunction alignment_costs_;
      base_local_planner::CmdVelCostFunction cmd_vel_costs_;

      base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
  };
}
#endif
