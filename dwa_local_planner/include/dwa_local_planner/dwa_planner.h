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

#include <base_local_planner/trajectory.h>
#include <base_local_planner/LocalPlannerLimitsConfig.h>


#include <dynamic_reconfigure/server.h>
#include <dwa_local_planner/DWAPlannerConfig.h>

//for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>
#include <pcl_ros/publisher.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/prefer_forward_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

namespace dwa_local_planner {
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
       */
      DWAPlanner(std::string name, const costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the planner
       */
      ~DWAPlanner() {}


      /**
       * @brief  Check if a trajectory is legal for a position/velocity pari
       * @param pos The robot's position 
       * @param vel The desired velocity
       * @return True if the trajectory is valid, false otherwise
       */
      bool checkTrajectory(
          const Eigen::Vector3f pos,
          const Eigen::Vector3f vel,
          const base_local_planner::LocalPlannerLimitsConfig& limits);

      /**
       * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
       * @param global_pose The current position of the robot 
       * @param global_vel The current velocity of the robot 
       * @param drive_velocities The velocities to send to the robot base
       * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
       */
      base_local_planner::Trajectory findBestPath(
          tf::Stamped<tf::Pose> global_pose,
          tf::Stamped<tf::Pose> global_vel,
          tf::Stamped<tf::Pose>& drive_velocities,
          const base_local_planner::LocalPlannerLimitsConfig& limits);

      /**
       * @brief  Take in a new global plan for the local planner to follow
       * @param  new_plan The new global plan
       */
      void updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan);

      /**
       * @brief  Get the acceleration limits of the robot
       * @return  The acceleration limits of the robot
       */
      Eigen::Vector3f getAccLimits() { return acc_lim_; }

      /**
       * @brief Get the period at which the local planner is expected to run
       * @return The simulation period
       */
      double getSimPeriod() { return sim_period_; }

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

    private:
      /**
       * @brief  Callback to update the local planner's parameters based on dynamic reconfigure
       */
      void reconfigureCB(DWAPlannerConfig &config, uint32_t level);


      void publishTrajectories();

      const costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D costmap_;
      double stop_time_buffer_;
      double pdist_scale_, gdist_scale_, occdist_scale_, heading_scale_;
      Eigen::Vector3f acc_lim_, vsamples_;

      double sim_time_, sim_granularity_;

      double sim_period_;
      base_local_planner::Trajectory result_traj_;


      double forward_point_distance_;
      double scaling_speed_, max_scaling_factor_;
      double backward_motion_penalty_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;

      dynamic_reconfigure::Server<DWAPlannerConfig> dsrv_;
      dwa_local_planner::DWAPlannerConfig default_config_;
      bool setup_;
      boost::mutex configuration_mutex_;
      bool penalize_negative_x_;
      pcl::PointCloud<base_local_planner::MapGridCostPoint> traj_cloud_;
      pcl_ros::Publisher<base_local_planner::MapGridCostPoint> traj_cloud_pub_;
      base_local_planner::MapGridVisualizer map_viz_; ///< @brief The map grid visualizer for outputting the potential field generated by the cost function

      // see constructor body for explanations
      base_local_planner::SimpleTrajectoryGenerator generator_;
      base_local_planner::OscillationCostFunction oscillation_costs_;
      base_local_planner::ObstacleCostFunction obstacle_costs_;
      base_local_planner::PreferForwardCostFunction prefer_forward_costs_;
      base_local_planner::MapGridCostFunction path_costs_;
      base_local_planner::MapGridCostFunction goal_costs_;
      base_local_planner::MapGridCostFunction goal_front_costs_;
      base_local_planner::MapGridCostFunction alignment_costs_;

      base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
  };
};
#endif
