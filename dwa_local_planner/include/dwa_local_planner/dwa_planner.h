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
#include <queue>
#include <vector>
#include <Eigen3/Core>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/map_grid.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <dwa_local_planner/velocity_iterator.h>

#include <dynamic_reconfigure/server.h>
#include <dwa_local_planner/DWAPlannerConfig.h>

#include <base_local_planner/map_grid_visualizer.h>

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
      DWAPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the planner
       */
      ~DWAPlanner() {delete world_model_;}

      /**
       * @brief  Given a current position, velocity, and timestep... compute a new position
       * @param  pos The current position
       * @param  vel The current velocity
       * @param  dt The timestep
       * @return The new position after applying the velocity for a timestep
       */
      Eigen3::Vector3f computeNewPositions(const Eigen3::Vector3f& pos, const Eigen3::Vector3f& vel, double dt);


      /**
       * @brief  Given the current position of the robot and a desired velocity, generate and score a trajectory for the given velocity
       * @param pos The current position of the robot 
       * @param vel The desired velocity for the trajectory
       * @param traj A reference to the Trajectory to be populated. A cost >= 0 for the trajectory means that it is valid.
       * @param two_point_scoring Whether to score the trajectory based on the
       * center point of the robot and a point directly in front of the center
       * point, or to score the trajectory only basaed on the center point of the robot
       */
      void generateTrajectory(Eigen3::Vector3f pos, const Eigen3::Vector3f& vel, base_local_planner::Trajectory& traj, bool two_point_scoring);

      /**
       * @brief  Given the current position and velocity of the robot, computes
       * and scores a number of possible trajectories to execute, returning the
       * best option
       * @param  pos The current position of the robot
       * @param  vel The current velocity of the robot
       * @return The highest scoring trajectory, a cost >= 0 corresponds to a valid trajectory
       */
      base_local_planner::Trajectory computeTrajectories(const Eigen3::Vector3f& pos, const Eigen3::Vector3f& vel);

      /**
       * @brief  Check if a trajectory is legal for a position/velocity pari
       * @param pos The robot's position 
       * @param vel The desired velocity
       * @return True if the trajectory is valid, false otherwise
       */
      bool checkTrajectory(const Eigen3::Vector3f& pos, const Eigen3::Vector3f& vel);

      /**
       * @brief Given the current position and velocity of the robot, find the best trajectory to exectue
       * @param global_pose The current position of the robot 
       * @param global_vel The current velocity of the robot 
       * @param drive_velocities The velocities to send to the robot base
       * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
       */
      base_local_planner::Trajectory findBestPath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel, 
          tf::Stamped<tf::Pose>& drive_velocities);

      /**
       * @brief  Take in a new global plan for the local planner to follow
       * @param  new_plan The new global plan
       */
      void updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan);

      /**
       * @brief  Get the acceleration limits of the robot
       * @return  The acceleration limits of the robot
       */
      Eigen3::Vector3f getAccLimits() { return acc_lim_; }

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

      /**
       * @brief  Given a position for the robot and a scale for the footprint, compute a cost
       * @param pos The pose of the robot
       * @param scale The scaling factor for the footprint
       * @return  A cost for the footprint... >= 0 is legal
       */
      double footprintCost(const Eigen3::Vector3f& pos, double scale);

      /**
       * @brief  Given two trajectories to compare... select the best one
       * @param  best The current best trajectory, will be set to the new best trajectory if comp scores higher
       * @param  comp The trajectory to compare to the current best trajectory
       */
      void selectBestTrajectory(base_local_planner::Trajectory* &best, base_local_planner::Trajectory* &comp);

      /**
       * @brief  Reset the oscillation flags for the local planner
       */
      void resetOscillationFlags();

      /**
       * @brief  Given the robot's current position and the position where
       * oscillation flags were last set, check to see if the robot had moved
       * far enough to reset them.
       * @param  pos The current position of the robot
       * @param  prev The position at which the oscillation flags were last set
       * @return 
       */
      void resetOscillationFlagsIfPossible(const Eigen3::Vector3f& pos, const Eigen3::Vector3f& prev);

      /**
       * @brief  Given a trajectory that's selected, set flags if needed to
       * prevent the robot from oscillating
       * @param  t The selected trajectory
       * @return True if a flag was set, false otherwise
       */
      bool setOscillationFlags(base_local_planner::Trajectory* t);

      /**
       * @brief Compute the square distance between two poses
       */
      inline double squareDist(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
        return (p1.pose.position.x - p2.pose.position.x) * (p1.pose.position.x - p2.pose.position.x)
          + (p1.pose.position.y - p2.pose.position.y) * (p1.pose.position.y - p2.pose.position.y);
      }

      /**
       * @brief Given a time and the robot's acceleration limits, get the
       * maximum velocity for the robot to be able to stop in the alloted time
       * period
       */
      inline Eigen3::Vector3f getMaxSpeedToStopInTime(double time){
        return acc_lim_ * std::max(time, 0.0);
      }

      /**
       * @brief  For a given velocity, check it it is illegal because of the oscillation flags set
       */
      bool oscillationCheck(const Eigen3::Vector3f& vel);

      base_local_planner::MapGrid map_, front_map_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D costmap_;
      double stop_time_buffer_;
      double pdist_scale_, gdist_scale_, occdist_scale_, heading_scale_;
      Eigen3::Vector3f acc_lim_, vsamples_, prev_stationary_pos_;
      std::vector<geometry_msgs::Point> footprint_spec_;
      base_local_planner::CostmapModel* world_model_;
      double sim_time_, sim_granularity_;
      double max_vel_x_, min_vel_x_;
      double max_vel_y_, min_vel_y_, min_vel_trans_, max_vel_trans_;
      double max_vel_th_, min_vel_th_, min_rot_vel_;
      double sim_period_;
      base_local_planner::Trajectory traj_one_, traj_two_;
      bool strafe_pos_only_, strafe_neg_only_, strafing_pos_, strafing_neg_;
      bool rot_pos_only_, rot_neg_only_, rotating_pos_, rotating_neg_;
      bool forward_pos_only_, forward_neg_only_, forward_pos_, forward_neg_;
      double oscillation_reset_dist_;
      double heading_lookahead_, forward_point_distance_;
      double scaling_speed_, max_scaling_factor_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      dynamic_reconfigure::Server<DWAPlannerConfig> dsrv_;
      boost::mutex configuration_mutex_;
      bool penalize_negative_x_;
      base_local_planner::MapGridVisualizer map_viz_; ///< @brief The map grid visualizer for outputting the potential field generated by the cost function
  };
};
#endif
