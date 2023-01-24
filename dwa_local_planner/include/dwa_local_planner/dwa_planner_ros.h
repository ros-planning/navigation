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
#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>
#include <dwa_local_planner/DWAPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <mbf_costmap_core/costmap_controller.h>

#include <dwa_local_planner/dwa_planner.h>

namespace dwa_local_planner {
  /**
   * @class DWAPlannerROS
   * @brief ROS Wrapper for the DWAPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class DWAPlannerROS : public mbf_costmap_core::CostmapController {
    public:
      /**
       * @brief  Constructor for DWAPlannerROS wrapper
       */
      DWAPlannerROS();

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros) override;

      /**
       * @brief  Destructor for the wrapper
       */
      ~DWAPlannerROS();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param pose the current pose of the robot.
       * @param velocity the current velocity of the robot.
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
       * @param message Optional more detailed outcome as a string.
       * @return Result code as described on ExePath action result:
       *         SUCCESS         = 0
       *         1..9 are reserved as plugin specific non-error results
       *         FAILURE         = 100   Unspecified failure, only used for old, non-mfb_core based plugins
       *         CANCELED        = 101
       *         NO_VALID_CMD    = 102
       *         PAT_EXCEEDED    = 103
       *         COLLISION       = 104
       *         OSCILLATION     = 105
       *         ROBOT_STUCK     = 106
       *         MISSED_GOAL     = 107
       *         MISSED_PATH     = 108
       *         BLOCKED_PATH    = 109
       *         INVALID_PATH    = 110
       *         TF_ERROR        = 111
       *         NOT_INITIALIZED = 112
       *         INVALID_PLUGIN  = 113
       *         INTERNAL_ERROR  = 114
       *         121..149 are reserved as plugin specific errors
       */
      uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                       const geometry_msgs::TwistStamped& velocity,
                                       geometry_msgs::TwistStamped& cmd_vel, std::string& message) override;

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base, using dynamic window approach
       * @param global_pose the current pose of the robot.
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
       * @param message Optional more detailed outcome as a string.
       * @return Result code as described on ExePath action result (see computeVelocityCommands for details)
       */
      uint32_t dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose,
                                          geometry_msgs::TwistStamped& cmd_vel, std::string& message);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;

      /**
       * @brief  Check if the goal pose has been achieved.
       * @param dist_tolerance The distance tolerance in which the current pose will be partly accepted as reached goal
       * @param angle_tolerance The angle tolerance in which the current pose will be partly accepted as reached goal
       * @return True if achieved, false otherwise
       */
      bool isGoalReached(double dist_tolerance, double angle_tolerance) override;

      /**
       * @brief Requests the planner to cancel; not implemented for this planner
       * @return True if a cancel has been successfully requested, false if not implemented.
       */
      bool cancel() override { return false; };

      bool isInitialized() {
        return initialized_;
      }

    private:
      /**
       * @brief Callback to update the local planner's parameters based on dynamic reconfigure
       */
      void reconfigureCB(DWAPlannerConfig &config, uint32_t level);

      void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      void publishScaledFootprint(const geometry_msgs::PoseStamped& pose, const base_local_planner::Trajectory &traj) const;

      bool finishedBestEffort();

      void resetBestEffort();

      tf2_ros::Buffer* tf_; ///< @brief Used for transforming point clouds

      // for visualisation, publishers of global and local plan
      ros::Publisher g_plan_pub_, l_plan_pub_, scaled_fp_pub_;

      base_local_planner::LocalPlannerUtil planner_util_;

      boost::shared_ptr<DWAPlanner> dp_; ///< @brief The trajectory controller

      costmap_2d::Costmap2DROS* costmap_ros_;

      dynamic_reconfigure::Server<DWAPlannerConfig> *dsrv_;
      dwa_local_planner::DWAPlannerConfig default_config_;
      base_local_planner::LocalPlannerLimits _latest_limits; ///< @brief latest limits set by dynamic reconfigure
      bool setup_;
      geometry_msgs::PoseStamped current_pose_;

      base_local_planner::LatchedStopRotateController latchedStopRotateController_;

      geometry_msgs::PoseStamped current_goal_;

      bool initialized_;

      double controller_frequency_; ///< Calling frequency to this plugin

      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;

      int prev_vel_dir_;
      bool oscillating_;
      bool latched_inner_goal_;
      std::vector<geometry_msgs::PoseStamped> transformed_plan_;
      boost::optional<geometry_msgs::PoseStamped> outer_goal_entry_;

      // mutex to avoid reconfigure while we're scoring trajectories
      std::mutex config_mtx_;
  };
};
#endif
