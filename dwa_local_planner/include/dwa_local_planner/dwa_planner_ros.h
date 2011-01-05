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
#include <angles/angles.h>
#include <dwa_local_planner/dwa_planner.h>
#include <boost/shared_ptr.hpp>
#include <nav_core/base_local_planner.h>

namespace dwa_local_planner {
  /**
   * @class DWAPlannerROS
   * @brief ROS Wrapper for the DWAPlanner that adheres to the BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class DWAPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Constructor for DWAPlannerROS wrapper
       */
      DWAPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    private:
      inline double sign(double x){
        return x < 0.0 ? -1.0 : 1.0;
      }

      /**
       * @brief Once a goal position is reached... rotate to the goal orientation
       * @param  global_pose The pose of the robot in the global frame
       * @param  robot_vel The velocity of the robot
       * @param  goal_th The desired th value for the goal
       * @param  cmd_vel The velocity commands to be filled
       * @return  True if a valid trajectory was found, false otherwise
       */
      bool rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel);
      /**
       * @brief Stop the robot taking into account acceleration limits
       * @param  global_pose The pose of the robot in the global frame
       * @param  robot_vel The velocity of the robot
       * @param  cmd_vel The velocity commands to be filled
       * @return  True if a valid trajectory was found, false otherwise
       */
      bool stopWithAccLimits(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Callback for receiving odometry data
       * @param msg An Odometry message 
       */
      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

      costmap_2d::Costmap2DROS* costmap_ros_;
      tf::TransformListener* tf_;
      double max_vel_th_, min_vel_th_, min_rot_vel_;
      double rot_stopped_vel_, trans_stopped_vel_;
      double yaw_goal_tolerance_, xy_goal_tolerance_;
      bool prune_plan_;
      bool initialized_;
      ros::Subscriber odom_sub_;
      ros::Publisher g_plan_pub_, l_plan_pub_;
      boost::mutex odom_mutex_;
      nav_msgs::Odometry base_odom_;
      boost::shared_ptr<DWAPlanner> dp_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      bool rotating_to_goal_;
      bool latch_xy_goal_tolerance_, xy_tolerance_latch_;
  };
};
#endif
