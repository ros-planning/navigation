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
#include <dwa_local_planner/dwa_planner.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

namespace dwa_local_planner {
  /**
   * @class DWAPlannerROS
   * @brief ROS Wrapper for the DWAPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class DWAPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Constructor for DWAPlannerROS wrapper
       */
      DWAPlannerROS();

      void initialize(std::string name, tf::TransformListener* tf,
            costmap_2d::Costmap2DROS* costmap_ros);

      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      bool isGoalReached();

      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      bool isInitialized() {
        return initialized_;
      }

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base, using dynamic window approach
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool dwaComputeVelocityCommands(tf::Stamped<tf::Pose>& global_pose, geometry_msgs::Twist& cmd_vel);

    private:


      void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      void updateDataPassive(tf::Stamped<tf::Pose>& global_pose);

      // for visualisation, publishers of global and local plan
      ros::Publisher g_plan_pub_, l_plan_pub_;

      boost::shared_ptr<DWAPlanner> dp_;

      base_local_planner::LatchedStopRotateController latchedStopRotateController_;


      bool initialized_;


      base_local_planner::OdometryHelperRos odom_helper_;
  };
};
#endif
