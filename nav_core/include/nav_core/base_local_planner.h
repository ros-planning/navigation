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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef NAV_CORE_BASE_LOCAL_PLANNER_H
#define NAV_CORE_BASE_LOCAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include "nav_core/abstract_local_planner.h"

namespace nav_core {
  /**
   * @class BaseLocalPlanner
   * @brief Provides an interface for local planners used in navigation. All local planners written as plugins for the navigation stack must adhere to this interface.
   */
  class BaseLocalPlanner : public AbstractLocalPlanner{
    public:

      typedef boost::shared_ptr< ::nav_core::BaseLocalPlanner > Ptr;

      /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base.
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid velocity command was found, false otherwise
       *
       * @deprecated This method is deprecated in move_base_flex in favor of the one providing detailed result.
       */
      virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
      {
        throw "Neither old nor flex APIs computeVelocityCommands method overridden";
      }

      /**
       * @brief Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base.
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @param plugin_code More detailed outcome; will be defaulted to DO_NOT_APPLY on planners
       * implementing the old move_base API
       * @param plugin_msg More detailed outcome as a string message
       * @return True if a valid velocity command was found, false otherwise
       */
      virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel,
                                           uint8_t& plugin_code, std::string& plugin_msg)
      {
        plugin_code = 255;  // DO_NOT_APPLY
        return computeVelocityCommands(cmd_vel);
      }

      /**
       * @brief Check if the goal pose has been achieved by the local planner
       * @return True if achieved, false otherwise
       */
      virtual bool isGoalReached() = 0;

      virtual bool isGoalReached(double dist_tolerance, double angle_tolerance)
      {
        return isGoalReached();
      }

      /**
       * @brief  Set the plan that the local planner is following
       * @param plan The plan to pass to the local planner
       * @return True if the plan was updated successfully, false otherwise
       */
      virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;

      /**
       * @brief Requests the planner to cancel, e.g. if it takes to much time.
       * @return True if a cancel has been successfully requested, false if not implemented.
       */
      virtual bool cancel()
      {
        return false;
      }

      /**
       * @brief Constructs the local planner
       * @param name The name to give this instance of the local planner
       * @param tf A pointer to a transform listener
       * @param costmap_ros The cost map to use for assigning costs to local plans
       */
      virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~BaseLocalPlanner(){}

    protected:
      BaseLocalPlanner(){}
  };
};  // namespace nav_core

#endif  // NAV_CORE_BASE_LOCAL_PLANNER_H