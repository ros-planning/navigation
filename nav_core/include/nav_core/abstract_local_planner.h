/*
 *  Copyright 2017, Sebastian Pütz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  abstract_local_planner.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#ifndef NAV_CORE_ABSTRACT_LOCAL_PLANNER_H_
#define NAV_CORE_ABSTRACT_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <boost/shared_ptr.hpp>

namespace nav_core{

  class AbstractLocalPlanner{

    public:

      typedef boost::shared_ptr< ::nav_core::AbstractLocalPlanner > Ptr;

      /**
       * @brief Destructor
       */
      virtual ~AbstractLocalPlanner(){};

      /**
       * @brief Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base.
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @param plugin_code More detailed outcome in case of failure, so the high level executive
       * can take better decisions. move_base_flex_msgs developers suggest to use one of the error
       * codes defined in move_base_flex_msgs/ExePath action.
       * Will be defaulted to DO_NOT_APPLY on planners not implementing the new move_base_flex API
       * @param plugin_msg More detailed outcome as a string message
       * @return True if a valid velocity command was found, false otherwise
       */
      virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel,
                                           uint8_t& plugin_code, std::string& plugin_msg) = 0;

      /**
       * @brief Check if the goal pose has been achieved by the local planner
       * @param angle_tolerance The angle tolerance in which the current pose will be partly accepted as reached goal
       * @param dist_tolerance The distance tolerance in which the current pose will be partly accepted as reached goal
       * @return True if achieved, false otherwise
       */
      virtual bool isGoalReached(double dist_tolerance, double angle_tolerance) = 0;

      /**
       * @brief Set the plan that the local planner is following
       * @param plan The plan to pass to the local planner
       * @return True if the plan was updated successfully, false otherwise
       */
      virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;

      /**
       * @brief Requests the planner to cancel, e.g. if it takes to much time.
       * @return True if a cancel has been successfully requested, false if not implemented.
       */
      virtual bool cancel() = 0;

    protected:
      /**
       * @brief Constructor
       */
      AbstractLocalPlanner(){};
  };
} /* namespace nav_core */

#endif /* abstract_local_planner.h */
