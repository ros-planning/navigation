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
#ifndef NAV_CORE_RECOVERY_BEHAVIOR_H_
#define NAV_CORE_RECOVERY_BEHAVIOR_H_
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/base_local_planner.h>

namespace nav_core {
  /**
   * @class RecoveryBehavior
   * @brief Provides an interface for recovery behaviors used in navigation. All recovery behaviors written as plugins for the navigation stack must adhere to this interface.
   */
  class RecoveryBehavior{
    public:
      /**
       * @brief  Initialization function for the RecoveryBehavior
       * @param tf A pointer to a transform listener
       * @param global_costmap A pointer to the global_costmap used by the navigation stack 
       * @param local_costmap A pointer to the local_costmap used by the navigation stack 
       */
      virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap) = 0;

      /**
       * @brief   Runs the RecoveryBehavior
       */
      virtual void runBehavior() = 0;

      /**
       * @brief   Cleans up any system-wide changes made by the behavior
       *
       * After the behavior executes, we return to planning state, which may succeed or fail. In either case, we want
       * to undo any system-wide changes we may have made during the execution of the behavior. This method runs when
       * we execute the next recovery behavior, when post-recovery planning succeeds, or all recoveries fail.
       */
      virtual void revertChanges() { };

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~RecoveryBehavior(){}

      /**
       * @brief Set the function that will get the current local planner
       * @param local_planner_fetch A boost function that will return the current local planner
       */
      virtual void setLocalPlannerFetchFunction(BaseLocalPlanner::FetchFunction f)
      {
        local_planner_fetch_ = f;
      }

      /**
       * @brief Set the function that will get the current global planner
       * @param global_planner_fetch A boost function that will return the current global planner
       */
      virtual void setGlobalPlannerFetchFunction(BaseGlobalPlanner::FetchFunction f)
      {
        global_planner_fetch_ = f;
      }

    protected:
      RecoveryBehavior(){}

      BaseGlobalPlanner::FetchFunction global_planner_fetch_;
      BaseLocalPlanner::FetchFunction local_planner_fetch_;
  };
};
#endif
