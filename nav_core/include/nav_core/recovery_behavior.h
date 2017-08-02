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
#ifndef NAV_CORE_RECOVERY_BEHAVIOR_H
#define NAV_CORE_RECOVERY_BEHAVIOR_H

#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include "nav_core/abstract_recovery_behavior.h"

namespace nav_core {
  /**
   * @class RecoveryBehavior
   * @brief Provides an interface for recovery behaviors used in navigation.
   * All recovery behaviors written as plugins for the navigation stack must adhere to this interface.
   */
  class RecoveryBehavior : public AbstractRecoveryBehavior{
    public:

      typedef boost::shared_ptr< ::nav_core::RecoveryBehavior> Ptr;

      /**
       * @brief Initialization function for the RecoveryBehavior
       * @param tf A pointer to a transform listener
       * @param global_costmap A pointer to the global_costmap used by the navigation stack
       * @param local_costmap A pointer to the local_costmap used by the navigation stack
       */
      virtual void initialize(std::string name, tf::TransformListener* tf,
                              costmap_2d::Costmap2DROS* global_costmap,
                              costmap_2d::Costmap2DROS* local_costmap) = 0;

      /**
       * @brief Runs the RecoveryBehavior
       */
      virtual void runBehavior() = 0;

      /**
       * @brief Requests the planner to cancel, e.g. if it takes to much time.
       * @return True if a cancel has been successfully requested, false if not implemented.
       */
      virtual bool cancel()
      {
        return false;
      }

      /**
       * @brief Virtual destructor for the interface
       */
      virtual ~RecoveryBehavior(){}

    protected:
      RecoveryBehavior(){}
  };
};  // namespace nav_core

#endif  // NAV_CORE_RECOVERY_BEHAVIOR_H