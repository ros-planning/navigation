/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Clearpath Robotics, Inc.
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
* Author: Jason Mercer
*********************************************************************/

#ifndef NAV_NAV_CORE_NAV_GOAL_MANAGER_H_
#define NAV_NAV_CORE_NAV_GOAL_MANAGER_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include "nav_core/nav_goal.h"

namespace nav_core
{

class NavGoalMananger
{
public:
  typedef boost::shared_ptr<NavGoalMananger> Ptr;
  NavGoalMananger()
    : active_goal_(false), next_goal_id_(0)
  {

  }

  /**
   * @brief setCurrentGoal set the goal for the goal manager
   * @param goal new goal
   */
  void setCurrentGoal(const NavGoal& goal)
  {
    boost::upgrade_lock<boost::shared_mutex> lock(_access);
    boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(lock);

    if (!active_goal_ || !goal.equalPose(current_goal_))
    {
      current_goal_ = goal;
      current_goal_.id_ = ++next_goal_id_;
    }

    setActiveGoal(true);
  }
  /**
   * @brief currentGoal get the current goal of the manager
   * @return current goal
   */
  NavGoal currentGoal()
  {
    boost::shared_lock<boost::shared_mutex> read_lock(_access);

    return NavGoal(current_goal_);
  }

  /**
   * @brief setActiveGoal set if there is or is not an active goal
   * @param b new active goal flag
   */
  void setActiveGoal(bool b)
  {
    active_goal_ = b;
  }

  /**
   * @brief activeGoal determine if there is an active goal
   * @return active goal flag
   */
  bool activeGoal() const
  {
    return active_goal_;
  }


private:
  NavGoal current_goal_;
  bool active_goal_;
  boost::shared_mutex _access;
  NavGoal::id_type next_goal_id_;
};

}  // namespace move_base

#endif  // NAV_NAV_CORE_NAV_GOAL_MANAGER_H_
