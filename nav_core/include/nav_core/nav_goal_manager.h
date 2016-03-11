/**
Software License Agreement (proprietary)

\file      nav_goal_manager.h
\authors   Jason Mercer <jmercer@clearpathrobotics.com>
\copyright Copyright (c) 2016, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/

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
