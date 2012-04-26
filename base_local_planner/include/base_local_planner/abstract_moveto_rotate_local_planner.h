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
#ifndef ABSTRACT_MOVETO_ROTATE_LOCAL_PLANNER_H_
#define ABSTRACT_MOVETO_ROTATE_LOCAL_PLANNER_H_

#include <cmath>

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>

#include <base_local_planner/goal_functions.h>

#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>

namespace base_local_planner {

/**
 * @class AbstractMoveToRotateLocalPlanner
 * @brief Abstract Local planner class for local planner who use any
 * strategy to get to the goal point, and then stop dead and rotate
 * to the goal orientation. Use this if you feel your algorithm cannot
 * well handle the specific situation of arriving at the goal orientation.
 */
class AbstractMoveToRotateLocalPlanner : public nav_core::BaseLocalPlanner {

private:
  // whether to latch at all, and whether in this turn we have already been in goal area
  bool latch_xy_goal_tolerance_, xy_tolerance_latch_;
  bool rotating_to_goal_;
  // whether to discard points we have passed
  bool prune_plan_;
  bool initialized_;

protected:
  LocalPlannerUtil planner_util_;
  OdometryHelperRos odom_helper_;

public:

  void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Once a goal position is reached... rotate to the goal orientation
   * @return  True if a valid trajectory was found, false otherwise
   */
  virtual bool rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel) = 0;

  /**
   * @brief Stop the robot taking into account acceleration limits
   */
  virtual bool stopWithAccLimits(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, geometry_msgs::Twist& cmd_vel) = 0;

  virtual bool doComputeVelocityCommands(tf::Stamped<tf::Pose>& global_pose, geometry_msgs::Twist& cmd_vel) = 0;

  // to call when we do not invoke doComputeVelocityCommands
  virtual void updateDataPassive(tf::Stamped<tf::Pose>& global_pose) = 0;

  AbstractMoveToRotateLocalPlanner() : initialized_(false), odom_helper_("odom") {}

  /**
   * @brief  Destructs a trajectory controller
   */
  virtual ~AbstractMoveToRotateLocalPlanner() {};

  bool isGoalReached();

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);


  bool isInitialized() {
    return initialized_;
  }
};



void AbstractMoveToRotateLocalPlanner::initialize(
    std::string name,
    tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* costmap_ros) {
  if (! isInitialized()) {
    planner_util_.initialize(name, tf, costmap_ros);
    rotating_to_goal_ = false;
    initialized_ = true;
  } else {
    ROS_WARN("This planner has already been initialized, doing nothing.");
  }
}

bool AbstractMoveToRotateLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  //when we get a new plan, we also want to clear any latch we may have on goal tolerances
  xy_tolerance_latch_ = false;
  ROS_INFO("Got new plan");
  return planner_util_.setPlan(orig_global_plan);
}

/**
 * returns true if we have passed the goal position and have reached goal orientation.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true.
 */
bool AbstractMoveToRotateLocalPlanner::isGoalReached() {
  if (! isInitialized()) {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  //copy over the odometry information
  nav_msgs::Odometry base_odom;
  odom_helper_.getOdom(base_odom);

  tf::Stamped<tf::Pose> global_pose;
  if (!planner_util_.getRobotPose(global_pose)) {
    return false;
  }
  //we assume the global goal is the last point in the global plan
  tf::Stamped<tf::Pose> goal_pose;
  if ( ! planner_util_.getGoal(goal_pose)) {
    return false;
  }

  double goal_x = goal_pose.getOrigin().getX();
  double goal_y = goal_pose.getOrigin().getY();
  double goal_th = tf::getYaw(goal_pose.getRotation());

  base_local_planner::LocalPlannerLimitsConfig limits = planner_util_.getCurrentLimits();

  //check to see if we've reached the goal position
  if (xy_tolerance_latch_ || base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= limits.xy_goal_tolerance) {
    //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
    //just rotate in place
    if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_) {
      ROS_DEBUG("Goal position reached (check), stopping and turning in place");
      xy_tolerance_latch_ = true;
    }
    double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
    //check to see if the goal orientation has been reached
    if (fabs(angle) <= limits.yaw_goal_tolerance) {
      //make sure that we're actually stopped before returning success
      if (base_local_planner::stopped(base_odom, limits.rot_stopped_vel, limits.trans_stopped_vel)) {
        return true;
      }
    }
  }
  return false;
}

bool AbstractMoveToRotateLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  tf::Stamped<tf::Pose> global_pose;
  if ( ! planner_util_.getRobotPose(global_pose)) {
    ROS_ERROR("Could not get robot pose");
    return false;
  }
  //we assume the global goal is the last point in the global plan
  tf::Stamped<tf::Pose> goal_pose;
  if ( ! planner_util_.getGoal(goal_pose)) {
    ROS_ERROR("Could not get goal pose");
    return false;
  }

  double goal_x = goal_pose.getOrigin().getX();
  double goal_y = goal_pose.getOrigin().getY();
  double goal_th = tf::getYaw(goal_pose.getRotation());
  base_local_planner::LocalPlannerLimitsConfig limits = planner_util_.getCurrentLimits();

  double goal_distance = base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y);
  //check to see if we've reached the goal position
  if (xy_tolerance_latch_ || goal_distance <= limits.xy_goal_tolerance) {
    //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
    //just rotate in place
    if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_ ) {
      ROS_INFO("Goal position reached, stopping and turning in place");
      xy_tolerance_latch_ = true;
    }
    //check to see if the goal orientation has been reached
    double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
    if (fabs(angle) <= limits.yaw_goal_tolerance) {
      //set the velocity command to zero
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      rotating_to_goal_ = false;
    } else {
      ROS_DEBUG("Angle: %f Tolerance: %f", angle, limits.yaw_goal_tolerance);
      tf::Stamped<tf::Pose> robot_vel;
      odom_helper_.getRobotVel(robot_vel);
      nav_msgs::Odometry base_odom;
      odom_helper_.getOdom(base_odom);

      //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
      if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, limits.rot_stopped_vel, limits.trans_stopped_vel)) {
        if ( ! stopWithAccLimits(global_pose, robot_vel, cmd_vel)) {
          ROS_INFO("Error when stopping.");
          return false;
        }
        ROS_DEBUG("Stopping...");
      }
      //if we're stopped... then we want to rotate to goal
      else {
        //set this so that we know its OK to be moving
        rotating_to_goal_ = true;
        if ( ! rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel)) {
          ROS_INFO("Error when rotating.");
          return false;
        }
        ROS_DEBUG("Rotating...");
      }
    }

    updateDataPassive(global_pose);

    //publish an empty plan because we've reached our goal position
    std::vector<geometry_msgs::PoseStamped> local_plan;
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    planner_util_.publishGlobalPlan(transformed_plan);
    planner_util_.publishLocalPlan(local_plan);

    return true;
  }
  ROS_DEBUG("Distance %f Tolerance %f", goal_distance, limits.xy_goal_tolerance);
  return doComputeVelocityCommands(global_pose, cmd_vel);
}



};

#endif /* ABSTRACT_MOVETO_ROTATE_LOCAL_PLANNER_H_ */
