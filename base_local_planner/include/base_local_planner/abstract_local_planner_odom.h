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

#ifndef ABSTRACT_LOCAL_PLANNER_ODOM_H_
#define ABSTRACT_LOCAL_PLANNER_ODOM_H_

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>

#include <base_local_planner/LocalPlannerLimitsConfig.h>
#include <base_local_planner/goal_functions.h>

namespace base_local_planner {

/**
 * @class AbstractLocalPlannerOdom
 * @brief Basic abstract class from which to derive other local planners.
 * Maintains odometry information from /odom topic. Has parameter to remember
 * that goal position has been reached already, while orientation not yet, to
 * prevent rotational shift to matter.
 */
class AbstractLocalPlannerOdom : public nav_core::BaseLocalPlanner {
  bool initialized_;

  // things we get from move_base
  std::string name_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  tf::TransformListener* tf_;

  // we listen on odometry on the odom topic
  ros::Subscriber odom_sub_;
  nav_msgs::Odometry base_odom_;
  boost::mutex odom_mutex_;

  std::vector<geometry_msgs::PoseStamped> global_plan_;
  // for visualisation, publishers of global and local plan
  ros::Publisher g_plan_pub_, l_plan_pub_;


  dynamic_reconfigure::Server<LocalPlannerLimitsConfig> *dsrv_;
  boost::mutex limits_configuration_mutex_;
  bool setup_;
  LocalPlannerLimitsConfig default_limits_;

  LocalPlannerLimitsConfig limits_;

public:

  /**
   * @brief  Callback to update the local planner's parameters based on dynamic reconfigure
   */
  void reconfigureCB(LocalPlannerLimitsConfig &config, uint32_t level);

  AbstractLocalPlannerOdom() : initialized_(false) {}

  /**
   * @brief  Destructs a trajectory controller
   */
  ~AbstractLocalPlannerOdom() {
    if (initialized_) {
      delete(dsrv_);
    }
  }

  void initialize(std::string name, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Callback for receiving odometry data
   * @param msg An Odometry message
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  void getRobotVel(tf::Stamped<tf::Pose>& robot_vel);

  bool getRobotPose(tf::Stamped<tf::Pose>& global_pose);

  bool getGoal(tf::Stamped<tf::Pose>& goal_pose);

  void getOdom(nav_msgs::Odometry& base_odom);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  bool getLocalPlan(tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan);

  void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

  void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

  costmap_2d::Costmap2DROS* getCostmapRos() {
    return costmap_ros_;
  }

  tf::TransformListener* getTfListener() {
    return tf_;
  }

  std::string getName() {
    return name_;
  }

  bool isInitialized() {
    return initialized_;
  }

  LocalPlannerLimitsConfig getCurrentLimits() {
    return limits_;
  }
};



void AbstractLocalPlannerOdom::initialize(std::string name,
    tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* costmap_ros) {
  if(!initialized_){
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    //to get odometry information, we need to get a handle to the topic in the global namespace
    ros::NodeHandle gn;
    odom_sub_ = gn.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&AbstractLocalPlannerOdom::odomCallback, this, _1));

    name_ = name;

    ros::NodeHandle pn("~/" + getName());
    g_plan_pub_ = pn.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = pn.advertise<nav_msgs::Path>("local_plan", 1);

    ros::NodeHandle pn2("~/localPlannerLimits");
    dsrv_ = new dynamic_reconfigure::Server<LocalPlannerLimitsConfig>(pn2);

    dynamic_reconfigure::Server<LocalPlannerLimitsConfig>::CallbackType cb = boost::bind(&AbstractLocalPlannerOdom::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    initialized_ = true;
  }
  else{
    ROS_WARN("This planner has already been initialized, doing nothing.");
  }
}

void AbstractLocalPlannerOdom::reconfigureCB(LocalPlannerLimitsConfig &config, uint32_t level)
{
  if(setup_ && config.restore_defaults) {
    config = default_limits_;
    config.restore_defaults = false;
  }

  if(!setup_) {
    default_limits_ = config;
    setup_ = true;
  }
  boost::mutex::scoped_lock l(limits_configuration_mutex_);
  limits_ = LocalPlannerLimitsConfig(config);
}

void AbstractLocalPlannerOdom::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  //we assume that the odometry is published in the frame of the base
  boost::mutex::scoped_lock lock(odom_mutex_);
  base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
  base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
  base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
  ROS_DEBUG_NAMED("dwa_local_planner", "In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
      base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
}


bool AbstractLocalPlannerOdom::getRobotPose(tf::Stamped<tf::Pose>& global_pose) {
  if(!costmap_ros_->getRobotPose(global_pose)) {
    return false;
  }
  return true;
}


bool AbstractLocalPlannerOdom::getGoal(tf::Stamped<tf::Pose>& goal_pose) {
  //we assume the global goal is the last point in the global plan
  return base_local_planner::getGoalPose(*tf_,
        global_plan_,
        costmap_ros_->getGlobalFrameID(),
        goal_pose);
}

//copy over the odometry information
void AbstractLocalPlannerOdom::getOdom(nav_msgs::Odometry& base_odom) {
  boost::mutex::scoped_lock lock(odom_mutex_);
  base_odom = base_odom_;
}

bool AbstractLocalPlannerOdom::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
  if(!initialized_){
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  boost::mutex odom_mutex_;

  //reset the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  return true;
}



void AbstractLocalPlannerOdom::getRobotVel(tf::Stamped<tf::Pose>& robot_vel) {
  // Set current velocities from odometry
  geometry_msgs::Twist global_vel;
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    global_vel.linear.x = base_odom_.twist.twist.linear.x;
    global_vel.linear.y = base_odom_.twist.twist.linear.y;
    global_vel.angular.z = base_odom_.twist.twist.angular.z;
  }
  robot_vel.setData(tf::Transform(tf::createQuaternionFromYaw(global_vel.angular.z), tf::Vector3(global_vel.linear.x, global_vel.linear.y, 0)));
  robot_vel.frame_id_ = costmap_ros_->getBaseFrameID();
  robot_vel.stamp_ = ros::Time();
}


bool AbstractLocalPlannerOdom::getLocalPlan(tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan) {
  //get the global plan in our frame
  if(!base_local_planner::transformGlobalPlan(
      *getTfListener(),
      global_plan_,
      *getCostmapRos(),
      getCostmapRos()->getGlobalFrameID(),
      transformed_plan)) {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }

  //now we'll prune the plan based on the position of the robot
  if(limits_.prune_plan) {
    base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);
  }
  return true;
}


void AbstractLocalPlannerOdom::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
  base_local_planner::publishPlan(path, l_plan_pub_);
}


void AbstractLocalPlannerOdom::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
  base_local_planner::publishPlan(path, l_plan_pub_);
}

};

#endif /* ABSTRACT_LOCAL_PLANNER_ODOM_H_ */
