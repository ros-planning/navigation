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
*   * Neither the name of the Willow Garage nor the names of its
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

#include <base_local_planner/trajectory_planner_ros.h>
#include <ros/console.h>
#include <sys/time.h>
#include <pluginlib/class_list_macros.h>

#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/Path.h"
#include "base_local_planner/goal_functions.h"

using namespace std;
using namespace costmap_2d;

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(base_local_planner, TrajectoryPlannerROS, base_local_planner::TrajectoryPlannerROS, nav_core::BaseLocalPlanner)

namespace base_local_planner {

  TrajectoryPlannerROS::TrajectoryPlannerROS() : world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

  TrajectoryPlannerROS::TrajectoryPlannerROS(std::string name, tf::TransformListener* tf, Costmap2DROS* costmap_ros) 
    : world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), initialized_(false){

      //initialize the planner
      initialize(name, tf, costmap_ros);
  }

  void TrajectoryPlannerROS::initialize(std::string name, tf::TransformListener* tf, Costmap2DROS* costmap_ros){
    if(!initialized_){
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      rot_stopped_velocity_ = 1e-2;
      trans_stopped_velocity_ = 1e-2;
      double acc_lim_x, acc_lim_y, acc_lim_theta, sim_time, sim_granularity;
      int vx_samples, vtheta_samples;
      double pdist_scale, gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta;
      bool holonomic_robot, dwa, simple_attractor, heading_scoring;
      double heading_scoring_timestep;
      double max_vel_x, min_vel_x;
      double backup_vel;
      string world_model_type;

      //initialize the copy of the costmap the controller will use
      costmap_ros_->getCostmapCopy(costmap_);

      ros::NodeHandle private_nh("~/" + name);

      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

      global_frame_ = costmap_ros_->getGlobalFrameID();
      robot_base_frame_ = costmap_ros_->getBaseFrameID();
      private_nh.param("prune_plan", prune_plan_, true);

      private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
      private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);

      //to get odometery information, we need to get a handle to the topic in the global namespace of the node
      ros::NodeHandle global_node;
      odom_sub_ = global_node.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&TrajectoryPlannerROS::odomCallback, this, _1));

      //we'll get the parameters for the robot radius from the costmap we're associated with
      inscribed_radius_ = costmap_ros_->getInscribedRadius();
      circumscribed_radius_ = costmap_ros_->getCircumscribedRadius();
      inflation_radius_ = costmap_ros_->getInflationRadius();

      private_nh.param("acc_lim_x", acc_lim_x, 2.5);
      private_nh.param("acc_lim_y", acc_lim_y, 2.5);
      private_nh.param("acc_lim_th", acc_lim_theta, 3.2);
      private_nh.param("sim_time", sim_time, 1.0);
      private_nh.param("sim_granularity", sim_granularity, 0.025);
      private_nh.param("vx_samples", vx_samples, 3);
      private_nh.param("vtheta_samples", vtheta_samples, 20);
      private_nh.param("path_distance_bias", pdist_scale, 0.6);
      private_nh.param("goal_distance_bias", gdist_scale, 0.8);
      private_nh.param("occdist_scale", occdist_scale, 0.01);
      private_nh.param("heading_lookahead", heading_lookahead, 0.325);
      private_nh.param("oscillation_reset_dist", oscillation_reset_dist, 0.05);
      private_nh.param("escape_reset_dist", escape_reset_dist, 0.10);
      private_nh.param("escape_reset_theta", escape_reset_theta, M_PI_4);
      private_nh.param("holonomic_robot", holonomic_robot, true);
      private_nh.param("max_vel_x", max_vel_x, 0.5);
      private_nh.param("min_vel_x", min_vel_x, 0.1);

      double max_rotational_vel;
      private_nh.param("max_rotational_vel", max_rotational_vel, 1.0);
      max_vel_th_ = max_rotational_vel;
      min_vel_th_ = -1.0 * max_rotational_vel;
      private_nh.param("min_in_place_rotational_vel", min_in_place_vel_th_, 0.4);

      private_nh.param("backup_vel", backup_vel, -0.1);
      private_nh.param("world_model", world_model_type, string("costmap")); 
      private_nh.param("dwa", dwa, true);
      private_nh.param("heading_scoring", heading_scoring, false);
      private_nh.param("heading_scoring_timestep", heading_scoring_timestep, 0.8);

      simple_attractor = false;

      //parameters for using the freespace controller
      double min_pt_separation, max_obstacle_height, grid_resolution;
      private_nh.param("point_grid/max_sensor_range", max_sensor_range_, 2.0);
      private_nh.param("point_grid/min_pt_separation", min_pt_separation, 0.01);
      private_nh.param("point_grid/max_obstacle_height", max_obstacle_height, 2.0);
      private_nh.param("point_grid/grid_resolution", grid_resolution, 0.2);

      ROS_ASSERT_MSG(world_model_type == "costmap", "At this time, only costmap world models are supported by this controller");
      world_model_ = new CostmapModel(costmap_); 

      std::vector<double> y_vels = loadYVels(private_nh);

      tc_ = new TrajectoryPlanner(*world_model_, costmap_, costmap_ros_->getRobotFootprint(), inscribed_radius_, circumscribed_radius_,
          acc_lim_x, acc_lim_y, acc_lim_theta, sim_time, sim_granularity, vx_samples, vtheta_samples, pdist_scale,
          gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta, holonomic_robot,
          max_vel_x, min_vel_x, max_vel_th_, min_vel_th_, min_in_place_vel_th_, backup_vel,
          dwa, heading_scoring, heading_scoring_timestep, simple_attractor, y_vels);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }

  std::vector<double> TrajectoryPlannerROS::loadYVels(ros::NodeHandle node){
    std::vector<double> y_vels;

    XmlRpc::XmlRpcValue y_vel_list;
    if(node.getParam("y_vels", y_vel_list)){
      ROS_ASSERT_MSG(y_vel_list.getType() == XmlRpc::XmlRpcValue::TypeArray, 
          "The y velocities to explore must be specified as a list");

      for(int i = 0; i < y_vel_list.size(); ++i){
        //make sure we have a list of lists of size 2
        XmlRpc::XmlRpcValue vel = y_vel_list[i];

        //make sure that the value we're looking at is either a double or an int
        ROS_ASSERT(vel.getType() == XmlRpc::XmlRpcValue::TypeInt || vel.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        double y_vel = vel.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(vel) : (double)(vel);

        y_vels.push_back(y_vel);

      }
    }
    else{
      //if no values are passed in, we'll provide defaults
      y_vels.push_back(-0.3);
      y_vels.push_back(-0.1);
      y_vels.push_back(0.1);
      y_vels.push_back(0.3);
    }

    return y_vels;
  }

  TrajectoryPlannerROS::~TrajectoryPlannerROS(){
    if(tc_ != NULL)
      delete tc_;

    if(world_model_ != NULL)
      delete world_model_;
  }

  bool TrajectoryPlannerROS::rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel){
    double yaw = tf::getYaw(global_pose.getRotation());
    double vel_yaw = tf::getYaw(robot_vel.getRotation());
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

    double v_theta_samp = ang_diff > 0.0 ? std::min(max_vel_th_,
        std::max(min_in_place_vel_th_, ang_diff)) : std::max(min_vel_th_,
        std::min(-1.0 * min_in_place_vel_th_, ang_diff));

    //we still want to lay down the footprint of the robot and check if the action is legal
    bool valid_cmd = tc_->checkTrajectory(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw, 
        robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), vel_yaw, 0.0, 0.0, v_theta_samp);

    ROS_DEBUG("Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);

    if(valid_cmd){
      cmd_vel.angular.z = v_theta_samp;
      return true;
    }

    cmd_vel.angular.z = 0.0;
    return false;

  }

  void TrajectoryPlannerROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::recursive_mutex::scoped_lock(odom_lock_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
        base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
  }

  bool TrajectoryPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;

    return true;
  }

  bool TrajectoryPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> local_plan;
    tf::Stamped<tf::Pose> global_pose;
    if(!costmap_ros_->getRobotPose(global_pose))
      return false;

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    //get the global plan in our frame
    if(!transformGlobalPlan(*tf_, global_plan_, costmap_, global_frame_, transformed_plan)){
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    //now we'll prune the plan based on the position of the robot
    if(prune_plan_)
      prunePlan(global_pose, transformed_plan, global_plan_);


    //we also want to clear the robot footprint from the costmap we're using
    costmap_ros_->clearRobotFootprint();

    //make sure to update the costmap we'll use for this cycle
    costmap_ros_->getCostmapCopy(costmap_);

    // Set current velocities from odometry
    geometry_msgs::Twist global_vel;

    odom_lock_.lock();
    global_vel.linear.x = base_odom_.twist.twist.linear.x;
    global_vel.linear.y = base_odom_.twist.twist.linear.y;
    global_vel.angular.z = base_odom_.twist.twist.angular.z;
    odom_lock_.unlock();

    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = robot_base_frame_;

    tf::Stamped<tf::Pose> robot_vel;
    robot_vel.setData(btTransform(tf::createQuaternionFromYaw(global_vel.angular.z), btVector3(global_vel.linear.x, global_vel.linear.y, 0)));
    robot_vel.frame_id_ = robot_base_frame_;
    robot_vel.stamp_ = ros::Time();

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty())
      return false;

    tf::Stamped<tf::Pose> goal_point;
    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.getOrigin().getX();
    double goal_y = goal_point.getOrigin().getY();

    double yaw = tf::getYaw(goal_point.getRotation());

    double goal_th = yaw;

    //check to see if we've reached the goal position
    if(goalPositionReached(global_pose, goal_x, goal_y, xy_goal_tolerance_)){
      //check to see if the goal orientation has been reached
      if(goalOrientationReached(global_pose, goal_th, yaw_goal_tolerance_)){
        //set the velocity command to zero
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
      }
      else {
        tc_->updatePlan(transformed_plan);

        //compute what trajectory to drive along
        Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
        if(!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel))
          return false;
      }

      //publish an empty plan because we've reached our goal position
      publishPlan(transformed_plan, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);
      publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);

      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }

    tc_->updatePlan(transformed_plan);

    //compute what trajectory to drive along
    Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    yaw = tf::getYaw(drive_cmds.getRotation());

    cmd_vel.angular.z = yaw;

    //if we cannot move... tell someone
    if(path.cost_ < 0){
      local_plan.clear();
      publishPlan(transformed_plan, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);
      publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);
      return false;
    }

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i){
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(p_th), tf::Point(p_x, p_y, 0.0)), ros::Time::now(), global_frame_);
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    //publish information to the visualizer
    publishPlan(transformed_plan, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);
    publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);
    return true;
  }

  bool TrajectoryPlannerROS::isGoalReached(){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    {
    boost::recursive_mutex::scoped_lock(odom_lock_);
      base_odom = base_odom_;
    }

    return base_local_planner::isGoalReached(*tf_, global_plan_, *costmap_ros_, global_frame_, base_odom, 
        rot_stopped_velocity_, trans_stopped_velocity_, xy_goal_tolerance_, yaw_goal_tolerance_);
  }
};
