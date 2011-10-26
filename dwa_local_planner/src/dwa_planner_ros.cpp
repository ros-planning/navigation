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
#include <pluginlib/class_list_macros.h>

#include <dwa_local_planner/dwa_planner_ros.h>
#include <base_local_planner/goal_functions.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(dwa_local_planner, DWAPlannerROS, dwa_local_planner::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_local_planner {
  void DWAPlannerROS::initialize(std::string name, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      tf_ = tf;
      rotating_to_goal_ = false;

      costmap_ros_ = costmap_ros;

      ros::NodeHandle pn("~/" + name);

      g_plan_pub_ = pn.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = pn.advertise<nav_msgs::Path>("local_plan", 1);

      pn.param("prune_plan", prune_plan_, true);

      pn.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
      pn.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);

      pn.param("rot_stopped_vel", rot_stopped_vel_, 1e-2);
      pn.param("trans_stopped_vel", trans_stopped_vel_, 1e-2);

      pn.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);

      //to get odometry information, we need to get a handle to the topic in the global namespace
      ros::NodeHandle gn;
      odom_sub_ = gn.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&DWAPlannerROS::odomCallback, this, _1));

      pn.param("max_rot_vel", max_vel_th_, 1.0);
      min_vel_th_ = -1.0 * max_vel_th_;

      pn.param("min_rot_vel", min_rot_vel_, 0.4);

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, costmap_ros_));

      initialized_ = true;
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }

  void DWAPlannerROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock lock(odom_mutex_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    ROS_DEBUG_NAMED("dwa_local_planner", "In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
        base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
  }

  bool DWAPlannerROS::stopWithAccLimits(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, geometry_msgs::Twist& cmd_vel){
    Eigen::Vector3f acc_lim = dp_->getAccLimits();
    //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
    //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
    double vx = sign(robot_vel.getOrigin().x()) * std::max(0.0, (fabs(robot_vel.getOrigin().x()) - acc_lim[0] * dp_->getSimPeriod()));
    double vy = sign(robot_vel.getOrigin().y()) * std::max(0.0, (fabs(robot_vel.getOrigin().y()) - acc_lim[1] * dp_->getSimPeriod()));

    double vel_yaw = tf::getYaw(robot_vel.getRotation());
    double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim[2] * dp_->getSimPeriod()));

    //we do want to check whether or not the command is valid
    double yaw = tf::getYaw(global_pose.getRotation());
    bool valid_cmd = dp_->checkTrajectory(Eigen::Vector3f(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw),
                                          Eigen::Vector3f(vx, vy, vth));

    //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
    if(valid_cmd){
      ROS_DEBUG_NAMED("dwa_local_planner", "Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
      cmd_vel.linear.x = vx;
      cmd_vel.linear.y = vy;
      cmd_vel.angular.z = vth;
      return true;
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  bool DWAPlannerROS::rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel){
    Eigen::Vector3f acc_lim = dp_->getAccLimits();
    double yaw = tf::getYaw(global_pose.getRotation());
    double vel_yaw = tf::getYaw(robot_vel.getRotation());
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

    double v_theta_samp = ang_diff > 0.0 ? std::min(max_vel_th_,
        std::max(min_rot_vel_, ang_diff)) : std::max(min_vel_th_,
        std::min(-1.0 * min_rot_vel_, ang_diff));

    //take the acceleration limits of the robot into account
    double max_acc_vel = fabs(vel_yaw) + acc_lim[2] * dp_->getSimPeriod();
    double min_acc_vel = fabs(vel_yaw) - acc_lim[2] * dp_->getSimPeriod();

    v_theta_samp = sign(v_theta_samp) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

    //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
    double max_speed_to_stop = sqrt(2 * acc_lim[2] * fabs(ang_diff)); 

    v_theta_samp = sign(v_theta_samp) * std::min(max_speed_to_stop, fabs(v_theta_samp));

    if(fabs(v_theta_samp) < min_rot_vel_)
      v_theta_samp = sign(v_theta_samp) * min_rot_vel_;

    //we still want to lay down the footprint of the robot and check if the action is legal
    bool valid_cmd = dp_->checkTrajectory(Eigen::Vector3f(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw),
                                          Eigen::Vector3f( 0.0, 0.0, v_theta_samp));

    ROS_DEBUG_NAMED("dwa_local_planner", "Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);

    if(valid_cmd){
      cmd_vel.angular.z = v_theta_samp;
      return true;
    }

    cmd_vel.angular.z = 0.0;
    return false;

  }

  bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> local_plan;
    tf::Stamped<tf::Pose> global_pose;
    if(!costmap_ros_->getRobotPose(global_pose))
      return false;

    costmap_2d::Costmap2D costmap;
    costmap_ros_->getCostmapCopy(costmap);
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    //get the global plan in our frame
    if(!base_local_planner::transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, costmap_ros_->getGlobalFrameID(), transformed_plan)){
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    //now we'll prune the plan based on the position of the robot
    if(prune_plan_)
      base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);


    //we also want to clear the robot footprint from the costmap we're using
    costmap_ros_->clearRobotFootprint();

    // Set current velocities from odometry
    geometry_msgs::Twist global_vel;

    {
      boost::mutex::scoped_lock lock(odom_mutex_);
      global_vel.linear.x = base_odom_.twist.twist.linear.x;
      global_vel.linear.y = base_odom_.twist.twist.linear.y;
      global_vel.angular.z = base_odom_.twist.twist.angular.z;
    }

    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = costmap_ros_->getBaseFrameID();

    tf::Stamped<tf::Pose> robot_vel;
    robot_vel.setData(btTransform(tf::createQuaternionFromYaw(global_vel.angular.z), btVector3(global_vel.linear.x, global_vel.linear.y, 0)));
    robot_vel.frame_id_ = costmap_ros_->getBaseFrameID();
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
    if(base_local_planner::goalPositionReached(global_pose, goal_x, goal_y, xy_goal_tolerance_) || xy_tolerance_latch_){
      //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
      //just rotate in place
      if(latch_xy_goal_tolerance_)
        xy_tolerance_latch_ = true;

      //check to see if the goal orientation has been reached
      if(base_local_planner::goalOrientationReached(global_pose, goal_th, yaw_goal_tolerance_)){
        //set the velocity command to zero
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        rotating_to_goal_ = false;
        xy_tolerance_latch_ = false;
      }
      else {
        //we need to call the next two lines to make sure that the dwa
        //planner updates its path distance and goal distance grids
        dp_->updatePlan(transformed_plan);
        base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);

        //copy over the odometry information
        nav_msgs::Odometry base_odom;
        {
          boost::mutex::scoped_lock lock(odom_mutex_);
          base_odom = base_odom_;
        }

        //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
        if(!rotating_to_goal_ && !base_local_planner::stopped(base_odom, rot_stopped_vel_, trans_stopped_vel_)){
          if(!stopWithAccLimits(global_pose, robot_vel, cmd_vel))
            return false;
        }
        //if we're stopped... then we want to rotate to goal
        else{
          //set this so that we know its OK to be moving
          rotating_to_goal_ = true;
          if(!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel))
            return false;
        }
      }

      //publish an empty plan because we've reached our goal position
      base_local_planner::publishPlan(transformed_plan, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);
      base_local_planner::publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);

      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }

    ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());
    dp_->updatePlan(transformed_plan);

    //compute what trajectory to drive along
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
    //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

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
      ROS_DEBUG_NAMED("dwa_local_planner", 
          "The dwa local planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
      local_plan.clear();
      base_local_planner::publishPlan(transformed_plan, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);
      base_local_planner::publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);
      return false;
    }

    ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                    cmd_vel.linear.x, cmd_vel.linear.y, yaw);

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i){
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(p_th), tf::Point(p_x, p_y, 0.0)), ros::Time::now(), costmap_ros_->getGlobalFrameID());
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    //publish information to the visualizer
    base_local_planner::publishPlan(transformed_plan, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);
    base_local_planner::publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);
    return true;
  }

  bool DWAPlannerROS::isGoalReached(){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    {
      boost::mutex::scoped_lock lock(odom_mutex_);
      base_odom = base_odom_;
    }

    return base_local_planner::isGoalReached(*tf_, global_plan_, *costmap_ros_, costmap_ros_->getGlobalFrameID(), base_odom, 
        rot_stopped_vel_, trans_stopped_vel_, xy_goal_tolerance_, yaw_goal_tolerance_);
  }

  bool DWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;

    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    xy_tolerance_latch_ = false;

    return true;
  }

};
