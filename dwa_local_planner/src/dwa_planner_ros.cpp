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
#include <Eigen/Core>
#include <cmath>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_DECLARE_CLASS(dwa_local_planner, DWAPlannerROS, dwa_local_planner::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_local_planner {

  void DWAPlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if(! isInitialized()) {
      AbstractMoveToRotateLocalPlanner::initialize(name, tf, costmap_ros);
      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(planner_util_.getName(), planner_util_.getCostmapRos()));

    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }



  bool DWAPlannerROS::stopWithAccLimits(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, geometry_msgs::Twist& cmd_vel) {
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
                                          Eigen::Vector3f(vx, vy, vth),
                                          planner_util_.getCurrentLimits());

    //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
    if(valid_cmd){
      ROS_DEBUG_NAMED("dwa_local_planner", "Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
      cmd_vel.linear.x = vx;
      cmd_vel.linear.y = vy;
      cmd_vel.angular.z = vth;
      return true;
    }
    ROS_WARN("Stopping cmd invalid");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  bool DWAPlannerROS::rotateToGoal(
      const tf::Stamped<tf::Pose>& global_pose,
      const tf::Stamped<tf::Pose>& robot_vel,
      double goal_th, geometry_msgs::Twist& cmd_vel){
    Eigen::Vector3f acc_lim = dp_->getAccLimits();
    double yaw = tf::getYaw(global_pose.getRotation());
    double vel_yaw = tf::getYaw(robot_vel.getRotation());
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    double ang_diff = angles::shortest_angular_distance(yaw, goal_th);
    double v_theta_samp = 0;
    base_local_planner::LocalPlannerLimitsConfig limits = planner_util_.getCurrentLimits();

    v_theta_samp = std::min(limits.max_rot_vel, std::max(limits.min_rot_vel, fabs(ang_diff)));

    //take the acceleration limits of the robot into account
    double max_acc_vel = fabs(vel_yaw) + acc_lim[2] * dp_->getSimPeriod();
    double min_acc_vel = fabs(vel_yaw) - acc_lim[2] * dp_->getSimPeriod();

    v_theta_samp = std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

    //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
    double max_speed_to_stop = sqrt(2 * acc_lim[2] * fabs(ang_diff));
    v_theta_samp = std::min(max_speed_to_stop, fabs(v_theta_samp));

    v_theta_samp = std::min(limits.max_rot_vel, std::max(limits.min_rot_vel, v_theta_samp));

    if (ang_diff < 0) {
      v_theta_samp = - v_theta_samp;
    }

    //we still want to lay down the footprint of the robot and check if the action is legal
    bool valid_cmd = dp_->checkTrajectory(Eigen::Vector3f(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw),
                                          Eigen::Vector3f( 0.0, 0.0, v_theta_samp),
                                          limits);

    if (valid_cmd) {
      ROS_DEBUG_NAMED("dwa_local_planner", "Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);
      cmd_vel.angular.z = v_theta_samp;
      return true;
    }
    ROS_WARN("Rotation cmd invalid");
    cmd_vel.angular.z = 0.0;
    return false;

  }

  void DWAPlannerROS::updateDataPassive(tf::Stamped<tf::Pose>& global_pose) {
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if ( ! planner_util_.getLocalPlan(global_pose, transformed_plan)) {
      return;
    }
    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) {
      return;
    }
    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = planner_util_.getCostmapRos()->getBaseFrameID();
    //we need to call the next two lines to make sure that the dwa
    //planner updates its path distance and goal distance grids
    dp_->updatePlan(transformed_plan);
    dp_->findBestPath(global_pose, robot_vel, drive_cmds, planner_util_.getCurrentLimits());
  }

  bool DWAPlannerROS::doComputeVelocityCommands(tf::Stamped<tf::Pose> &global_pose, geometry_msgs::Twist& cmd_vel) {
    if(! isInitialized()){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if ( ! planner_util_.getLocalPlan(global_pose, transformed_plan)) {
      return false;
    }

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) {
      return false;
    }
    ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());
    dp_->updatePlan(transformed_plan);

    //we also want to clear the robot footprint from the costmap we're using
    planner_util_.getCostmapRos()->clearRobotFootprint();


    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = planner_util_.getCostmapRos()->getBaseFrameID();

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */



    //compute what trajectory to drive along
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds, planner_util_.getCurrentLimits());
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
    cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if(path.cost_ < 0) {
      ROS_DEBUG_NAMED("dwa_local_planner", 
          "The dwa local planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
      local_plan.clear();
      planner_util_.publishGlobalPlan(transformed_plan);
      planner_util_.publishLocalPlan(local_plan);
      return false;
    }

    ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      tf::Stamped<tf::Pose> p =
    		  tf::Stamped<tf::Pose>(tf::Pose(
    				  tf::createQuaternionFromYaw(p_th),
    				  tf::Point(p_x, p_y, 0.0)),
    				  ros::Time::now(),
    				  planner_util_.getCostmapRos()->getGlobalFrameID());
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    //publish information to the visualizer
    planner_util_.publishGlobalPlan(transformed_plan);
    planner_util_.publishLocalPlan(local_plan);
    return true;
  }



};
