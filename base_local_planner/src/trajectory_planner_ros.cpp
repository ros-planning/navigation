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

using namespace std;
using namespace costmap_2d;

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_REGISTER_CLASS(TrajectoryPlannerROS, base_local_planner::TrajectoryPlannerROS, nav_core::BaseLocalPlanner)

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
      double max_vel_x, min_vel_x, max_vel_th, min_vel_th;
      double backup_vel;
      string world_model_type;

      //initialize the copy of the costmap the controller will use
      costmap_ros_->getCostmapCopy(costmap_);

      ros::NodeHandle ros_node("~/" + name);

      footprint_pub_ = ros_node.advertise<geometry_msgs::PolygonStamped>("robot_footprint", 1);
      g_plan_pub_ = ros_node.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = ros_node.advertise<nav_msgs::Path>("local_plan", 1);

      global_frame_ = costmap_ros_->getGlobalFrameID();
      robot_base_frame_ = costmap_ros_->getBaseFrameID();
      ros_node.param("prune_plan", prune_plan_, true);

      ros_node.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
      ros_node.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);

      //to get odometery information, we need to get a handle to the topic in the global namespace of the node
      ros::NodeHandle global_node;
      odom_sub_ = global_node.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&TrajectoryPlannerROS::odomCallback, this, _1));

      //we'll get the parameters for the robot radius from the costmap we're associated with
      inscribed_radius_ = costmap_ros_->getInscribedRadius();
      circumscribed_radius_ = costmap_ros_->getCircumscribedRadius();
      inflation_radius_ = costmap_ros_->getInflationRadius();

      ros_node.param("acc_lim_x", acc_lim_x, 2.5);
      ros_node.param("acc_lim_y", acc_lim_y, 2.5);
      ros_node.param("acc_lim_th", acc_lim_theta, 3.2);
      ros_node.param("sim_time", sim_time, 1.0);
      ros_node.param("sim_granularity", sim_granularity, 0.025);
      ros_node.param("vx_samples", vx_samples, 3);
      ros_node.param("vtheta_samples", vtheta_samples, 20);
      ros_node.param("path_distance_bias", pdist_scale, 0.6);
      ros_node.param("goal_distance_bias", gdist_scale, 0.8);
      ros_node.param("occdist_scale", occdist_scale, 0.2);
      ros_node.param("heading_lookahead", heading_lookahead, 0.325);
      ros_node.param("oscillation_reset_dist", oscillation_reset_dist, 0.05);
      ros_node.param("escape_reset_dist", escape_reset_dist, 0.10);
      ros_node.param("escape_reset_theta", escape_reset_theta, M_PI_4);
      ros_node.param("holonomic_robot", holonomic_robot, true);
      ros_node.param("max_vel_x", max_vel_x, 0.5);
      ros_node.param("min_vel_x", min_vel_x, 0.1);
      ros_node.param("max_vel_th", max_vel_th, 1.0);
      ros_node.param("min_vel_th", min_vel_th, -1.0);
      ros_node.param("min_in_place_vel_th", min_in_place_vel_th_, 0.4);
      ros_node.param("backup_vel", backup_vel, -0.1);
      ros_node.param("world_model", world_model_type, string("costmap")); 
      ros_node.param("dwa", dwa, true);
      ros_node.param("heading_scoring", heading_scoring, false);
      ros_node.param("heading_scoring_timestep", heading_scoring_timestep, 0.1);
      ros_node.param("simple_attractor", simple_attractor, false);

      //parameters for using the freespace controller
      double min_pt_separation, max_obstacle_height, grid_resolution;
      ros_node.param("point_grid/max_sensor_range", max_sensor_range_, 2.0);
      ros_node.param("point_grid/min_pt_separation", min_pt_separation, 0.01);
      ros_node.param("point_grid/max_obstacle_height", max_obstacle_height, 2.0);
      ros_node.param("point_grid/grid_resolution", grid_resolution, 0.2);

      ROS_ASSERT_MSG(world_model_type == "costmap", "At this time, only costmap world models are supported by this controller");
      world_model_ = new CostmapModel(costmap_); 

      tc_ = new TrajectoryPlanner(*world_model_, costmap_, costmap_ros_->getRobotFootprint(), inscribed_radius_, circumscribed_radius_,
          acc_lim_x, acc_lim_y, acc_lim_theta, sim_time, sim_granularity, vx_samples, vtheta_samples, pdist_scale,
          gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta, holonomic_robot,
          max_vel_x, min_vel_x, max_vel_th, min_vel_th, min_in_place_vel_th_, backup_vel,
          dwa, heading_scoring, heading_scoring_timestep, simple_attractor);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }

  TrajectoryPlannerROS::~TrajectoryPlannerROS(){
    if(tc_ != NULL)
      delete tc_;

    if(world_model_ != NULL)
      delete world_model_;
  }

  vector<geometry_msgs::Point> TrajectoryPlannerROS::drawFootprint(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return vector<geometry_msgs::Point>();
    }
    return tc_->drawFootprint(x_i, y_i, theta_i);
  }

  bool TrajectoryPlannerROS::stopped(){
    boost::recursive_mutex::scoped_lock(odom_lock_);
    return abs(base_odom_.twist.twist.angular.z) <= rot_stopped_velocity_ 
      && abs(base_odom_.twist.twist.linear.x) <= trans_stopped_velocity_
      && abs(base_odom_.twist.twist.linear.y) <= trans_stopped_velocity_;
  }

  double TrajectoryPlannerROS::distance(double x1, double y1, double x2, double y2){
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

  bool TrajectoryPlannerROS::goalPositionReached(const tf::Stamped<tf::Pose>& global_pose, double goal_x, double goal_y){
    double dist = distance(global_pose.getOrigin().x(), global_pose.getOrigin().y(), goal_x, goal_y);
    return fabs(dist) <= xy_goal_tolerance_;
  }

  bool TrajectoryPlannerROS::goalOrientationReached(const tf::Stamped<tf::Pose>& global_pose, double goal_th){
    double useless_pitch, useless_roll, yaw;
    global_pose.getBasis().getEulerZYX(yaw, useless_pitch, useless_roll);
    return fabs(angles::shortest_angular_distance(yaw, goal_th)) <= yaw_goal_tolerance_;
  }

  bool TrajectoryPlannerROS::rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel){
    double uselessPitch, uselessRoll, yaw, vel_yaw;
    global_pose.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
    robot_vel.getBasis().getEulerZYX(vel_yaw, uselessPitch, uselessRoll);
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    double ang_diff = angles::shortest_angular_distance(yaw, goal_th);
    double v_theta_samp = ang_diff > 0.0 ? std::max(min_in_place_vel_th_, ang_diff) : std::min(-1.0 * min_in_place_vel_th_, ang_diff);

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
    boost::recursive_mutex::scoped_lock(odom_lock_);
    try
    {
      tf::Stamped<btVector3> v_in(btVector3(msg->twist.twist.linear.x, msg->twist.twist.linear.y, 0), ros::Time(), msg->header.frame_id), v_out;
      tf_->transformVector(robot_base_frame_, ros::Time(), v_in, msg->header.frame_id, v_out);
      base_odom_.twist.twist.linear.x = v_in.x();
      base_odom_.twist.twist.linear.y = v_in.y();
      base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    }
    catch(tf::LookupException& ex)
    {
      ROS_DEBUG("No odom->base Tx yet: %s", ex.what());
    }
    catch(tf::ConnectivityException& ex)
    {
      ROS_DEBUG("No odom->base Tx yet: %s\n", ex.what());
    }
    catch(tf::ExtrapolationException& ex)
    {
      ROS_DEBUG("Extrapolation exception");
    }
  }

  bool TrajectoryPlannerROS::isGoalReached(){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    const geometry_msgs::PoseStamped& plan_goal_pose = global_plan_.back();
    tf::Stamped<tf::Pose> goal_pose;

    try{
      if (!global_plan_.size() > 0)
      {
        ROS_ERROR("Recieved plan with zero length");
        return false;
      }

      tf::Stamped<tf::Transform> transform;
      tf_->lookupTransform(global_frame_, ros::Time(), 
          plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp, 
          plan_goal_pose.header.frame_id, transform);

      poseStampedMsgToTF(plan_goal_pose, goal_pose);
      goal_pose.setData(transform * goal_pose);
      goal_pose.stamp_ = transform.stamp_;
      goal_pose.frame_id_ = global_frame_;

    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (global_plan_.size() > 0)
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame_.c_str(), (unsigned int)global_plan_.size(), global_plan_[0].header.frame_id.c_str());

      return false;
    }

    //we assume the global goal is the last point in the global plan
    double goal_x = goal_pose.getOrigin().getX();
    double goal_y = goal_pose.getOrigin().getY();

    double uselessPitch, uselessRoll, yaw;
    goal_pose.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);

    double goal_th = yaw;

    tf::Stamped<tf::Pose> global_pose;
    if(!costmap_ros_->getRobotPose(global_pose))
      return false;

    //check to see if we've reached the goal position
    if(goalPositionReached(global_pose, goal_x, goal_y)){
      //check to see if the goal orientation has been reached
      if(goalOrientationReached(global_pose, goal_th)){
        //make sure that we're actually stopped before returning success
        if(stopped())
          return true;
      }
    }

    return false;
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

  bool TrajectoryPlannerROS::transformGlobalPlan(std::vector<geometry_msgs::PoseStamped>& transformed_plan){
    const geometry_msgs::PoseStamped& plan_pose = global_plan_[0];

    transformed_plan.clear();

    try{
      if (!global_plan_.size() > 0)
      {
        ROS_ERROR("Recieved plan with zero length");
        return false;
      }

      tf::Stamped<tf::Transform> transform;
      tf_->lookupTransform(global_frame_, ros::Time(), 
          plan_pose.header.frame_id, plan_pose.header.stamp, 
          plan_pose.header.frame_id, transform);


      tf::Stamped<tf::Pose> tf_pose;
      geometry_msgs::PoseStamped newer_pose;

      //we'll look ahead on the path the size of our window
      unsigned int needed_path_length = std::max(costmap_.getSizeInCellsX(), costmap_.getSizeInCellsY());

      for(unsigned int i = 0; i < std::min((unsigned int)global_plan_.size(), needed_path_length); ++i){
        const geometry_msgs::PoseStamped& pose = global_plan_[i];
        poseStampedMsgToTF(pose, tf_pose);
        tf_pose.setData(transform * tf_pose);
        tf_pose.stamp_ = transform.stamp_;
        tf_pose.frame_id_ = global_frame_;
        poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);
      }
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (global_plan_.size() > 0)
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame_.c_str(), (unsigned int)global_plan_.size(), global_plan_[0].header.frame_id.c_str());

      return false;
    }

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
    if(!transformGlobalPlan(transformed_plan)){
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
    btQuaternion qt(global_vel.angular.z, 0, 0);
    robot_vel.setData(btTransform(qt, btVector3(global_vel.linear.x, global_vel.linear.y, 0)));
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

    double uselessPitch, uselessRoll, yaw;
    goal_point.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);

    double goal_th = yaw;

    //check to see if we've reached the goal position
    if(goalPositionReached(global_pose, goal_x, goal_y)){
      //check to see if the goal orientation has been reached
      if(goalOrientationReached(global_pose, goal_th)){
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

      //publish the robot footprint and an empty plan because we've reached our goal position
      publishFootprint(global_pose);
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
    drive_cmds.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);

    cmd_vel.angular.z = yaw;

    //if we cannot move... tell someone
    if(path.cost_ < 0){
      local_plan.clear();
      publishFootprint(global_pose);
      publishPlan(transformed_plan, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);
      publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);
      return false;
    }

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i){
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::Quaternion(p_th, 0.0, 0.0), tf::Point(p_x, p_y, 0.0)), ros::Time::now(), global_frame_);
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    //publish information to the visualizer
    publishFootprint(global_pose);
    publishPlan(transformed_plan, g_plan_pub_, 0.0, 1.0, 0.0, 0.0);
    publishPlan(local_plan, l_plan_pub_, 0.0, 0.0, 1.0, 0.0);
    return true;
  }

  void TrajectoryPlannerROS::prunePlan(const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& global_plan){
    ROS_ASSERT(global_plan.size() >= plan.size());
    std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator global_it = global_plan.begin();
    while(it != plan.end()){
      const geometry_msgs::PoseStamped& w = *it;
      // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
      double x_diff = global_pose.getOrigin().x() - w.pose.position.x;
      double y_diff = global_pose.getOrigin().y() - w.pose.position.y;
      double distance_sq = x_diff * x_diff + y_diff * y_diff;
      if(distance_sq < 1){
        ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose.getOrigin().x(), global_pose.getOrigin().y(), w.pose.position.x, w.pose.position.y);
        break;
      }
      it = plan.erase(it);
      global_it = global_plan.erase(global_it);
    }
  }

  void TrajectoryPlannerROS::publishFootprint(const tf::Stamped<tf::Pose>& global_pose){
    double useless_pitch, useless_roll, yaw;
    global_pose.getBasis().getEulerZYX(yaw, useless_pitch, useless_roll);
    std::vector<geometry_msgs::Point> footprint = drawFootprint(global_pose.getOrigin().x(), global_pose.getOrigin().y(), yaw);

    //create a polygon message for the footprint
    geometry_msgs::PolygonStamped footprint_poly;
    footprint_poly.header.frame_id = global_frame_;
    footprint_poly.header.stamp = ros::Time::now();
    footprint_poly.polygon.set_points_size(footprint.size());
    
    for(unsigned int i = 0; i < footprint.size(); ++i){
      footprint_poly.polygon.points[i].x = footprint[i].x;
      footprint_poly.polygon.points[i].y = footprint[i].y;
      footprint_poly.polygon.points[i].z = footprint[i].z;
    }
    footprint_pub_.publish(footprint_poly);
  }

  void TrajectoryPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub, double r, double g, double b, double a){
    //given an empty path we won't do anything
    if(path.empty())
      return;

    //create a path message
    nav_msgs::Path gui_path;
    gui_path.set_poses_size(path.size());
    gui_path.header.frame_id = global_frame_;
    gui_path.header.stamp = path[0].header.stamp;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i].pose.position.x = path[i].pose.position.x;
      gui_path.poses[i].pose.position.y = path[i].pose.position.y;
      gui_path.poses[i].pose.position.z = path[i].pose.position.z;
    }

    pub.publish(gui_path);
  }
};
