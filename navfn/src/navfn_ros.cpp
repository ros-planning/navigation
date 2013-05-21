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
#include <navfn/navfn_ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_DECLARE_CLASS(navfn, NavfnROS, navfn::NavfnROS, nav_core::BaseGlobalPlanner)

namespace navfn {

  NavfnROS::NavfnROS() 
    : costmap_ros_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {}

  NavfnROS::NavfnROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
    : costmap_ros_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {
      //initialize the planner
      initialize(name, costmap_ros);
  }

  void NavfnROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
      planner_ = boost::shared_ptr<NavFn>(new NavFn(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()));

      ros::NodeHandle private_nh("~/" + name);

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

      private_nh.param("visualize_potential", visualize_potential_, false);

      //if we're going to visualize the potential array we need to advertise
      if(visualize_potential_)
        potarr_pub_.advertise(private_nh, "potential", 1);

      private_nh.param("allow_unknown", allow_unknown_, true);
      private_nh.param("planner_window_x", planner_window_x_, 0.0);
      private_nh.param("planner_window_y", planner_window_y_, 0.0);
      private_nh.param("default_tolerance", default_tolerance_, 0.0);
        
      double costmap_pub_freq;
      private_nh.param("planner_costmap_publish_frequency", costmap_pub_freq, 0.0);

      //get the tf prefix
      ros::NodeHandle prefix_nh;
      tf_prefix_ = tf::getPrefixParam(prefix_nh);

      make_plan_srv_ =  private_nh.advertiseService("make_plan", &NavfnROS::makePlanService, this);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }

  bool NavfnROS::validPointPotential(const geometry_msgs::Point& world_point){
    return validPointPotential(world_point, default_tolerance_);
  }

  bool NavfnROS::validPointPotential(const geometry_msgs::Point& world_point, double tolerance){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    double resolution = costmap_ros_->getCostmap()->getResolution();
    geometry_msgs::Point p;
    p = world_point;

    p.y = world_point.y - tolerance;

    while(p.y <= world_point.y + tolerance){
      p.x = world_point.x - tolerance;
      while(p.x <= world_point.x + tolerance){
        double potential = getPointPotential(p);
        if(potential < POT_HIGH){
          return true;
        }
        p.x += resolution;
      }
      p.y += resolution;
    }

    return false;
  }

  double NavfnROS::getPointPotential(const geometry_msgs::Point& world_point){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return -1.0;
    }

    unsigned int mx, my;
    if(!costmap_ros_->getCostmap()->worldToMap(world_point.x, world_point.y, mx, my))
      return DBL_MAX;

    unsigned int index = my * planner_->nx + mx;
    return planner_->potarr[index];
  }

  bool NavfnROS::computePotential(const geometry_msgs::Point& world_point){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }
    
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

    //make sure to resize the underlying array that Navfn uses
    planner_->setNavArr(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
    planner_->setCostmap(costmap->getCharMap(), true, allow_unknown_);

    unsigned int mx, my;
    if(!costmap->worldToMap(world_point.x, world_point.y, mx, my))
      return false;

    int map_start[2];
    map_start[0] = 0;
    map_start[1] = 0;

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_start);
    planner_->setGoal(map_goal);

    return planner_->calcNavFnDijkstra();
  }

  void NavfnROS::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //set the associated costs in the cost map to be free
    costmap_ros_->getCostmap()->setCost(mx, my, costmap_2d::FREE_SPACE);
  }

  bool NavfnROS::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp){
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = costmap_ros_->getGlobalFrameID();

    return true;
  } 

  void NavfnROS::mapToWorld(double mx, double my, double& wx, double& wy) {
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    wx = costmap->getOriginX() + mx * costmap->getResolution();
    wy = costmap->getOriginY() + my * costmap->getResolution();
  }

  bool NavfnROS::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    return makePlan(start, goal, default_tolerance_, plan);
  }

  bool NavfnROS::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::mutex::scoped_lock lock(mutex_);
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    std::string global_frame = costmap_ros_->getGlobalFrameID();

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)){
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
      return false;
    }

    if(tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)){
      ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
      return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int mx, my;
    if(!costmap->worldToMap(wx, wy, mx, my)){
      ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
      return false;
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
    clearRobotCell(start_pose, mx, my);

#if 0
    {
      static int n = 0;
      static char filename[1000];
      snprintf( filename, 1000, "navfnros-makeplan-costmapB-%04d.pgm", n++ );
      costmap->saveRawMap( std::string( filename ));
    }
#endif

    //make sure to resize the underlying array that Navfn uses
    planner_->setNavArr(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
    planner_->setCostmap(costmap->getCharMap(), true, allow_unknown_);

#if 0
    {
      static int n = 0;
      static char filename[1000];
      snprintf( filename, 1000, "navfnros-makeplan-costmapC-%04d", n++ );
      planner_->savemap( filename );
    }
#endif

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if(!costmap->worldToMap(wx, wy, mx, my)){
      if(tolerance <= 0.0){
        ROS_WARN("The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
        return false;
      }
      mx = 0;
      my = 0;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_goal);
    planner_->setGoal(map_start);

    //bool success = planner_->calcNavFnAstar();
    planner_->calcNavFnDijkstra(true);

    double resolution = costmap->getResolution();
    geometry_msgs::PoseStamped p, best_pose;
    p = goal;

    bool found_legal = false;
    double best_sdist = DBL_MAX;

    p.pose.position.y = goal.pose.position.y - tolerance;

    while(p.pose.position.y <= goal.pose.position.y + tolerance){
      p.pose.position.x = goal.pose.position.x - tolerance;
      while(p.pose.position.x <= goal.pose.position.x + tolerance){
        double potential = getPointPotential(p.pose.position);
        double sdist = sq_distance(p, goal);
        if(potential < POT_HIGH && sdist < best_sdist){
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.pose.position.x += resolution;
      }
      p.pose.position.y += resolution;
    }

    if(found_legal){
      //extract the plan
      if(getPlanFromPotential(best_pose, plan)){
        //make sure the goal we push on has the same timestamp as the rest of the plan
        geometry_msgs::PoseStamped goal_copy = best_pose;
        goal_copy.header.stamp = ros::Time::now();
        plan.push_back(goal_copy);
      }
      else{
        ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
      }
    }

    if (visualize_potential_){
      //publish potential array
      pcl::PointCloud<PotarrPoint> pot_area;
      pot_area.header.frame_id = global_frame;
      pot_area.points.clear();
      pot_area.header.stamp = ros::Time::now();

      PotarrPoint pt;
      float *pp = planner_->potarr;
      double pot_x, pot_y;
      for (unsigned int i = 0; i < (unsigned int)planner_->ny*planner_->nx ; i++)
      {
        if (pp[i] < 10e7)
        {
          mapToWorld(i%planner_->nx, i/planner_->nx, pot_x, pot_y);
          pt.x = pot_x;
          pt.y = pot_y;
          pt.z = pp[i]/pp[planner_->start[1]*planner_->nx + planner_->start[0]]*20;
          pt.pot_value = pp[i];
          pot_area.push_back(pt);
        }
      }
      potarr_pub_.publish(pot_area);
    }

    //publish the plan for visualization purposes
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);

    return !plan.empty();
  }

  void NavfnROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //create a message for the plan 
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if(!path.empty())
    {
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }

  bool NavfnROS::getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }
    
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    std::string global_frame = costmap_ros_->getGlobalFrameID();

    //clear the plan, just in case
    plan.clear();

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)){
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
      return false;
    }

    double wx = goal.pose.position.x;
    double wy = goal.pose.position.y;

    //the potential has already been computed, so we won't update our copy of the costmap
    unsigned int mx, my;
    if(!costmap->worldToMap(wx, wy, mx, my)){
      ROS_WARN("The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
      return false;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_goal);

    planner_->calcPath(costmap->getSizeInCellsX() * 4);

    //extract the plan
    float *x = planner_->getPathX();
    float *y = planner_->getPathY();
    int len = planner_->getPathLen();
    ros::Time plan_time = ros::Time::now();

    for(int i = len - 1; i >= 0; --i){
      //convert the plan to world coordinates
      double world_x, world_y;
      mapToWorld(x[i], y[i], world_x, world_y);

      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame;
      pose.pose.position.x = world_x;
      pose.pose.position.y = world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }

    //publish the plan for visualization purposes
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
    return !plan.empty();
  }
};
