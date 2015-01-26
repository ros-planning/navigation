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
#include <move_slow_and_clear/move_slow_and_clear.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/obstacle_layer.h>

PLUGINLIB_EXPORT_CLASS(move_slow_and_clear::MoveSlowAndClear, nav_core::RecoveryBehavior)

namespace move_slow_and_clear
{
  MoveSlowAndClear::MoveSlowAndClear():global_costmap_(NULL), local_costmap_(NULL), 
                                       initialized_(false), remove_limit_thread_(NULL), limit_set_(false){}

  MoveSlowAndClear::~MoveSlowAndClear()
  {
    delete remove_limit_thread_;
  }

  void MoveSlowAndClear::initialize (std::string n, tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* global_costmap,
      costmap_2d::Costmap2DROS* local_costmap)
  {
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    ros::NodeHandle private_nh_("~/" + n);
    private_nh_.param("clearing_distance", clearing_distance_, 0.5);
    private_nh_.param("limited_trans_speed", limited_trans_speed_, 0.25);
    private_nh_.param("limited_rot_speed", limited_rot_speed_, 0.45);
    private_nh_.param("limited_distance", limited_distance_, 0.3);

    std::string planner_namespace;
    private_nh_.param("planner_namespace", planner_namespace, std::string("DWAPlannerROS"));
    planner_nh_ = ros::NodeHandle("~/" + planner_namespace);
    planner_dynamic_reconfigure_service_ = planner_nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters", true);
    initialized_ = true;
  }

  void MoveSlowAndClear::runBehavior()
  {
    if(!initialized_)
    {
      ROS_ERROR("This recovery behavior has not been initialized, doing nothing.");
      return;
    }
    ROS_WARN("Move slow and clear recovery behavior started.");
    geometry_msgs::PoseStamped global_pose, local_pose;
    global_costmap_->getRobotPose(global_pose);
    local_costmap_->getRobotPose(local_pose);

    std::vector<geometry_msgs::Point> global_poly, local_poly;
    geometry_msgs::Point pt;

    for(int i = -1; i <= 1; i+=2)
    {
      pt.x = global_pose.pose.position.x + i * clearing_distance_;
      pt.y = global_pose.pose.position.y + i * clearing_distance_;
      global_poly.push_back(pt);

      pt.x = global_pose.pose.position.x + i * clearing_distance_;
      pt.y = global_pose.pose.position.y + -1.0 * i * clearing_distance_;
      global_poly.push_back(pt);

      pt.x = local_pose.pose.position.x + i * clearing_distance_;
      pt.y = local_pose.pose.position.y + i * clearing_distance_;
      local_poly.push_back(pt);

      pt.x = local_pose.pose.position.x + i * clearing_distance_;
      pt.y = local_pose.pose.position.y + -1.0 * i * clearing_distance_;
      local_poly.push_back(pt);
    }

    //clear the desired space in the costmaps
    std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = global_costmap_->getLayeredCostmap()->getPlugins();
    for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
            boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
          if(plugin->getName().find("obstacles")!=std::string::npos){
            boost::shared_ptr<costmap_2d::ObstacleLayer> costmap;
            costmap = boost::static_pointer_cast<costmap_2d::ObstacleLayer>(plugin);
            costmap->setConvexPolygonCost(global_poly, costmap_2d::FREE_SPACE);
          }
    }
     
    plugins = local_costmap_->getLayeredCostmap()->getPlugins();
    for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
            boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
          if(plugin->getName().find("obstacles")!=std::string::npos){
            boost::shared_ptr<costmap_2d::ObstacleLayer> costmap;
            costmap = boost::static_pointer_cast<costmap_2d::ObstacleLayer>(plugin);
            costmap->setConvexPolygonCost(local_poly, costmap_2d::FREE_SPACE);
          }
    } 

    //lock... just in case we're already speed limited
    boost::mutex::scoped_lock l(mutex_);

    //get the old maximum speed for the robot... we'll want to set it back
    if(!limit_set_)
    {
      if(!planner_nh_.getParam("max_trans_vel", old_trans_speed_))
      {
        ROS_ERROR("The planner %s, does not have the parameter max_trans_vel", planner_nh_.getNamespace().c_str());
      }

      if(!planner_nh_.getParam("max_rot_vel", old_rot_speed_))
      {
        ROS_ERROR("The planner %s, does not have the parameter max_rot_vel", planner_nh_.getNamespace().c_str());
      }
    }

    //we also want to save our current position so that we can remove the speed limit we impose later on
    speed_limit_pose_ = global_pose;

    //limit the speed of the robot until it moves a certain distance
    setRobotSpeed(limited_trans_speed_, limited_rot_speed_);
    limit_set_ = true;
    distance_check_timer_ = private_nh_.createTimer(ros::Duration(0.1), &MoveSlowAndClear::distanceCheck, this);
  }

  double MoveSlowAndClear::getSqDistance()
  {
    geometry_msgs::PoseStamped global_pose;
    global_costmap_->getRobotPose(global_pose);
    double x1 = global_pose.pose.position.x;
    double y1 = global_pose.pose.position.y;

    double x2 = speed_limit_pose_.pose.position.x;
    double y2 = speed_limit_pose_.pose.position.y;

    return (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
  }

  void MoveSlowAndClear::distanceCheck(const ros::TimerEvent& e)
  {
    if(limited_distance_ * limited_distance_ <= getSqDistance())
    {
      ROS_INFO("Moved far enough, removing speed limit.");
      //have to do this because a system call within a timer cb does not seem to play nice
      if(remove_limit_thread_)
      {
        remove_limit_thread_->join();
        delete remove_limit_thread_;
      }
      remove_limit_thread_ = new boost::thread(boost::bind(&MoveSlowAndClear::removeSpeedLimit, this));

      distance_check_timer_.stop();
    }
  }

  void MoveSlowAndClear::removeSpeedLimit()
  {
    boost::mutex::scoped_lock l(mutex_);
    setRobotSpeed(old_trans_speed_, old_rot_speed_);
    limit_set_ = false;
  }

  void MoveSlowAndClear::setRobotSpeed(double trans_speed, double rot_speed)
  {

    {
      dynamic_reconfigure::Reconfigure vel_reconfigure;
      dynamic_reconfigure::DoubleParameter new_trans;
      new_trans.name = "max_trans_vel";
      new_trans.value = trans_speed;
      vel_reconfigure.request.config.doubles.push_back(new_trans);
      try {
        planner_dynamic_reconfigure_service_.call(vel_reconfigure);
        ROS_INFO_STREAM("Recovery setting trans vel: " << trans_speed);
      }
      catch(...) {
        ROS_ERROR("Something went wrong in the service call to dynamic_reconfigure");
      }
    }
    {
      dynamic_reconfigure::Reconfigure rot_reconfigure;
      dynamic_reconfigure::DoubleParameter new_rot;
      new_rot.name = "max_rot_vel";
      new_rot.value = rot_speed;
      rot_reconfigure.request.config.doubles.push_back(new_rot);
      try {
        planner_dynamic_reconfigure_service_.call(rot_reconfigure);
        ROS_INFO_STREAM("Recovery setting rot vel: " << rot_speed);
      }
      catch(...) {
        ROS_ERROR("Something went wrong in the service call to dynamic_reconfigure");
      }
    }
  }
};
