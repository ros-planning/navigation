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
*         David V. Lu!!
*********************************************************************/
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/layered_costmap_ros.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>

using namespace std;

namespace costmap_2d {
  LayeredCostmapROS::LayeredCostmapROS(std::string name, tf::TransformListener& tf) :
                    layered_costmap_(NULL),
                    name_(name), tf_(tf), map_update_thread_(NULL),
                    plugin_loader_("layered_costmap", "layered_costmap::CostmapPlugin") {
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle g_nh;

    // get our tf prefix
    ros::NodeHandle prefix_nh;
    tf_prefix_ = tf::getPrefixParam(prefix_nh);

    // get two frames
    private_nh.param("global_frame", global_frame_, std::string("/map"));
    // make sure that we set the global frame appropriately based on the tf_prefix
    global_frame_ = tf::resolve(tf_prefix_, global_frame_);

    private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    // make sure that we set the base frame appropriately based on the tf_prefix
    robot_base_frame_ = tf::resolve(tf_prefix_, robot_base_frame_);

    ros::Time last_error = ros::Time::now();
    std::string tf_error;
    // we need to make sure that the transform between the robot base frame and the global frame is available
    while (ros::ok() && !tf_.waitForTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), ros::Duration(0.01), &tf_error)) {
      ros::spinOnce();
      if (last_error + ros::Duration(5.0) < ros::Time::now()) {
        ROS_WARN("Waiting on transform from %s to %s to become available before running costmap, tf error: %s",
            robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
        last_error = ros::Time::now();
      }
    }

    private_nh.param("transform_tolerance", transform_tolerance_, 0.3);

    //private_nh.param("track_unknown_space", track_unknown_space_, false);


    bool rolling_window;
    // check if we want a rolling window version of the costmap
    private_nh.param("rolling_window", rolling_window, false);
    
    layered_costmap_ = new LayeredCostmap(global_frame_, rolling_window);

    // find size parameters
    double map_width_meters, map_height_meters, resolution, origin_x, origin_y;
    private_nh.param("width", map_width_meters, 10.0);
    private_nh.param("height", map_height_meters, 10.0);
    private_nh.param("resolution", resolution, 0.05);
    private_nh.param("origin_x", origin_x, 0.0);
    private_nh.param("origin_y", origin_y, 0.0);
    layered_costmap_->resizeMap( (unsigned int)(map_width_meters  / resolution),
               (unsigned int)(map_height_meters / resolution),
               resolution, origin_x, origin_y);

    if (private_nh.hasParam("plugins")) {
        XmlRpc::XmlRpcValue my_list;
        private_nh.getParam("plugins", my_list);
        for (int32_t i = 0; i < my_list.size(); ++i) {
            std::string pname = static_cast<std::string>(my_list[i]["name"]);
            std::string type = static_cast<std::string>(my_list[i]["type"]);
            ROS_INFO("Using plugin \"%s\"", pname.c_str());

            boost::shared_ptr<CostmapPlugin> plugin = plugin_loader_.createInstance(type);
            layered_costmap_->addPlugin(plugin);
            plugin->initialize(layered_costmap_, name + "/" + pname);
        }
    } else {
        ROS_INFO("No plugins");
    }

    double map_publish_frequency;
    private_nh.param("publish_frequency", map_publish_frequency, 0.0);

    // create a thread to handle updating the map
    stop_updates_ = false;
    initialized_ = true;
    stopped_ = false;
    map_update_thread_shutdown_ = false;
    double map_update_frequency;
    private_nh.param("update_frequency", map_update_frequency, 5.0);
    map_update_thread_ = new boost::thread(boost::bind(&LayeredCostmapROS::mapUpdateLoop, this, map_update_frequency));

    // Create a time r to check if the robot is moving
    robot_stopped_ = false;
    timer_ = private_nh.createTimer(ros::Duration(.1), &LayeredCostmapROS::movementCB, this);
  }

  LayeredCostmapROS::~LayeredCostmapROS() {
    map_update_thread_shutdown_ = true;
    if (map_update_thread_ != NULL) {
      map_update_thread_->join();
      delete map_update_thread_;
    }
  }
  
  void LayeredCostmapROS::movementCB(const ros::TimerEvent &event) {
    //don't allow configuration to happen while this check occurs
    //boost::recursive_mutex::scoped_lock mcl(configuration_mutex_);

    tf::Stamped<tf::Pose> new_pose;

    if(!getRobotPose(new_pose)){
      ROS_WARN_THROTTLE(1.0, "Could not get robot pose, cancelling reconfiguration");
      robot_stopped_ = false;
    }
    //make sure that the robot is not moving 
    else if(fabs((old_pose_.getOrigin() - new_pose.getOrigin()).length()) < 1e-3 && fabs(old_pose_.getRotation().angle(new_pose.getRotation())) < 1e-3)
    {
      old_pose_ = new_pose;
      robot_stopped_ = true;
    }
    else
    {
      old_pose_ = new_pose; 
      robot_stopped_ = false;
    }
  }

  void LayeredCostmapROS::mapUpdateLoop(double frequency) {
    // the user might not want to run the loop every cycle
    if (frequency == 0.0)
      return;

    ros::NodeHandle nh;
    ros::Rate r(frequency);
    while (nh.ok() && !map_update_thread_shutdown_) {
      struct timeval start, end;
      double start_t, end_t, t_diff;
      gettimeofday(&start, NULL);
      if (!stop_updates_) {
        //get global pose
         tf::Stamped<tf::Pose> pose;
         if(getRobotPose(pose)){
            layered_costmap_->updateMap(pose.getOrigin().x(), pose.getOrigin().y(), tf::getYaw(pose.getRotation()));
            initialized_ = true;
         }
      }
      gettimeofday(&end, NULL);
      start_t = start.tv_sec + double(start.tv_usec) / 1e6;
      end_t = end.tv_sec + double(end.tv_usec) / 1e6;
      t_diff = end_t - start_t;
      ROS_DEBUG("Map update time: %.9f", t_diff);

      r.sleep();
      // make sure to sleep for the remainder of our cycle time
      if (r.cycleTime() > ros::Duration(1 / frequency))
        ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency, r.cycleTime().toSec());
    }
  }


  void LayeredCostmapROS::start() {
    std::vector<boost::shared_ptr<CostmapPlugin> >* plugins = layered_costmap_->getPlugins();
    // check if we're stopped or just paused
    if (stopped_) {
      // if we're stopped we need to re-subscribe to topics
      for (vector<boost::shared_ptr<CostmapPlugin> >::iterator plugin = plugins->begin(); plugin != plugins->end(); ++plugin) {
          (*plugin)->activate();
      }
      stopped_ = false;
    }
    stop_updates_ = false;

    // block until the costmap is re-initialized.. meaning one update cycle has run
    ros::Rate r(100.0);
    while (ros::ok() && !initialized_)
      r.sleep();
  }

  void LayeredCostmapROS::stop() {
    stop_updates_ = true;
    std::vector<boost::shared_ptr<CostmapPlugin> >* plugins = layered_costmap_->getPlugins();
    // unsubscribe from topics
    for (vector<boost::shared_ptr<CostmapPlugin> >::iterator plugin = plugins->begin(); plugin != plugins->end(); ++plugin) {
      (*plugin)->deactivate();
    }
    initialized_ = false;
    stopped_ = true;
  }
  
  void LayeredCostmapROS::pause() {
    stop_updates_ = true;
    initialized_ = false;
  }

  void LayeredCostmapROS::resume() {
    stop_updates_ = false;

    // block until the costmap is re-initialized.. meaning one update cycle has run
    ros::Rate r(100.0);
    while (!initialized_)
      r.sleep();
  }
  
   bool LayeredCostmapROS::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const {

    global_pose.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    //get the global pose of the robot
    try{
      tf_.transformPose(global_frame_, robot_pose, global_pose);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    // check global_pose timeout
    if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_) {
      ROS_WARN_THROTTLE(1.0, "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
          current_time.toSec() ,global_pose.stamp_.toSec() ,transform_tolerance_);
      return false;
    }

    return true;
  }


};  // namespace layered_costmap
