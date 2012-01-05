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
#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <limits>

using namespace std;

namespace costmap_2d {

  double sign(double x){
    return x < 0.0 ? -1.0 : 1.0;
  }

  Costmap2DROS::Costmap2DROS(std::string name, tf::TransformListener& tf) : name_(name), tf_(tf), costmap_(NULL), 
                             map_update_thread_(NULL), costmap_publisher_(NULL), stop_updates_(false), 
                             initialized_(true), stopped_(false), map_update_thread_shutdown_(false), 
                             save_debug_pgm_(false), map_initialized_(false), costmap_initialized_(false), 
                             robot_stopped_(false), setup_(false){
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle g_nh;

    //get our tf prefix
    ros::NodeHandle prefix_nh;
    tf_prefix_ = tf::getPrefixParam(prefix_nh);

    std::string map_type;
    private_nh.param("map_type", map_type, std::string("voxel"));

    private_nh.param("publish_voxel_map", publish_voxel_, false);

    if(publish_voxel_ && map_type == "voxel")
      voxel_pub_ = private_nh.advertise<costmap_2d::VoxelGrid>("voxel_grid", 1);
    else
      publish_voxel_ = false;

    std::string topics_string;
    //get the topics that we'll subscribe to from the parameter server
    private_nh.param("observation_sources", topics_string, std::string(""));
    ROS_INFO("Subscribed to Topics: %s", topics_string.c_str());

    private_nh.param("global_frame", global_frame_, std::string("/map"));
    //make sure that we set the global frame appropriately based on the tf_prefix
    global_frame_ = tf::resolve(tf_prefix_, global_frame_);

    private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    //make sure that we set the base frame appropriately based on the tf_prefix
    robot_base_frame_ = tf::resolve(tf_prefix_, robot_base_frame_);

    //check if the user wants to save pgms of the costmap for debugging
    private_nh.param("save_debug_pgm", save_debug_pgm_, false);

    bool static_map;
    unsigned int map_width, map_height;
    double map_resolution;
    double map_origin_x, map_origin_y;

    private_nh.param("static_map", static_map, true);

    //check if we want a rolling window version of the costmap
    private_nh.param("rolling_window", rolling_window_, false);

    double map_width_meters, map_height_meters;
    std::string map_topic;
    private_nh.param("map_topic", map_topic, std::string("map"));
    private_nh.param("width", map_width_meters, 10.0);
    private_nh.param("height", map_height_meters, 10.0);
    private_nh.param("resolution", map_resolution, 0.05);
    private_nh.param("origin_x", map_origin_x, 0.0);
    private_nh.param("origin_y", map_origin_y, 0.0);
    map_width = (unsigned int)(map_width_meters / map_resolution);
    map_height = (unsigned int)(map_height_meters / map_resolution);

    if(static_map){
      //we'll subscribe to the latched topic that the map server uses
      ROS_INFO("Requesting the map...\n");
      map_sub_ = g_nh.subscribe(map_topic, 1, &Costmap2DROS::incomingMap, this);

      ros::Rate r(1.0);
      while(!map_initialized_ && ros::ok()){
        ros::spinOnce();
        ROS_INFO("Still waiting on map...\n");
        r.sleep();
      }

      //check if the user has set any parameters that will be overwritten
      bool user_map_params = false;
      user_map_params |= private_nh.hasParam("width");
      user_map_params |= private_nh.hasParam("height");
      user_map_params |= private_nh.hasParam("resolution");
      user_map_params |= private_nh.hasParam("origin_x");
      user_map_params |= private_nh.hasParam("origin_y");

      if(user_map_params)
        ROS_WARN("You have set map parameters, but also requested to use the static map. Your parameters will be overwritten by those given by the map server");

      {
        //lock just in case something weird is going on with the compiler or scheduler
        boost::recursive_mutex::scoped_lock lock(map_data_lock_);
        map_width = (unsigned int)map_meta_data_.width;
        map_height = (unsigned int)map_meta_data_.height;
        map_resolution = map_meta_data_.resolution;
        map_origin_x = map_meta_data_.origin.position.x;
        map_origin_y = map_meta_data_.origin.position.y;

        ROS_INFO("Received a %d X %d map at %f m/pix\n",
            map_width, map_height, map_resolution);
      }

    }

    ros::Time last_error = ros::Time::now();
    std::string tf_error;
    //we need to make sure that the transform between the robot base frame and the global frame is available
    while(ros::ok() && !tf_.waitForTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), ros::Duration(0.01), &tf_error)){
      ros::spinOnce();
      if(last_error + ros::Duration(5.0) < ros::Time::now()){
        ROS_WARN("Waiting on transform from %s to %s to become available before running costmap, tf error: %s", 
            robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
        last_error = ros::Time::now();
      }
    }

    private_nh.param("transform_tolerance", transform_tolerance_, 0.3);

    //now we need to split the topics based on whitespace which we can use a stringstream for
    std::stringstream ss(topics_string);

    double raytrace_range = 3.0;
    double obstacle_range = 2.5;

    std::string source;
    while(ss >> source){
      ros::NodeHandle source_node(private_nh, source);
      //get the parameters for the specific topic
      double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
      std::string topic, sensor_frame, data_type;
      source_node.param("topic", topic, source);
      source_node.param("sensor_frame", sensor_frame, std::string(""));
      source_node.param("observation_persistence", observation_keep_time, 0.0);
      source_node.param("expected_update_rate", expected_update_rate, 0.0);
      source_node.param("data_type", data_type, std::string("PointCloud"));
      source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
      source_node.param("max_obstacle_height", max_obstacle_height, 2.0);

      if(!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan")){
        ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
        throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
      }


      bool clearing, marking;
      source_node.param("clearing", clearing, false);
      source_node.param("marking", marking, true);

      std::string raytrace_range_param_name, obstacle_range_param_name;
      double source_raytrace_range, source_obstacle_range;

      //get the obstacle range for the sensor
      if(!source_node.searchParam("obstacle_range", obstacle_range_param_name))
        source_obstacle_range = 2.5;
      else
        source_node.param(obstacle_range_param_name, source_obstacle_range, 2.5);

      //get the raytrace range for the sensor
      if(!source_node.searchParam("raytrace_range", raytrace_range_param_name))
        source_raytrace_range = 3.0;
      else
        source_node.param(raytrace_range_param_name, source_raytrace_range, 3.0);


      //keep track of the maximum raytrace range for the costmap to be able to inflate efficiently
      raytrace_range = std::max(raytrace_range, source_raytrace_range);
      obstacle_range = std::max(obstacle_range, source_obstacle_range);

      ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(), sensor_frame.c_str());

      //create an observation buffer
      observation_buffers_.push_back(boost::shared_ptr<ObservationBuffer>(new ObservationBuffer(topic, observation_keep_time, 
              expected_update_rate, min_obstacle_height, max_obstacle_height, source_obstacle_range, source_raytrace_range, tf_, global_frame_, sensor_frame, transform_tolerance_)));

      //check if we'll add this buffer to our marking observation buffers
      if(marking)
        marking_buffers_.push_back(observation_buffers_.back());

      //check if we'll also add this buffer to our clearing observation buffers
      if(clearing)
        clearing_buffers_.push_back(observation_buffers_.back());

      ROS_DEBUG("Created an observation buffer for source %s, topic %s, global frame: %s, expected update rate: %.2f, observation persistence: %.2f", 
          source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

      //create a callback for the topic
      if(data_type == "LaserScan"){
        boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan> > sub(
              new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

        boost::shared_ptr<tf::MessageFilter<sensor_msgs::LaserScan> > filter(
            new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, tf_, global_frame_, 50));
        filter->registerCallback(boost::bind(&Costmap2DROS::laserScanCallback, this, _1, observation_buffers_.back()));

        observation_subscribers_.push_back(sub);
        observation_notifiers_.push_back(filter);

        observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
      }
      else if(data_type == "PointCloud"){
        boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud> > sub(
              new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));

        boost::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud> > filter(
            new tf::MessageFilter<sensor_msgs::PointCloud>(*sub, tf_, global_frame_, 50));
        filter->registerCallback(boost::bind(&Costmap2DROS::pointCloudCallback, this, _1, observation_buffers_.back()));

        observation_subscribers_.push_back(sub);
        observation_notifiers_.push_back(filter);
      }
      else{
        boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2> > sub(
              new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

        boost::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud2> > filter(
            new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub, tf_, global_frame_, 50));
        filter->registerCallback(boost::bind(&Costmap2DROS::pointCloud2Callback, this, _1, observation_buffers_.back()));

        observation_subscribers_.push_back(sub);
        observation_notifiers_.push_back(filter);
      }

      if(sensor_frame != ""){
        std::vector<std::string> target_frames;
        target_frames.push_back(global_frame_);
        target_frames.push_back(sensor_frame);
        observation_notifiers_.back()->setTargetFrames(target_frames);
      }

    }

    double inscribed_radius, circumscribed_radius, inflation_radius;
    inscribed_radius = 0.46;

    if(private_nh.hasParam("robot_radius")){
      private_nh.param("robot_radius", inscribed_radius, 0.46);
    }

    circumscribed_radius = inscribed_radius;
    private_nh.param("inflation_radius", inflation_radius, 0.55);

    //load the robot footprint from the parameter server if its available in the global namespace
    footprint_spec_ = loadRobotFootprint(private_nh, inscribed_radius, circumscribed_radius);

    if(inscribed_radius > inflation_radius || circumscribed_radius > inflation_radius){
      ROS_WARN("You have set an inflation radius that is less than the inscribed and circumscribed radii of the robot. This is dangerous and could casue the robot to hit obstacles. Please change your inflation radius setting appropraitely.");
    }

    if(footprint_spec_.size() > 2){
      //now we need to compute the inscribed/circumscribed radius of the robot from the footprint specification
      double min_dist = std::numeric_limits<double>::max();
      double max_dist = 0.0;

      for(unsigned int i = 0; i < footprint_spec_.size() - 1; ++i){
        //check the distance from the robot center point to the first vertex
        double vertex_dist = distance(0.0, 0.0, footprint_spec_[i].x, footprint_spec_[i].y);
        double edge_dist = distanceToLine(0.0, 0.0, footprint_spec_[i].x, footprint_spec_[i].y, footprint_spec_[i+1].x, footprint_spec_[i+1].y);
        min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
        max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
      }

      //we also need to do the last vertex and the first vertex
      double vertex_dist = distance(0.0, 0.0, footprint_spec_.back().x, footprint_spec_.back().y);
      double edge_dist = distanceToLine(0.0, 0.0, footprint_spec_.back().x, footprint_spec_.back().y, footprint_spec_.front().x, footprint_spec_.front().y);
      min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
      max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));

      inscribed_radius = min_dist;
      circumscribed_radius = max_dist;
    }

    double max_obstacle_height;
    private_nh.param("max_obstacle_height", max_obstacle_height, 2.0);

    double cost_scale;
    private_nh.param("cost_scaling_factor", cost_scale, 10.0);

    int temp_lethal_threshold, temp_unknown_cost_value;
    private_nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
    private_nh.param("unknown_cost_value", temp_unknown_cost_value, int(0));

    unsigned char lethal_threshold = std::max(std::min(temp_lethal_threshold, 255), 0);
    unsigned char unknown_cost_value = std::max(std::min(temp_unknown_cost_value, 255), 0);

    bool track_unknown_space;
    private_nh.param("track_unknown_space", track_unknown_space, false);

    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    if(map_type == "costmap"){
      //make sure to lock the map data
      boost::recursive_mutex::scoped_lock lock(map_data_lock_);
      costmap_ = new Costmap2D(map_width, map_height,
          map_resolution, map_origin_x, map_origin_y, inscribed_radius, circumscribed_radius, inflation_radius,
          obstacle_range, max_obstacle_height, raytrace_range, cost_scale, input_data_, lethal_threshold, track_unknown_space, unknown_cost_value);
    }
    else if(map_type == "voxel"){

      int z_voxels;
      private_nh.param("z_voxels", z_voxels, 10);

      double z_resolution, map_origin_z;
      private_nh.param("z_resolution", z_resolution, 0.2);
      private_nh.param("origin_z", map_origin_z, 0.0);

      int unknown_threshold, mark_threshold;
      private_nh.param("unknown_threshold", unknown_threshold, z_voxels);
      private_nh.param("mark_threshold", mark_threshold, 0);

      if(!(z_voxels >= 0 && unknown_threshold >= 0 && mark_threshold >= 0)){
        ROS_FATAL("Values for z_voxels, unknown_threshold, and mark_threshold parameters must be positive.");
        throw std::runtime_error("Values for z_voxels, unknown_threshold, and mark_threshold parameters must be positive.");
      }

      boost::recursive_mutex::scoped_lock lock(map_data_lock_);
      costmap_ = new VoxelCostmap2D(map_width, map_height, z_voxels, map_resolution, z_resolution, map_origin_x, map_origin_y, map_origin_z, inscribed_radius,
          circumscribed_radius, inflation_radius, obstacle_range, raytrace_range, cost_scale, input_data_, lethal_threshold, unknown_threshold, mark_threshold,
          unknown_cost_value);
    }
    else{
      ROS_FATAL("Unsuported map type");
      throw std::runtime_error("Unsuported map type");
    }

    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_DEBUG("New map construction time: %.9f", t_diff);

    double map_publish_frequency;
    private_nh.param("publish_frequency", map_publish_frequency, 0.0);

    //create a publisher for the costmap if desired
    costmap_publisher_ = new Costmap2DPublisher(private_nh, map_publish_frequency, global_frame_);
    if(costmap_publisher_->active()){
      std::vector<geometry_msgs::Point> oriented_footprint;
      getOrientedFootprint(oriented_footprint);
      tf::Stamped<tf::Pose> global_pose;
      getRobotPose(global_pose);
      costmap_publisher_->updateCostmapData(*costmap_, oriented_footprint, global_pose);
    }

    //create a thread to handle updating the map
    double map_update_frequency;
    private_nh.param("update_frequency", map_update_frequency, 5.0);
    map_update_thread_ = new boost::thread(boost::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));

    costmap_initialized_ = true;
    
    //Create a time r to check if the robot is moving
    timer_ = private_nh.createTimer(ros::Duration(.1), &Costmap2DROS::movementCB, this);
    dsrv_ = new dynamic_reconfigure::Server<Costmap2DConfig>(ros::NodeHandle("~/"+name));
    dynamic_reconfigure::Server<Costmap2DConfig>::CallbackType cb = boost::bind(&Costmap2DROS::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void Costmap2DROS::movementCB(const ros::TimerEvent &event) {
    //don't allow configuration to happen while this check occurs
    boost::recursive_mutex::scoped_lock mcl(configuration_mutex_);

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

  void Costmap2DROS::reconfigureCB(Costmap2DConfig &config, uint32_t level) {
    ros::NodeHandle nh = ros::NodeHandle("~/"+name_);
    
    if(setup_ && config.restore_defaults) {
      config = default_config_;
      //in case someone set restore_defaults on the parameter server, avoid looping
      config.restore_defaults = false;
    }

    //change the configuration defaults to match the param server
    if(!setup_) {
      boost::recursive_mutex::scoped_lock rel(configuration_mutex_);

      last_config_ = config;
      maptype_config_ = config;
      default_config_ = config;

      setup_ = true;
    }
    else if(setup_ && robot_stopped_) {
      //lock before modifying anything
      boost::recursive_mutex::scoped_lock rel(configuration_mutex_);

      transform_tolerance_ = config.transform_tolerance;

      // shutdown and restart the map update loop at a new frequency
      map_update_thread_shutdown_ = true;
      map_update_thread_->join();
      boost::mutex::scoped_lock ml(map_update_mutex_);

      //check and configure a new robot footprint
      //accepts a lis of points formatted [[x1, y1],[x2,y2],....[xn,yn]]
      string footprint_string = config.footprint;
      boost::erase_all(footprint_string, " ");
      boost::char_separator<char> sep("[]");
      boost::tokenizer<boost::char_separator<char> > tokens(footprint_string, sep);
      vector<string> points(tokens.begin(), tokens.end());

      //parse the input string into points
      vector<geometry_msgs::Point> footprint_spec;
      bool circular = false;
      bool valid_foot = true;

      if(points.size() >= 5) {
        BOOST_FOREACH(string t, tokens) {
          if (t != ",") {
            boost::erase_all(t, " ");
            boost::char_separator<char> pt_sep(",");
            boost::tokenizer<boost::char_separator<char> > pt_tokens(t, pt_sep);
            std::vector<string> point(pt_tokens.begin(), pt_tokens.end());

            if(point.size() != 2) {
              ROS_WARN("Each point must have exactly 2 coordinates");
              valid_foot = false;
              break;
            }

            vector<double>tmp_pt;
            BOOST_FOREACH(string p, point) {
              istringstream iss(p);
              double temp;
              if(iss >> temp) {
                tmp_pt.push_back(temp);
              }
              else {
                ROS_WARN("Each coordinate must convert to a double.");
                valid_foot = false;
                break;
              }
            }
            if(!valid_foot)
              break;

            geometry_msgs::Point pt;
            pt.x = tmp_pt[0];
            pt.y = tmp_pt[1];

            footprint_spec.push_back(pt);
          }
        }
        if (valid_foot) {
          footprint_spec_ = footprint_spec;
        }
      }
      //clear the footprint for a circular robot
      else if((footprint_string == "[]" || footprint_string == "") && config.robot_radius > 0.0) {
        footprint_spec_ = vector<geometry_msgs::Point>();
        circular = true;
      }
      else if(config.footprint != last_config_.footprint) {
        ROS_ERROR("You must specify at least three points for the robot footprint, reverting to previous footprint");
        config.footprint = last_config_.footprint;
      }
      
      if(!valid_foot) {
        ROS_FATAL("The footprint must be specified as a list of at least three points, you specified %s", footprint_string.c_str());
        config.footprint = last_config_.footprint;
      }

      double inscribed_radius, circumscribed_radius;
      inscribed_radius = config.robot_radius;
      circumscribed_radius = inscribed_radius;

      //if we have a footprint, recaclulate the radii
      if(footprint_spec_.size() > 2){
        //now we need to compute the inscribed/circumscribed radius of the robot from the footprint specification
        double min_dist = std::numeric_limits<double>::max();
        double max_dist = 0.0;

        for(unsigned int i = 0; i < footprint_spec_.size() - 1; ++i){
          //check the distance from the robot center point to the first vertex
          double vertex_dist = distance(0.0, 0.0, footprint_spec_[i].x, footprint_spec_[i].y);
          double edge_dist = distanceToLine(0.0, 0.0, footprint_spec_[i].x, footprint_spec_[i].y, footprint_spec_[i+1].x, footprint_spec_[i+1].y);
          min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
          max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
        }


        //we also need to do the last vertex and the first vertex
        double vertex_dist = distance(0.0, 0.0, footprint_spec_.back().x, footprint_spec_.back().y);
        double edge_dist = distanceToLine(0.0, 0.0, footprint_spec_.back().x, footprint_spec_.back().y, footprint_spec_.front().x, footprint_spec_.front().y);
        min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
        max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));

        inscribed_radius = min_dist;
        circumscribed_radius = max_dist;
      }
      if(inscribed_radius > config.inflation_radius || circumscribed_radius > config.inflation_radius){
        ROS_WARN("You have set an inflation radius that is less than the inscribed and circumscribed radii of the robot. This is dangerous and could casue the robot to hit obstacles. Please change your inflation radius setting appropraitely.");
      }

      //if both static map and rolling window are set, we'll use static map and warn
      if(config.static_map && config.rolling_window){
        ROS_WARN("You have selected both rolling window and static map, using static_map");
        config.rolling_window = false;
      }

      //check to see if the map needs to be regenerated
      //TODO: Make sure that this check actually does the right things
      bool user_params = false;
      bool map_params = false;
      if(config.max_obstacle_height != last_config_.max_obstacle_height ||
         config.max_obstacle_range != last_config_.max_obstacle_range ||
         config.raytrace_range != last_config_.raytrace_range ||
         config.cost_scaling_factor != last_config_.cost_scaling_factor ||
         config.inflation_radius != last_config_.inflation_radius ||
         config.footprint != last_config_.footprint ||
         config.robot_radius != last_config_.robot_radius ||
         config.static_map != last_config_.static_map ||
         config.rolling_window != last_config_.rolling_window ||
         config.unknown_cost_value != last_config_.unknown_cost_value ||
         config.width != last_config_.width ||
         config.height != last_config_.height ||
         config.resolution != last_config_.resolution ||
         config.origin_x != last_config_.origin_x ||
         config.origin_y != last_config_.origin_y ||
         config.lethal_cost_threshold != last_config_.lethal_cost_threshold ||
         config.map_topic != last_config_.map_topic ||
         config.origin_z != last_config_.origin_z ||
         config.z_resolution != last_config_.z_resolution ||
         config.unknown_threshold != last_config_.unknown_threshold ||
         config.mark_threshold != last_config_.mark_threshold ||
         config.track_unknown_space != last_config_.track_unknown_space ||
         config.map_type != last_config_.map_type ||
         config.static_map != last_config_.static_map ||
         config.rolling_window != last_config_.rolling_window)
      {
        user_params = true;
      }

      if(config.width != last_config_.width ||
         config.height != last_config_.height ||
         config.resolution != last_config_.resolution ||
         config.origin_x != last_config_.origin_x ||
         config.origin_y != last_config_.origin_y)
      {
        map_params = true;
      }

      if(user_params) {
        //unmangle rolling_window and static map
        //both can be false but if one is true then the other is not
        if(!config.static_map && !config.rolling_window){
          rolling_window_ = false;
          static_map_ = false;

          if(last_config_.static_map)
          {
            map_sub_.shutdown();
            map_initialized_ = false;
          }
        }
        else if(config.static_map) {
          static_map_ = true;
          rolling_window_ = false;

          //we'll subscribe to the latched topic that the map server uses
          ros::NodeHandle g_nh;
          map_sub_.shutdown();
          map_initialized_ = false;
          map_sub_ = g_nh.subscribe(config.map_topic, 1, &Costmap2DROS::incomingMap, this);
          ROS_INFO("Requesting new map on topic %s\n", map_sub_.getTopic().c_str());

          ros::Rate r(1.0);
          while(!map_initialized_ && ros::ok()){
            ros::spinOnce();
            ROS_INFO("Still waiting on new map...\n");
            r.sleep();
          }
        }
        else if(config.rolling_window) {
          rolling_window_ = true;
          static_map_ = false;

          if(last_config_.static_map)
          {
            map_sub_.shutdown();
            map_initialized_ = false;
          }
        }

        //make sure to lock before we start messing with the map
        boost::recursive_mutex::scoped_lock mdl(map_data_lock_);

        if(!config.static_map && last_config_.static_map) {
          config.width = maptype_config_.width;
          config.height = maptype_config_.height;
          config.resolution = maptype_config_.resolution;
          config.origin_x = maptype_config_.origin_x;
          config.origin_y = maptype_config_.origin_y;

          maptype_config_ = last_config_;
        }


        unsigned int size_x, size_y;
        size_x = ceil(config.width/config.resolution);
        size_y = ceil(config.height/config.resolution);
        double resolution = config.resolution;
        double origin_x = config.origin_x;
        double origin_y = config.origin_y;

        if(config.static_map)
        {
          if(map_params)
          {
            ROS_WARN("You're trying to change parameters that will be overwritten since static_map is set. Your changes will have no effect");
          }

          size_x = (unsigned int)map_meta_data_.width;
          size_y = (unsigned int)map_meta_data_.height;
          resolution = map_meta_data_.resolution;
          origin_x = map_meta_data_.origin.position.x;
          origin_y = map_meta_data_.origin.position.y;

          config.width = size_x;
          config.height = size_y;
          config.resolution = resolution;
          config.origin_x = origin_x;
          config.origin_y = origin_y;

          ROS_INFO("Received a %d X %d map at %f m/pix\n",
              size_x, size_y, resolution);
        }

        if(config.map_type == "voxel") {
          unsigned int z_voxels;
          unsigned char lethal_threshold, unknown_cost_value;
          unsigned int unknown_threshold, mark_threshold;

          z_voxels = config.z_voxels;

          lethal_threshold = config.lethal_cost_threshold;
          unknown_threshold = config.unknown_threshold;
          mark_threshold = config.mark_threshold;

          unknown_cost_value = config.unknown_cost_value;

          VoxelCostmap2D *new_map = new VoxelCostmap2D(size_x, size_y, z_voxels, 
                         resolution, config.z_resolution, origin_x, origin_y, config.origin_z, 
                         inscribed_radius, circumscribed_radius, config.inflation_radius, 
                         config.max_obstacle_range, config.raytrace_range, config.cost_scaling_factor, 
                         input_data_, lethal_threshold, unknown_threshold, mark_threshold, unknown_cost_value);
          delete costmap_;
          costmap_ = new_map;
        }
        else if(config.map_type == "costmap") {
          unsigned char lethal_threshold, unknown_cost_value;

          lethal_threshold = config.lethal_cost_threshold;
          unknown_cost_value = config.unknown_cost_value;
         
          Costmap2D *new_map = new Costmap2D(size_x, size_y, resolution, origin_x, origin_y,
                  inscribed_radius, circumscribed_radius, config.inflation_radius,
                  config.max_obstacle_range, config.max_obstacle_height, config.raytrace_range, config.cost_scaling_factor, 
                  input_data_, lethal_threshold, config.track_unknown_space, unknown_cost_value);
          delete costmap_;
          costmap_ = new_map;
        }
      }

      if(config.map_type == "voxel" && config.publish_voxel_map) {
        publish_voxel_ = true;
        if(voxel_pub_ == NULL) {
          ros::NodeHandle nh("~/" + name_);
          voxel_pub_ = nh.advertise<costmap_2d::VoxelGrid>("voxel_grid", 1);
        }
      }
      else {
        publish_voxel_ = false;
        config.publish_voxel_map = false;
      }

      //make sure that nothing else is modifying the costmap while we reconfigure it
      boost::recursive_mutex::scoped_lock lock(lock_);

      //reconfigure the underlying costmap 
      costmap_->reconfigure(config, last_config_);

      if(config.publish_frequency != last_config_.publish_frequency)
      {
        double map_publish_frequency = config.publish_frequency;
        delete costmap_publisher_;
        costmap_publisher_ = new Costmap2DPublisher(ros::NodeHandle("~/"+name_), map_publish_frequency, global_frame_);
      }

      //once all configuration is done, restart the map update loop
      delete map_update_thread_;
      map_update_thread_shutdown_ = false;
      double map_update_frequency = config.update_frequency;
      map_update_thread_ = new boost::thread(boost::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));

      last_config_ = config;
    }
    else {
      if(setup_)
      {
        ROS_WARN("You cannot reconfigure the costmap unless the robot is stopped");
      }
      config = last_config_;
    }
  }

  double Costmap2DROS::distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1){
    double A = pX - x0;
    double B = pY - y0;
    double C = x1 - x0;
    double D = y1 - y0;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = dot / len_sq;

    double xx,yy;

    if(param < 0)
    {
      xx = x0;
      yy = y0;
    }
    else if(param > 1)
    {
      xx = x1;
      yy = y1;
    }
    else
    {
      xx = x0 + param * C;
      yy = y0 + param * D;
    }

    return distance(pX,pY,xx,yy);
  }

  std::vector<geometry_msgs::Point> Costmap2DROS::loadRobotFootprint(ros::NodeHandle node, double inscribed_radius, double circumscribed_radius){
    std::vector<geometry_msgs::Point> footprint;
    geometry_msgs::Point pt;
    double padding;

    std::string padding_param, footprint_param;
    if(!node.searchParam("footprint_padding", padding_param))
      padding = 0.01;
    else
      node.param(padding_param, padding, 0.01);

    //grab the footprint from the parameter server if possible
    XmlRpc::XmlRpcValue footprint_list;
    std::string footprint_string;
    std::vector<string> footstring_list;
    if(node.searchParam("footprint", footprint_param)){
      node.getParam(footprint_param, footprint_list);
      if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
        footprint_string = std::string(footprint_list);

        //if there's just an empty footprint up there, return
        if(footprint_string == "[]" || footprint_string == "")
          return footprint;

        boost::erase_all(footprint_string, " ");

        boost::char_separator<char> sep("[]");
        boost::tokenizer<boost::char_separator<char> > tokens(footprint_string, sep);
        footstring_list = std::vector<string>(tokens.begin(), tokens.end());
      }
      //make sure we have a list of lists
      if(!(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeArray && footprint_list.size() > 2) && !(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString && footstring_list.size() > 5)){
        ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s", footprint_param.c_str(), std::string(footprint_list).c_str());
        throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
      }

      if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for(int i = 0; i < footprint_list.size(); ++i){
          //make sure we have a list of lists of size 2
          XmlRpc::XmlRpcValue point = footprint_list[i];
          if(!(point.getType() == XmlRpc::XmlRpcValue::TypeArray && point.size() == 2)){
            ROS_FATAL("The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
            throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
          }
       
          //make sure that the value we're looking at is either a double or an int
          if(!(point[0].getType() == XmlRpc::XmlRpcValue::TypeInt || point[0].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
            ROS_FATAL("Values in the footprint specification must be numbers");
            throw std::runtime_error("Values in the footprint specification must be numbers");
          }
          pt.x = point[0].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[0]) : (double)(point[0]);
          pt.x += sign(pt.x) * padding;
       
          //make sure that the value we're looking at is either a double or an int
          if(!(point[1].getType() == XmlRpc::XmlRpcValue::TypeInt || point[1].getType() == XmlRpc::XmlRpcValue::TypeDouble)){
            ROS_FATAL("Values in the footprint specification must be numbers");
            throw std::runtime_error("Values in the footprint specification must be numbers");
          }
          pt.y = point[1].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(point[1]) : (double)(point[1]);
          pt.y += sign(pt.y) * padding;
       
          footprint.push_back(pt);

          node.deleteParam(footprint_param);
          ostringstream oss;
          bool first = true;
          BOOST_FOREACH(geometry_msgs::Point p, footprint) {
            if(first) {
              oss << "[[" << p.x << "," << p.y << "]";
              first = false;
            }
            else {
              oss << ",[" << p.x << "," << p.y << "]";
            }
          }
          oss << "]";
          node.setParam(footprint_param, oss.str().c_str());
          node.setParam("footprint", oss.str().c_str());
        }
      }

      else if(footprint_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
        std::vector<geometry_msgs::Point> footprint_spec;
        bool valid_foot = true;
        BOOST_FOREACH(string t, footstring_list) {
          if( t != "," ) {
            boost::erase_all(t, " ");
            boost::char_separator<char> pt_sep(",");
            boost::tokenizer<boost::char_separator<char> > pt_tokens(t, pt_sep);
            std::vector<string> point(pt_tokens.begin(), pt_tokens.end());

            if(point.size() != 2) {
              ROS_WARN("Each point must have exactly 2 coordinates");
              valid_foot = false;
              break;
            }

            vector<double>tmp_pt;
            BOOST_FOREACH(string p, point) {
              istringstream iss(p);
              double temp;
              if(iss >> temp) {
                tmp_pt.push_back(temp);
              }
              else {
                ROS_WARN("Each coordinate must convert to a double.");
                valid_foot = false;
                break;
              }
            }
            if(!valid_foot)
              break;

            geometry_msgs::Point pt;
            pt.x = tmp_pt[0];
            pt.y = tmp_pt[1];

            footprint_spec.push_back(pt);
          }
        }
        if (valid_foot) {
          footprint = footprint_spec;
          node.setParam("footprint", footprint_string);
        }
        else {
          ROS_FATAL("This footprint is not vaid it must be specified as a list of lists with at least 3 points, you specified %s", footprint_string.c_str());
          throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
        }
      }
    }
    return footprint;
  }

  Costmap2DROS::~Costmap2DROS(){
    delete dsrv_;
    map_update_thread_shutdown_ = true;
    if(map_update_thread_ != NULL){
      map_update_thread_->join();
      delete map_update_thread_;
    }

    if(costmap_publisher_ != NULL){
      delete costmap_publisher_;
    }

    if(costmap_ != NULL)
      delete costmap_;
  }

  void Costmap2DROS::start(){
    //check if we're stopped or just paused
    if(stopped_){
      //if we're stopped we need to re-subscribe to topics
      for(unsigned int i = 0; i < observation_subscribers_.size(); ++i){
        if(observation_subscribers_[i] != NULL)
          observation_subscribers_[i]->subscribe();
      }
      stopped_ = false;
    }
    for (unsigned int i=0; i < observation_buffers_.size(); ++i){
      if (observation_buffers_[i])
        observation_buffers_[i]->resetLastUpdated();
    } 
    stop_updates_ = false;

    //block until the costmap is re-initialized.. meaning one update cycle has run
    ros::Rate r(100.0);
    while(ros::ok() && !initialized_)
      r.sleep();
  }

  void Costmap2DROS::stop(){
    stop_updates_ = true;
    //unsubscribe from topics
    for(unsigned int i = 0; i < observation_subscribers_.size(); ++i){
      if(observation_subscribers_[i] != NULL)
        observation_subscribers_[i]->unsubscribe();
    }
    initialized_ = false;
    stopped_ = true;
  }

  void Costmap2DROS::addObservationBuffer(const boost::shared_ptr<ObservationBuffer>& buffer){
    if(buffer)
      observation_buffers_.push_back(buffer);
  }

  void Costmap2DROS::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message, const boost::shared_ptr<ObservationBuffer>& buffer){
    //project the laser into a point cloud
    sensor_msgs::PointCloud2 cloud;
    cloud.header = message->header;

    //project the scan into a point cloud
    try
    {
      projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, tf_);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN ("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str (), ex.what ());
      projector_.projectLaser(*message, cloud);
    }

    //buffer the point cloud
    buffer->lock();
    buffer->bufferCloud(cloud);
    buffer->unlock();
  }

  void Costmap2DROS::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message, const boost::shared_ptr<ObservationBuffer>& buffer){
    sensor_msgs::PointCloud2 cloud2;

    if(!sensor_msgs::convertPointCloudToPointCloud2(*message, cloud2)){
      ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
      return;
    }

    //buffer the point cloud
    buffer->lock();
    buffer->bufferCloud(cloud2);
    buffer->unlock();
  }

  void Costmap2DROS::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message, const boost::shared_ptr<ObservationBuffer>& buffer){
    //buffer the point cloud
    buffer->lock();
    buffer->bufferCloud(*message);
    buffer->unlock();
  }

  void Costmap2DROS::mapUpdateLoop(double frequency){
    //the user might not want to run the loop every cycle
    if(frequency == 0.0)
      return;

    boost::mutex::scoped_lock ml(map_update_mutex_);

    ros::NodeHandle nh;
    ros::Rate r(frequency);
    while(nh.ok() && !map_update_thread_shutdown_){
      struct timeval start, end;
      double start_t, end_t, t_diff;
      gettimeofday(&start, NULL);
      if(!stop_updates_){
        updateMap();
        initialized_ = true;
      }
      gettimeofday(&end, NULL);
      start_t = start.tv_sec + double(start.tv_usec) / 1e6;
      end_t = end.tv_sec + double(end.tv_usec) / 1e6;
      t_diff = end_t - start_t;
      ROS_DEBUG("Map update time: %.9f", t_diff);

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / frequency))
        ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency, r.cycleTime().toSec());
    }
  }

  bool Costmap2DROS::getMarkingObservations(std::vector<Observation>& marking_observations) const {
    bool current = true;
    //get the marking observations
    for(unsigned int i = 0; i < marking_buffers_.size(); ++i){
      marking_buffers_[i]->lock();
      marking_buffers_[i]->getObservations(marking_observations);
      current = marking_buffers_[i]->isCurrent() && current;
      marking_buffers_[i]->unlock();
    }
    return current;
  }

  bool Costmap2DROS::getClearingObservations(std::vector<Observation>& clearing_observations) const {
    bool current = true;
    //get the clearing observations
    for(unsigned int i = 0; i < clearing_buffers_.size(); ++i){
      clearing_buffers_[i]->lock();
      clearing_buffers_[i]->getObservations(clearing_observations);
      current = clearing_buffers_[i]->isCurrent() && current;
      clearing_buffers_[i]->unlock();
    }
    return current;
  }


  void Costmap2DROS::updateMap(){
    tf::Stamped<tf::Pose> global_pose;
    if(!getRobotPose(global_pose))
      return;

    double wx = global_pose.getOrigin().x();
    double wy = global_pose.getOrigin().y();

    bool current = true;
    std::vector<Observation> observations, clearing_observations;

    //get the marking observations
    current = current && getMarkingObservations(observations);

    //get the clearing observations
    current = current && getClearingObservations(clearing_observations);

    //update the global current status
    current_ = current;

    boost::recursive_mutex::scoped_lock uml(configuration_mutex_);
    boost::recursive_mutex::scoped_lock lock(lock_);
    //if we're using a rolling buffer costmap... we need to update the origin using the robot's position
    if(rolling_window_){
      double origin_x = wx - costmap_->getSizeInMetersX() / 2;
      double origin_y = wy - costmap_->getSizeInMetersY() / 2;
      costmap_->updateOrigin(origin_x, origin_y);
    }
    costmap_->updateWorld(wx, wy, observations, clearing_observations);

    //make sure to clear the robot footprint of obstacles at the end
    clearRobotFootprint();
    
    if(save_debug_pgm_)
      costmap_->saveMap(name_ + ".pgm");

    //if we have an active publisher... we'll update its costmap data
    if(costmap_publisher_->active()){
      std::vector<geometry_msgs::Point> oriented_footprint;
      getOrientedFootprint(oriented_footprint);
      tf::Stamped<tf::Pose> global_pose;
      getRobotPose(global_pose);
      costmap_publisher_->updateCostmapData(*costmap_, oriented_footprint, global_pose);
    }

    if(publish_voxel_){
      costmap_2d::VoxelGrid voxel_grid;
      ((VoxelCostmap2D*)costmap_)->getVoxelGridMessage(voxel_grid);
      voxel_grid.header.frame_id = global_frame_;
      voxel_grid.header.stamp = ros::Time::now();
      voxel_pub_.publish(voxel_grid);
    }
  }

  void Costmap2DROS::clearNonLethalWindow(double size_x, double size_y){
    tf::Stamped<tf::Pose> global_pose;
    if(!getRobotPose(global_pose))
      return;

    double wx = global_pose.getOrigin().x();
    double wy = global_pose.getOrigin().y();
    lock_.lock();
    ROS_DEBUG("Clearing map in window");
    costmap_->clearNonLethal(wx, wy, size_x, size_y, true);
    lock_.unlock();

    //make sure to force an update of the map to take in the latest sensor data
    updateMap();
  }

  void Costmap2DROS::resetMapOutsideWindow(double size_x, double size_y){
    tf::Stamped<tf::Pose> global_pose;
    if(!getRobotPose(global_pose))
      return;

    double wx = global_pose.getOrigin().x();
    double wy = global_pose.getOrigin().y();
    lock_.lock();
    ROS_DEBUG("Resetting map outside window");
    costmap_->resetMapOutsideWindow(wx, wy, size_x, size_y);
    lock_.unlock();

    //make sure to force an update of the map to take in the latest sensor data
    updateMap();

  }

  void Costmap2DROS::getCostmapCopy(Costmap2D& costmap) const {
    boost::recursive_mutex::scoped_lock lock(lock_);
    costmap = *costmap_;
  }

  void Costmap2DROS::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map){
    if(!map_initialized_){
      initFromMap(*new_map);
      map_initialized_ = true;
    }
    else if(costmap_initialized_)
      updateStaticMap(*new_map);
  }

  void Costmap2DROS::initFromMap(const nav_msgs::OccupancyGrid& map){
    boost::recursive_mutex::scoped_lock lock(map_data_lock_);

    // We need to cast to unsigned chars from int
    unsigned int numCells = map.info.width * map.info.height;
    for(unsigned int i = 0; i < numCells; i++){
      input_data_.push_back((unsigned char) map.data[i]);
    }

    map_meta_data_ = map.info;
    global_frame_ = tf::resolve(tf_prefix_, map.header.frame_id);
  }

  void Costmap2DROS::updateStaticMap(const nav_msgs::OccupancyGrid& new_map){
    std::vector<unsigned char> new_map_data;
    // We need to cast to unsigned chars from int
    unsigned int numCells = new_map.info.width * new_map.info.height;
    for(unsigned int i = 0; i < numCells; i++){
      new_map_data.push_back((unsigned char) new_map.data[i]);
    }

    double map_width = (unsigned int)new_map.info.width;
    double map_height = (unsigned int)new_map.info.height;
    double map_resolution = new_map.info.resolution;
    double map_origin_x = new_map.info.origin.position.x;
    double map_origin_y = new_map.info.origin.position.y;

    if(fabs(map_resolution - costmap_->getResolution()) > 1e-6){
      ROS_ERROR("You cannot update a map with resolution: %.4f, with a new map that has resolution: %.4f", 
          costmap_->getResolution(), map_resolution);
      return;
    }

    if(fabs(new_map.info.origin.orientation.x) > 1e-6 
       && fabs(new_map.info.origin.orientation.y) > 1e-6 
       && fabs(new_map.info.origin.orientation.z) > 1e-6 
       && (fabs(new_map.info.origin.orientation.w) > 1e-6 || fabs(new_map.info.origin.orientation.w - 1.0) > 1e-6)){
      ROS_ERROR("The costmap does not support origins that contain rotations. The origin must be aligned with the global_frame.");
      return;
    }

    if(tf::resolve(tf_prefix_, new_map.header.frame_id) != tf::resolve(tf_prefix_, global_frame_)){
      std::string new_global_frame = tf::resolve(tf_prefix_, new_map.header.frame_id);

      ROS_DEBUG("Map with a global_frame of: %s, updated with a new map that has a global frame of: %s, wiping map", global_frame_.c_str(), new_map.header.frame_id.c_str());

      //we'll update all the observation buffers we have associated with this map
      for(unsigned int i = 0; i < observation_buffers_.size(); ++i){
        observation_buffers_[i]->lock();
        observation_buffers_[i]->setGlobalFrame(new_global_frame);
        observation_buffers_[i]->unlock();
      }

      //make sure to lock the costmap
      boost::recursive_mutex::scoped_lock uml(configuration_mutex_);
      boost::recursive_mutex::scoped_lock lock(lock_);

      //if the map has a new global frame... we'll actually wipe the whole map rather than trying to be efficient about updating a potential window
      costmap_->replaceFullMap(map_origin_x, map_origin_y, map_width, map_height, new_map_data);

      //we'll also update the global frame id for this costmap
      global_frame_ = new_global_frame;

      return;
    }

    boost::recursive_mutex::scoped_lock lock(lock_);
    costmap_->updateStaticMapWindow(map_origin_x, map_origin_y, map_width, map_height, new_map_data);
  }

  void Costmap2DROS::getCostmapWindowCopy(double win_size_x, double win_size_y, Costmap2D& costmap) const {
    boost::recursive_mutex::scoped_lock lock(lock_);
    tf::Stamped<tf::Pose> global_pose;
    if(!getRobotPose(global_pose)){
      ROS_ERROR("Could not get a window of this costmap centered at the robot, because we failed to get the pose of the robot");
      return;
    }
    getCostmapWindowCopy(global_pose.getOrigin().x(), global_pose.getOrigin().y(), win_size_x, win_size_y, costmap);
  }

  void Costmap2DROS::getCostmapWindowCopy(double win_center_x, double win_center_y, double win_size_x, double win_size_y, Costmap2D& costmap) const {
    boost::recursive_mutex::scoped_lock lock(lock_);

    //we need to compute legal bounds for the window and shrink it if necessary
    double ll_x = std::min(std::max(win_center_x - win_size_x, costmap_->getOriginX()), costmap_->getSizeInMetersX());
    double ll_y = std::min(std::max(win_center_y - win_size_y, costmap_->getOriginY()), costmap_->getSizeInMetersY());
    double ur_x = std::min(std::max(win_center_x + win_size_x, costmap_->getOriginX()), costmap_->getSizeInMetersX());
    double ur_y = std::min(std::max(win_center_y + win_size_y, costmap_->getOriginY()), costmap_->getSizeInMetersY());
    double size_x = ur_x - ll_x;
    double size_y = ur_y - ll_y;

    //copy the appropriate window from our costmap into the one passed in by the user
    costmap.copyCostmapWindow(*costmap_, ll_x, ll_y, size_x, size_y);
  }

  unsigned int Costmap2DROS::getSizeInCellsX() const {
    boost::recursive_mutex::scoped_lock lock(lock_);
    return costmap_->getSizeInCellsX();
  }

  unsigned int Costmap2DROS::getSizeInCellsY() const {
    boost::recursive_mutex::scoped_lock lock(lock_);
    return costmap_->getSizeInCellsY();
  }

  double Costmap2DROS::getResolution() const {
    boost::recursive_mutex::scoped_lock lock(lock_);
    return costmap_->getResolution();
  }

  bool Costmap2DROS::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const {

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

  void Costmap2DROS::clearRobotFootprint(){
    tf::Stamped<tf::Pose> global_pose;
    if(!getRobotPose(global_pose))
      return;

    clearRobotFootprint(global_pose);
  }

  std::vector<geometry_msgs::Point> Costmap2DROS::getRobotFootprint() const {
    return footprint_spec_;
  }

  void Costmap2DROS::getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const {
    tf::Stamped<tf::Pose> global_pose;
    if(!getRobotPose(global_pose))
      return;

    double yaw = tf::getYaw(global_pose.getRotation());

    getOrientedFootprint(global_pose.getOrigin().x(), global_pose.getOrigin().y(), yaw, oriented_footprint);
  }

  void Costmap2DROS::getOrientedFootprint(double x, double y, double theta, std::vector<geometry_msgs::Point>& oriented_footprint) const {
    //build the oriented footprint at the robot's current location
    double cos_th = cos(theta);
    double sin_th = sin(theta);
    for(unsigned int i = 0; i < footprint_spec_.size(); ++i){
      geometry_msgs::Point new_pt;
      new_pt.x = x + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
      new_pt.y = y + (footprint_spec_[i].x * sin_th + footprint_spec_[i].y * cos_th);
      oriented_footprint.push_back(new_pt);
    }
  }

  bool Costmap2DROS::setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value){
    lock_.lock();
    bool success = costmap_->setConvexPolygonCost(polygon, cost_value);
    lock_.unlock();

    //make sure to take our active sensor data into account
    updateMap();

    return success;
  }

  std::string Costmap2DROS::getGlobalFrameID() const {
    return global_frame_;
  }

  std::string Costmap2DROS::getBaseFrameID() const {
    return robot_base_frame_;
  }

  double Costmap2DROS::getInscribedRadius() const {
    boost::recursive_mutex::scoped_lock lock(lock_);
    return costmap_->getInscribedRadius();
  }

  double Costmap2DROS::getCircumscribedRadius() const {
    boost::recursive_mutex::scoped_lock lock(lock_);
    return costmap_->getCircumscribedRadius();
  }

  double Costmap2DROS::getInflationRadius() const {
    boost::recursive_mutex::scoped_lock lock(lock_);
    return costmap_->getInflationRadius();
  }

  void Costmap2DROS::clearRobotFootprint(const tf::Stamped<tf::Pose>& global_pose){
    std::vector<geometry_msgs::Point> oriented_footprint;

    //check if we have a circular footprint or a polygon footprint
    if(footprint_spec_.size() < 3){
      //we'll build an approximation of the circle as the footprint and clear that
      double angle = 0;
      double step = 2 * M_PI / 72;
      while(angle < 2 * M_PI){
        geometry_msgs::Point pt;
        pt.x = getInscribedRadius() * cos(angle) + global_pose.getOrigin().x();
        pt.y = getInscribedRadius() * sin(angle) + global_pose.getOrigin().y();
        pt.z = 0.0;
        oriented_footprint.push_back(pt);
        angle += step;
      }
    }
    else{
      double yaw = tf::getYaw(global_pose.getRotation());

      //get the oriented footprint of the robot
      double x = global_pose.getOrigin().x();
      double y = global_pose.getOrigin().y();
      double theta = yaw;

      //build the oriented footprint at the robot's current location
      getOrientedFootprint(x, y, theta, oriented_footprint);
    }

    //lock the map if necessary
    boost::recursive_mutex::scoped_lock lock(lock_);

    //set the associated costs in the cost map to be free
    if(!costmap_->setConvexPolygonCost(oriented_footprint, costmap_2d::FREE_SPACE))
      return;

    double max_inflation_dist = 2 * (costmap_->getInflationRadius() + costmap_->getCircumscribedRadius());

    //clear all non-lethal obstacles out to the maximum inflation distance of an obstacle in the robot footprint
    costmap_->clearNonLethal(global_pose.getOrigin().x(), global_pose.getOrigin().y(), max_inflation_dist, max_inflation_dist);

    //make sure to re-inflate obstacles in the affected region... plus those obstalces that could inflate to have costs in the footprint
    costmap_->reinflateWindow(global_pose.getOrigin().x(), global_pose.getOrigin().y(), 
        max_inflation_dist + 2 * costmap_->getInflationRadius(), max_inflation_dist + 2 * costmap_->getInflationRadius(), false);

  }

};
