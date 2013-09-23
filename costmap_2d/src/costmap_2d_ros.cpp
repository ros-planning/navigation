/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *         David V. Lu!!
 *********************************************************************/
#include "costmap_2d/array_parser.h"
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>


using namespace std;

namespace costmap_2d
{

void move_parameter(ros::NodeHandle& old_h, ros::NodeHandle& new_h, std::string name)
{
  if (!old_h.hasParam(name))
    return;

  XmlRpc::XmlRpcValue value;
  old_h.getParam(name, value);
  new_h.setParam(name, value);
  old_h.deleteParam(name);
}

Costmap2DROS::Costmap2DROS(std::string name, tf::TransformListener& tf) :
    layered_costmap_(NULL), name_(name), tf_(tf), stop_updates_(false), initialized_(true), stopped_(false), robot_stopped_(
        false), map_update_thread_(NULL), last_publish_(0), plugin_loader_("costmap_2d",
                                                                           "costmap_2d::Layer"), publisher_(
        NULL)
{
  ros::NodeHandle private_nh("~/" + name);
  ros::NodeHandle g_nh;

  // get our tf prefix
  ros::NodeHandle prefix_nh;
  std::string tf_prefix = tf::getPrefixParam(prefix_nh);

  // get two frames
  private_nh.param("global_frame", global_frame_, std::string("/map"));
  private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));

  // make sure that we set the frames appropriately based on the tf_prefix
  global_frame_ = tf::resolve(tf_prefix, global_frame_);
  robot_base_frame_ = tf::resolve(tf_prefix, robot_base_frame_);

  ros::Time last_error = ros::Time::now();
  std::string tf_error;
  // we need to make sure that the transform between the robot base frame and the global frame is available
  while (ros::ok()
      && !tf_.waitForTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), ros::Duration(0.01),
                               &tf_error))
  {
    ros::spinOnce();
    if (last_error + ros::Duration(5.0) < ros::Time::now())
    {
      ROS_WARN("Waiting on transform from %s to %s to become available before running costmap, tf error: %s",
               robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
      last_error = ros::Time::now();
    }
  }

  // check if we want a rolling window version of the costmap
  bool rolling_window, track_unknown_space;
  private_nh.param("rolling_window", rolling_window, false);
  private_nh.param("track_unknown_space", track_unknown_space, false);

  layered_costmap_ = new LayeredCostmap(global_frame_, rolling_window, track_unknown_space);

  if (!private_nh.hasParam("plugins"))
  {
    resetOldParameters(private_nh);
  }

  if (private_nh.hasParam("plugins"))
  {
    XmlRpc::XmlRpcValue my_list;
    private_nh.getParam("plugins", my_list);
    for (int32_t i = 0; i < my_list.size(); ++i)
    {
      std::string pname = static_cast<std::string>(my_list[i]["name"]);
      std::string type = static_cast<std::string>(my_list[i]["type"]);
      ROS_INFO("Using plugin \"%s\"", pname.c_str());

      boost::shared_ptr<Layer> plugin = plugin_loader_.createInstance(type);
      layered_costmap_->addPlugin(plugin);
      plugin->initialize(layered_costmap_, name + "/" + pname, &tf_);
    }
  }

  // subscribe to the footprint topic
  std::string topic_param, topic;
  if(!private_nh.searchParam("footprint_topic", topic_param))
  {
    topic_param = "footprint_topic";
  }

  private_nh.param(topic_param, topic, std::string("footprint"));
  footprint_sub_ = private_nh.subscribe(topic, 1, &Costmap2DROS::setUnpaddedRobotFootprintPolygon, this);

  readFootprintFromParams( private_nh );

  publisher_ = new Costmap2DPublisher(&private_nh, layered_costmap_->getCostmap(), global_frame_, "costmap");

  // create a thread to handle updating the map
  stop_updates_ = false;
  initialized_ = true;
  stopped_ = false;

  // Create a time r to check if the robot is moving
  robot_stopped_ = false;
  timer_ = private_nh.createTimer(ros::Duration(.1), &Costmap2DROS::movementCB, this);

  dsrv_ = new dynamic_reconfigure::Server<Costmap2DConfig>(ros::NodeHandle("~/" + name));
  dynamic_reconfigure::Server<Costmap2DConfig>::CallbackType cb = boost::bind(&Costmap2DROS::reconfigureCB, this, _1,
                                                                              _2);
  dsrv_->setCallback(cb);
}

void Costmap2DROS::setUnpaddedRobotFootprintPolygon( const geometry_msgs::Polygon& footprint )
{
  setUnpaddedRobotFootprint( toPointVector( footprint ));
}

Costmap2DROS::~Costmap2DROS()
{
  map_update_thread_shutdown_ = true;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_->join();
    delete map_update_thread_;
  }
  if (publisher_ != NULL)
    delete publisher_;

  delete layered_costmap_;
}

void Costmap2DROS::resetOldParameters(ros::NodeHandle& nh)
{
  ROS_INFO("Loading from pre-hydro parameter style");
  bool flag;
  std::string s;
  std::vector < XmlRpc::XmlRpcValue > plugins;

  XmlRpc::XmlRpcValue::ValueStruct map;
  SuperValue super_map;
  SuperValue super_array;

  if (nh.getParam("static_map", flag) && flag)
  {
    map["name"] = XmlRpc::XmlRpcValue("static_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::StaticLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);

    ros::NodeHandle map_layer(nh, "static_layer");
    move_parameter(nh, map_layer, "map_topic");
    move_parameter(nh, map_layer, "unknown_cost_value");
    move_parameter(nh, map_layer, "lethal_cost_threshold");
    move_parameter(nh, map_layer, "track_unknown_space");
  }

  ros::NodeHandle obstacles(nh, "obstacle_layer");
  if (nh.getParam("map_type", s) && s == "voxel")
  {

    map["name"] = XmlRpc::XmlRpcValue("obstacle_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::VoxelLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);

    move_parameter(nh, obstacles, "origin_z");
    move_parameter(nh, obstacles, "z_resolution");
    move_parameter(nh, obstacles, "z_voxels");
    move_parameter(nh, obstacles, "mark_threshold");
    move_parameter(nh, obstacles, "unknown_threshold");
    move_parameter(nh, obstacles, "publish_voxel_map");
  }
  else
  {
    map["name"] = XmlRpc::XmlRpcValue("obstacle_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::ObstacleLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);
  }

  move_parameter(nh, obstacles, "max_obstacle_height");
  move_parameter(nh, obstacles, "raytrace_range");
  move_parameter(nh, obstacles, "obstacle_range");
  nh.param("observation_sources", s, std::string(""));
  std::stringstream ss(s);
  std::string source;
  while (ss >> source)
  {
    move_parameter(nh, obstacles, source);
  }
  move_parameter(nh, obstacles, "observation_sources");

  map["name"] = XmlRpc::XmlRpcValue("footprint_layer");
  map["type"] = XmlRpc::XmlRpcValue("costmap_2d::FootprintLayer");
  super_map.setStruct(&map);
  plugins.push_back(super_map);

  ros::NodeHandle inflation(nh, "inflation_layer");
  move_parameter(nh, inflation, "cost_scaling_factor");
  move_parameter(nh, inflation, "inflation_radius");
  map["name"] = XmlRpc::XmlRpcValue("inflation_layer");
  map["type"] = XmlRpc::XmlRpcValue("costmap_2d::InflationLayer");
  super_map.setStruct(&map);
  plugins.push_back(super_map);

  super_array.setArray(&plugins);
  nh.setParam("plugins", super_array);

}

void Costmap2DROS::reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level)
{
  transform_tolerance_ = config.transform_tolerance;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_shutdown_ = true;
    map_update_thread_->join();
    delete map_update_thread_;
  }
  map_update_thread_shutdown_ = false;
  double map_update_frequency = config.update_frequency;

  double map_publish_frequency = config.publish_frequency;
  if (map_publish_frequency > 0)
    publish_cycle = ros::Duration(1 / map_publish_frequency);
  else
    publish_cycle = ros::Duration(-1);

  // find size parameters
  double map_width_meters = config.width, map_height_meters = config.height, resolution = config.resolution, origin_x =
             config.origin_x,
         origin_y = config.origin_y;

  if (!layered_costmap_->isSizeLocked())
  {
    layered_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
  }

  // If the padding has changed, call setUnpaddedRobotFootprint() to
  // re-apply the padding.
  if( footprint_padding_ != config.footprint_padding )
  {
    footprint_padding_ = config.footprint_padding;
    setUnpaddedRobotFootprint( unpadded_footprint_ );
  }

  readFootprintFromConfig( config, old_config_ );

  old_config_ = config;

  map_update_thread_ = new boost::thread(boost::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));
}

void Costmap2DROS::readFootprintFromConfig( const costmap_2d::Costmap2DConfig &new_config,
                                            const costmap_2d::Costmap2DConfig &old_config )
{
  // Only change the footprint if footprint or robot_radius has
  // changed.  Otherwise we might overwrite a footprint sent on a
  // topic by a dynamic_reconfigure call which was setting some other
  // variable.
  if( new_config.footprint == old_config.footprint &&
      new_config.robot_radius == old_config.robot_radius )
  {
    return;
  }

  if( new_config.footprint != "" && new_config.footprint != "[]" )
  {
    readFootprintFromString( new_config.footprint );
  }
  else
  {
    // robot_radius may be 0, but that must be intended at this point.
    setFootprintFromRadius( new_config.robot_radius );
  }
}

bool Costmap2DROS::readFootprintFromString( const std::string& footprint_string )
{
  std::string error;
  std::vector<std::vector<float> > vvf = parseVVF( footprint_string, error );
  if( error != "" )
  {
    ROS_ERROR( "Error parsing footprint parameter: '%s'", error.c_str() );
    ROS_ERROR( "  Footprint string was '%s'.", footprint_string.c_str() );
    return false;
  }

  // convert vvf into points.
  if( vvf.size() < 3 )
  {
    ROS_ERROR( "You must specify at least three points for the robot footprint, reverting to previous footprint." );
    return false;
  }
  std::vector<geometry_msgs::Point> points;
  points.reserve( vvf.size() );
  for( unsigned int i = 0; i < vvf.size(); i++ )
  {
    if( vvf[ i ].size() == 2 )
    {
      geometry_msgs::Point point;
      point.x = vvf[ i ][ 0 ];
      point.y = vvf[ i ][ 1 ];
      point.z = 0;
      points.push_back( point );
    }
    else
    {
      ROS_ERROR( "Points in the footprint specification must be pairs of numbers.  Found a point with %d numbers.",
                 int( vvf[ i ].size() ));
      return false;
    }
  }

  setUnpaddedRobotFootprint( points );
  return true;
}

void Costmap2DROS::setFootprintFromRadius( double radius )
{
  std::vector<geometry_msgs::Point> points;

  // Loop over 16 angles around a circle making a point each time
  int N = 16;
  geometry_msgs::Point pt;
  for( int i = 0; i < N; ++i )
  {
    double angle = i * 2 * M_PI / N;
    pt.x = cos( angle ) * radius;
    pt.y = sin( angle ) * radius;

    points.push_back( pt );
  }

  setUnpaddedRobotFootprint( points );
}

void Costmap2DROS::readFootprintFromParams( ros::NodeHandle& nh )
{
  std::string full_param_name;
  std::string full_radius_param_name;

  if( nh.searchParam( "footprint", full_param_name ))
  {
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    nh.getParam( full_param_name, footprint_xmlrpc );
    if( footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString )
    {
      if( readFootprintFromString( std::string( footprint_xmlrpc )))
      {
        writeFootprintToParam( nh );
      }
    }
    else if( footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray )
    {
      readFootprintFromXMLRPC( footprint_xmlrpc, full_param_name );
      writeFootprintToParam( nh );
    }
  }
  else if( nh.searchParam( "robot_radius", full_radius_param_name ))
  {
    double robot_radius;
    nh.param( full_radius_param_name, robot_radius, 1.234 );
    setFootprintFromRadius( robot_radius );
    nh.setParam( "robot_radius", robot_radius );
  }
  // Else neither param was found anywhere this knows about, so
  // defaults will come from dynamic_reconfigure stuff, set in
  // cfg/Costmap2D.cfg and read in this file in reconfigureCB().
}

void Costmap2DROS::writeFootprintToParam( ros::NodeHandle& nh )
{
  ostringstream oss;
  bool first = true;
  for( unsigned int i = 0; i < unpadded_footprint_.size(); i++ )
  {
    geometry_msgs::Point& p = unpadded_footprint_[ i ];
    if( first )
    {
      oss << "[[" << p.x << "," << p.y << "]";
      first = false;
    }
    else
    {
      oss << ",[" << p.x << "," << p.y << "]";
    }
  }
  oss << "]";
  nh.setParam( "footprint", oss.str().c_str() );
}

double getNumberFromXMLRPC( XmlRpc::XmlRpcValue& value, const std::string& full_param_name )
{
  // Make sure that the value we're looking at is either a double or an int.
  if( value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble )
  {
    std::string& value_string = value;
    ROS_FATAL( "Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str() );
    throw std::runtime_error("Values in the footprint specification must be numbers");
  }
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

void Costmap2DROS::readFootprintFromXMLRPC( XmlRpc::XmlRpcValue& footprint_xmlrpc,
                                            const std::string& full_param_name )
{
  // Make sure we have an array of at least 3 elements.
  if( footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      footprint_xmlrpc.size() < 3 )
  {
    ROS_FATAL( "The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
               full_param_name.c_str(), std::string( footprint_xmlrpc ).c_str() );
    throw std::runtime_error( "The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
  }

  std::vector<geometry_msgs::Point> footprint;  
  geometry_msgs::Point pt;

  for( int i = 0; i < footprint_xmlrpc.size(); ++i )
  {
    // Make sure each element of the list is an array of size 2. (x and y coordinates)
    XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
    if( point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        point.size() != 2 )
    {
      ROS_FATAL( "The footprint (parameter %s) must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                 full_param_name.c_str() );
      throw std::runtime_error( "The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form" );
    }
       
    pt.x = getNumberFromXMLRPC( point[ 0 ], full_param_name );
    pt.y = getNumberFromXMLRPC( point[ 1 ], full_param_name );
       
    footprint.push_back( pt );
  }

  setUnpaddedRobotFootprint( footprint );
}

double sign(double x){
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

void Costmap2DROS::setUnpaddedRobotFootprint( const std::vector<geometry_msgs::Point>& points )
{
  unpadded_footprint_ = points;
  padded_footprint_ = points;

  // apply footprint_padding_ to padded_footprint_ (changing it in place).
  for( unsigned int i = 0; i < padded_footprint_.size(); i++ )
  {
    geometry_msgs::Point& pt = padded_footprint_[ i ];
    pt.x += sign( pt.x ) * footprint_padding_;
    pt.y += sign( pt.y ) * footprint_padding_;
  }

  layered_costmap_->setFootprint( padded_footprint_ );
}

void Costmap2DROS::movementCB(const ros::TimerEvent &event)
{
  //don't allow configuration to happen while this check occurs
  //boost::recursive_mutex::scoped_lock mcl(configuration_mutex_);

  tf::Stamped < tf::Pose > new_pose;

  if (!getRobotPose(new_pose))
  {
    ROS_WARN_THROTTLE(1.0, "Could not get robot pose, cancelling reconfiguration");
    robot_stopped_ = false;
  }
  //make sure that the robot is not moving
  else if (fabs((old_pose_.getOrigin() - new_pose.getOrigin()).length()) < 1e-3
      && fabs(old_pose_.getRotation().angle(new_pose.getRotation())) < 1e-3)
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

void Costmap2DROS::mapUpdateLoop(double frequency)
{
  // the user might not want to run the loop every cycle
  if (frequency == 0.0)
    return;

  ros::NodeHandle nh;
  ros::Rate r(frequency);
  while (nh.ok() && !map_update_thread_shutdown_)
  {
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    if (!stop_updates_)
    {
      //get global pose
      tf::Stamped < tf::Pose > pose;
      if (getRobotPose (pose))
      {
        layered_costmap_->updateMap(pose.getOrigin().x(), pose.getOrigin().y(), tf::getYaw(pose.getRotation()));
        initialized_ = true;
      }
    }
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_DEBUG("Map update time: %.9f", t_diff);
    if (publish_cycle.toSec() > 0)
    {
      unsigned int x0, y0, xn, yn;
      layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
      publisher_->updateBounds(x0, xn, y0, yn);

      ros::Time now = ros::Time::now();
      if (last_publish_ + publish_cycle < now)
      {
        publisher_->publishCostmap();
        last_publish_ = now;
      }
    }
    r.sleep();
    // make sure to sleep for the remainder of our cycle time
    if (r.cycleTime() > ros::Duration(1 / frequency))
      ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency,
               r.cycleTime().toSec());
  }
}

void Costmap2DROS::start()
{
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  // check if we're stopped or just paused
  if (stopped_)
  {
    // if we're stopped we need to re-subscribe to topics
    for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
        ++plugin)
    {
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

void Costmap2DROS::stop()
{
  stop_updates_ = true;
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  // unsubscribe from topics
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->deactivate();
  }
  initialized_ = false;
  stopped_ = true;
}

void Costmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void Costmap2DROS::resume()
{
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  ros::Rate r(100.0);
  while (!initialized_)
    r.sleep();
}


void Costmap2DROS::resetLayers()
{
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->reset();
  }
}

bool Costmap2DROS::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const
{

  global_pose.setIdentity();
  tf::Stamped < tf::Pose > robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = robot_base_frame_;
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

  //get the global pose of the robot
  try
  {
    tf_.transformPose(global_frame_, robot_pose, global_pose);
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  // check global_pose timeout
  if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.stamp_.toSec(), transform_tolerance_);
    return false;
  }

  return true;
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
  for(unsigned int i = 0; i < padded_footprint_.size(); ++i){
    geometry_msgs::Point new_pt;
    new_pt.x = x + (padded_footprint_[i].x * cos_th - padded_footprint_[i].y * sin_th);
    new_pt.y = y + (padded_footprint_[i].x * sin_th + padded_footprint_[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
  }
}

} // namespace layered_costmap
