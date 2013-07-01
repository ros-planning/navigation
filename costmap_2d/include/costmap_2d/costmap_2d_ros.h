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
#ifndef COSTMAP_COSTMAP_2D_ROS_H_
#define COSTMAP_COSTMAP_2D_ROS_H_

#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/Costmap2DConfig.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/Polygon.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.h>

class SuperValue : public XmlRpc::XmlRpcValue
{
public:
  void setStruct(XmlRpc::XmlRpcValue::ValueStruct* a)
  {
    _type = TypeStruct;
    _value.asStruct = new XmlRpc::XmlRpcValue::ValueStruct(*a);
  }
  void setArray(XmlRpc::XmlRpcValue::ValueArray* a)
  {
    _type = TypeArray;
    _value.asArray = new std::vector<XmlRpc::XmlRpcValue>(*a);
  }
};

namespace costmap_2d
{

/** @brief A ROS wrapper for a 2D Costmap. Handles subscribing to
 * topics that provide observations about obstacles in either the form
 * of PointCloud or LaserScan messages. */
class Costmap2DROS
{
public:
  /**
   * @brief  Constructor for the wrapper
   * @param name The name for this costmap
   * @param tf A reference to a TransformListener
   */
  Costmap2DROS(std::string name, tf::TransformListener& tf);
  ~Costmap2DROS();
  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y);

  /**
   * @brief  Subscribes to sensor topics if necessary and starts costmap
   * updates, can be called to restart the costmap after calls to either
   * stop() or pause()
   */
  void start();

  /**
   * @brief  Stops costmap updates and unsubscribes from sensor topics
   */
  void stop();

  /**
   * @brief  Stops the costmap from updating, but sensor data still comes in over the wire
   */
  void pause();

  /**
   * @brief  Resumes costmap updates
   */
  void resume();

  bool isCurrent()
    {
      return layered_costmap_->isCurrent();
    }

  /**
   * @brief Get the pose of the robot in the global frame of the costmap
   * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
   * @return True if the pose was set successfully, false otherwise
   */
  bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;

  Costmap2D* getCostmap()
    {
      return layered_costmap_->getCostmap();
    }

  /**
   * @brief  Returns the global frame of the costmap
   * @return The global frame of the costmap
   */
  std::string getGlobalFrameID()
    {
      return global_frame_;
    }

  /**
   * @brief  Returns the local frame of the costmap
   * @return The local frame of the costmap
   */
  std::string getBaseFrameID()
    {
      return robot_base_frame_;
    }
  LayeredCostmap* getLayeredCostmap()
    {
      return layered_costmap_;
    }

  geometry_msgs::Polygon getRobotFootprintPolygon()
  {
    return costmap_2d::toPolygon(footprint_spec_); 
  }


  std::vector<geometry_msgs::Point> getRobotFootprint()
  {
    return footprint_spec_;
  }

  /**
   * @brief  Given a pose, build the oriented footprint of the robot
   * @param  x The x position of the robot
   * @param  y The y position of the robot
   * @param  theta The orientation of the robot
   * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
   */
  void getOrientedFootprint(double x, double y, double theta, std::vector<geometry_msgs::Point>& oriented_footprint) const;

  /**
   * @brief  Build the oriented footprint of the robot at the robot's current pose
   * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
   */
  void getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const;


protected:
  LayeredCostmap* layered_costmap_;
  std::string name_;
  tf::TransformListener& tf_;  ///< @brief Used for transforming point clouds
  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
  double transform_tolerance_; ///< timeout before transform errors

private:
  /** @brief Set the footprint from the given string.
   *
   * Format should be bracketed array of arrays of floats, like so: [[1.0, 2.2], [3.3, 4.2], ...] */
  void readFootprintFromString( const std::string& footprint_string );

  /** @brief Set the footprint from the new_config object.
   *
   * If the values of footprint and robot_radius are the same in
   * new_config and old_config, nothing is changed. */
  void readFootprintFromConfig( const costmap_2d::Costmap2DConfig &new_config,
                                const costmap_2d::Costmap2DConfig &old_config );

  /** @brief Set the footprint to a circle of the given radius, in meters. */
  void setFootprintFromRadius( double radius );
  
  /** @brief Read the ros-params "footprint" and/or "robot_radius" from
   * the given NodeHandle using searchParam() to go up the tree.
   *
   * Calls setFootprint() when successful. */
  void readFootprintFromParams( ros::NodeHandle& nh );

  /** @brief Set the footprint of the robot to be the given set of
   * points, including footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets footprint_spec_ and calls
   * layered_costmap_->setFootprint(). */
  void setFootprint( const std::vector<geometry_msgs::Point>& points );

  void resetOldParameters(ros::NodeHandle& nh);
  void reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level);
  void movementCB(const ros::TimerEvent &event);
  void mapUpdateLoop(double frequency);
  bool map_update_thread_shutdown_;
  bool stop_updates_, initialized_, stopped_, robot_stopped_;
  boost::thread* map_update_thread_;  ///< @brief A thread for updating the map
  ros::Timer timer_;
  ros::Time last_publish_;
  ros::Duration publish_cycle;
  pluginlib::ClassLoader<Layer> plugin_loader_;
  tf::Stamped<tf::Pose> old_pose_;
  Costmap2DPublisher* publisher_;
  dynamic_reconfigure::Server<costmap_2d::Costmap2DConfig> *dsrv_;

  boost::recursive_mutex configuration_mutex_;

  void footprint_cb(const geometry_msgs::Polygon& footprint);
  ros::Subscriber footprint_sub_;
  bool got_footprint_;
  std::vector<geometry_msgs::Point> footprint_spec_; 
  float footprint_padding_;
  costmap_2d::Costmap2DConfig old_config_;
};
// class Costmap2DROS
}// namespace costmap_2d

#endif
