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
#ifndef COSTMAP_COSTMAP_2D_PUBLISHER_H_
#define COSTMAP_COSTMAP_2D_PUBLISHER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GridCells.h>

#include <tf/transform_datatypes.h>

#include <boost/thread.hpp>

namespace costmap_2d {
  /**
   * @class Costmap2DPublisher
   * @brief A tool to periodically publish visualization data from a Costmap2D
   */
  class Costmap2DPublisher {
    public:
      /**
       * @brief  Constructor for the Costmap2DPublisher
       * @param  ros_node The node under which to publish the visualization output
       * @param  global_frame The frame in which to publish the visualization output
       */
      Costmap2DPublisher(ros::NodeHandle ros_node, double publish_frequency, std::string global_frame);

      /**
       * @brief  Destructor
       */
      ~Costmap2DPublisher();

      /**
       * @brief  Publishes footprint visualization data over ROS
       */
      void publishFootprint();

      /**
       * @brief  Publishes the visualization data over ROS
       */
      void publishCostmap();

      /**
       * @brief  Update the visualization data from a Costmap2D
       * @param costmap The Costmap2D object to create visualization messages from 
       * @param footprint The footprint of the robot associated with the costmap
       */
      void updateCostmapData(const Costmap2D& costmap, 
          const std::vector<geometry_msgs::Point>& footprint = std::vector<geometry_msgs::Point>(),
          const tf::Stamped<tf::Pose>& global_pose = tf::Stamped<tf::Pose>());

      /**
       * @brief Check if the publisher is active
       * @return True if the frequency for the publisher is non-zero, false otherwise
       */
      bool active() {return active_;}

    private:
      void mapPublishLoop(double frequency);

      std::string global_frame_;
      boost::thread* visualizer_thread_; ///< @brief A thread for publising to the visualizer
      std::vector< std::pair<double, double> > raw_obstacles_, inflated_obstacles_, unknown_space_;
      boost::recursive_mutex lock_; ///< @brief A lock
      bool active_, new_data_;
      ros::Publisher obs_pub_, inf_obs_pub_, unknown_space_pub_, footprint_pub_;
      double resolution_, inscribed_radius_;
      std::vector<geometry_msgs::Point> footprint_;
      tf::Stamped<tf::Pose> global_pose_;
      bool visualizer_thread_shutdown_;
  };
};
#endif
