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
#ifndef COSTMAP_COSTMAP_2D_ROS_H_
#define COSTMAP_COSTMAP_2D_ROS_H_

#include <XmlRpc.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/voxel_costmap_2d.h>
#include <costmap_2d/VoxelGrid.h>
#include <map>
#include <vector>
#include <string>
#include <sstream>

#include <tf/transform_datatypes.h>
#include <tf/message_notifier_base.h>
#include <tf/message_notifier.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/PointCloud.h>

// Thread suppport
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

namespace costmap_2d {

  /**
   * @class Costmap2DROS
   * @brief A ROS wrapper for a 2D Costmap. Handles subscribing to topics that
   * provide observations about obstacles in either the form of PointCloud or LaserScan
   * messages.
   */
  class Costmap2DROS {
    public:
      /**
       * @brief  Constructor for the wrapper
       * @param name The name for this costmap
       * @param tf A reference to a TransformListener
       */
      Costmap2DROS(std::string name, tf::TransformListener& tf);

      /**
       * @brief  Destructor for the wrapper. Cleans up pointers.
       */
      ~Costmap2DROS();

      /**
       * @brief  If you want to manage your own observation buffer you can add
       * it to the costmap. Note, this is somewhat experimental as this feature
       * has not been used enough to have been proven reliable.
       * @param  buffer A shared pointer to your observation buffer
       */
      void addObservationBuffer(const boost::shared_ptr<ObservationBuffer>& buffer);

      /**
       * @brief  Get the observations used to mark space
       * @param marking_observations A reference to a vector that will be populated with the observations 
       * @return True if all the observation buffers are current, false otherwise
       */
      bool getMarkingObservations(std::vector<Observation>& marking_observations);

      /**
       * @brief  Get the observations used to clear space
       * @param marking_observations A reference to a vector that will be populated with the observations 
       * @return True if all the observation buffers are current, false otherwise
       */
      bool getClearingObservations(std::vector<Observation>& clearing_observations);

      /**
       * @brief  Update the underlying costmap with new sensor data. 
       * If you want to update the map outside of the update loop that runs, you can call this.
       */
      void updateMap();

      /**
       * @brief  Given a pose, build the oriented footprint of the robot
       * @param  x The x position of the robot
       * @param  y The y position of the robot
       * @param  theta The orientation of the robot
       * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
       */
      void getOrientedFootprint(double x, double y, double theta, std::vector<geometry_msgs::Point>& oriented_footprint);

      /**
       * @brief Get the pose of the robot in the global frame of the costmap
       * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
       * @return True if the pose was set successfully, false otherwise
       */
      bool getRobotPose(tf::Stamped<tf::Pose>& global_pose);

      /**
       * @brief Clear the footprint of the robot in the costmap
       */
      void clearRobotFootprint();

      /**
       * @brief Clear the footprint of the robot in the costmap at a given pose
       * @param global_pose The pose to clear the footprint at
       */
      void clearRobotFootprint(const tf::Stamped<tf::Pose>& global_pose);

      /**
       * @brief  Set a region in the costmap specified by a convex polygon to a cost
       * @param polygon The polygon affected
       * @param cost_value The cost to apply
       * @return True if the operation was successful, false otherwise
       */
      bool setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value);

      /**
       * @brief  Reset to the static map outside of a window around the robot specified by the user
       * @param size_x The x size of the window to keep unchanged 
       * @param size_y The y size of the window to keep unchanged 
       */
      void resetMapOutsideWindow(double size_x, double size_y);

      /**
       * @brief  Clear all non-lethal obstacles outside of a window around the robot... including cells with NO_INFORMATION
       * @param size_x The x size of the window to keep unchanged 
       * @param size_y The y size of the window to keep unchanged 
       */
      void clearNonLethalWindow(double size_x, double size_y);

      /**
       * @brief  Returns a copy of the underlying costmap
       * @param costmap A reference to the map to populate
       */
      void getCostmapCopy(Costmap2D& costmap);

      /**
       * @brief  Returns the x size of the costmap in cells
       * @return The x size of the costmap in cells
       */
      unsigned int getSizeInCellsX();

      /**
       * @brief  Returns the y size of the costmap in cells
       * @return The y size of the costmap in cells
       */
      unsigned int getSizeInCellsY();

      /**
       * @brief  Returns the resolution of the costmap in meters
       * @return The resolution of the costmap in meters
       */
      double getResolution();

      /**
       * @brief  Returns the global frame of the costmap
       * @return The global frame of the costmap
       */
      std::string getGlobalFrameID();

      /**
       * @brief  Returns the local frame of the costmap
       * @return The local frame of the costmap
       */
      std::string getBaseFrameID();

      /**
       * @brief  Returns the inscribed radius of the costmap
       * @return The inscribed radius of the costmap
       */
      double getInscribedRadius();

      /**
       * @brief  Returns the circumscribed radius of the costmap
       * @return The circumscribed radius of the costmap
       */
      double getCircumscribedRadius();

      /**
       * @brief  Returns the inflation radius of the costmap
       * @return The inflation radius of the costmap
       */
      double getInflationRadius();

      /**
       * @brief  Returns the footprint of the robot in the robot_base_frame. To get the footprint in the global_frame use getOrientedFootprint
       * @return The footprint of the robot in the robot_base_frame
       */
      std::vector<geometry_msgs::Point> getRobotFootprint();

      /**
       * @brief  Check if the observation buffers for the cost map are current
       * @return True if the buffers are current, false otherwise
       */
      bool isCurrent() {return current_;}

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
      void pause() {
        stop_updates_ = true;
        initialized_ = false;
      }


      /**
       * @brief  Resumes costmap updates
       */
      void resume(){
        stop_updates_ = false;

        //block until the costmap is re-initialized.. meaning one update cycle has run
        ros::Rate r(100.0);
        while(!initialized_)
          r.sleep();
      }

    private:
      /**
       * @brief  A callback to handle buffering LaserScan messages
       * @param message The message returned from a message notifier 
       * @param buffer A pointer to the observation buffer to update
       */
      void laserScanCallback(const tf::MessageNotifier<sensor_msgs::LaserScan>::MessagePtr& message, const boost::shared_ptr<ObservationBuffer>& buffer);

      /**
       * @brief  A callback to handle buffering PointCloud messages
       * @param message The message returned from a message notifier 
       * @param buffer A pointer to the observation buffer to update
       */
      void pointCloudCallback(const tf::MessageNotifier<sensor_msgs::PointCloud>::MessagePtr& message, const boost::shared_ptr<ObservationBuffer>& buffer);

      /**
       * @brief  The loop that handles updating the costmap
       * @param  frequency The rate at which to run the loop
       */
      void mapUpdateLoop(double frequency);

      /**
       * @brief  Grab the footprint of the robot from the parameter server if available
       */
      std::vector<geometry_msgs::Point> loadRobotFootprint(ros::NodeHandle node, double inscribed_radius, double circumscribed_radius);

      ros::NodeHandle ros_node_; ///< @brief The ros node to use
      tf::TransformListener& tf_; ///< @brief Used for transforming point clouds
      laser_geometry::LaserProjection projector_; ///< @brief Used to project laser scans into point clouds
      Costmap2D* costmap_; ///< @brief The underlying costmap to update
      std::string global_frame_; ///< @brief The global frame for the costmap
      std::string robot_base_frame_; ///< @brief The frame_id of the robot base
      boost::thread* map_update_thread_; ///< @brief A thread for updating the map

      std::vector<boost::shared_ptr<tf::MessageNotifierBase> > observation_notifiers_; ///< @brief Used to make sure that transforms are available for each sensor
      std::vector<boost::shared_ptr<ObservationBuffer> > observation_buffers_; ///< @brief Used to store observations from various sensors
      std::vector<boost::shared_ptr<ObservationBuffer> > marking_buffers_; ///< @brief Used to store observation buffers used for marking obstacles
      std::vector<boost::shared_ptr<ObservationBuffer> > clearing_buffers_; ///< @brief Used to store observation buffers used for clearing obstacles
      bool rolling_window_; ///< @brief Whether or not the costmap should roll with the robot
      bool current_; ///< @brief Whether or not all the observation buffers are updating at the desired rate
      double transform_tolerance_; // timeout before transform errors
      Costmap2DPublisher* costmap_publisher_;
      bool stop_updates_, initialized_, stopped_;
      bool publish_voxel_;
      std::vector<geometry_msgs::Point> footprint_spec_;
      ros::Publisher voxel_pub_;
      boost::recursive_mutex lock_;

  };
};

#endif

