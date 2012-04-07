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

#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/observation_buffer.h>

#include <nav_msgs/OccupancyGrid.h>

#include <vector>
#include <string>


#include <boost/algorithm/string.hpp>

#include <tf/transform_datatypes.h>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/PointCloud.h>

//Support for PointCloud2 messages
#include <sensor_msgs/PointCloud2.h>

// Thread suppport
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <stdlib.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/Costmap2DConfig.h>

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
      bool getMarkingObservations(std::vector<Observation>& marking_observations) const;

      /**
       * @brief  Get the observations used to clear space
       * @param marking_observations A reference to a vector that will be populated with the observations 
       * @return True if all the observation buffers are current, false otherwise
       */
      bool getClearingObservations(std::vector<Observation>& clearing_observations) const;

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
      void getOrientedFootprint(double x, double y, double theta, std::vector<geometry_msgs::Point>& oriented_footprint) const;

      /**
       * @brief  Build the oriented footprint of the robot at the robot's current pose
       * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
       */
      void getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const;

      /**
       * @brief Get the pose of the robot in the global frame of the costmap
       * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
       * @return True if the pose was set successfully, false otherwise
       */
      bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;

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
      void getCostmapCopy(Costmap2D& costmap) const;

      /**
       * @brief  Updates the costmap's static map with new information
       * @param new_map The map to put into the costmap. The origin of the new
       * map along with its size will determine what parts of the costmap's
       * static map are overwritten.
       */
      void updateStaticMap(const nav_msgs::OccupancyGrid& new_map);

      /**
       * @brief  Get a copy of a window of the costmap centered at the robot's
       * position. If the requested size of the window does not fit within the
       * bounds of the costmap, the window is truncated automatically to fit.
       * @param win_size_x The x size of the desired window in meters
       * @param win_size_y The y size of the desired window in meters
       * @param costmap A reference to the map to populate
       */
      void getCostmapWindowCopy(double win_size_x, double win_size_y, Costmap2D& costmap) const;

      /**
       * @brief  Get a copy of a window of the costmap centered at a given
       * position. If the requested size of the window does not fit within the
       * bounds of the costmap, the window is truncated automatically to fit.
       * @param win_center_x The x center point of the desired window in meters
       * @param win_center_y The y center point of the desired window in meters
       * @param win_size_x The x size of the desired window in meters
       * @param win_size_y The y size of the desired window in meters
       * @param costmap A reference to the map to populate
       */
      void getCostmapWindowCopy(double win_center_x, double win_center_y, double win_size_x, double win_size_y, Costmap2D& costmap) const;

      /**
       * @brief  Returns the x size of the costmap in cells
       * @return The x size of the costmap in cells
       */
      unsigned int getSizeInCellsX() const;

      /**
       * @brief  Returns the y size of the costmap in cells
       * @return The y size of the costmap in cells
       */
      unsigned int getSizeInCellsY() const;

      /**
       * @brief  Returns the resolution of the costmap in meters
       * @return The resolution of the costmap in meters
       */
      double getResolution() const;

      /**
       * @brief  Returns the global frame of the costmap
       * @return The global frame of the costmap
       */
      std::string getGlobalFrameID() const;

      /**
       * @brief  Returns the local frame of the costmap
       * @return The local frame of the costmap
       */
      std::string getBaseFrameID() const;

      /**
       * @brief  Returns the inscribed radius of the costmap
       * @return The inscribed radius of the costmap
       */
      double getInscribedRadius() const;

      /**
       * @brief  Returns the circumscribed radius of the costmap
       * @return The circumscribed radius of the costmap
       */
      double getCircumscribedRadius() const;

      /**
       * @brief  Returns the inflation radius of the costmap
       * @return The inflation radius of the costmap
       */
      double getInflationRadius() const;

      /**
       * @brief  Returns the footprint of the robot in the robot_base_frame. To get the footprint in the global_frame use getOrientedFootprint
       * @return The footprint of the robot in the robot_base_frame
       */
      std::vector<geometry_msgs::Point> getRobotFootprint() const;

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
       * @brief Callback for dynamic_reconfigure
       */
      void reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level);

      void movementCB(const ros::TimerEvent &event);

      /**
       * @brief  Callback to update the costmap's map from the map_server
       * @param new_map The map to put into the costmap. The origin of the new
       * map along with its size will determine what parts of the costmap's
       * static map are overwritten.
       */
      void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);

      /**
       * @brief  Initialize static_data from a map
       * @param new_map The map to initialize from
       */
      void initFromMap(const nav_msgs::OccupancyGrid& map);

      /**
       * @brief  A callback to handle buffering LaserScan messages
       * @param message The message returned from a message notifier 
       * @param buffer A pointer to the observation buffer to update
       */
      void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message, const boost::shared_ptr<ObservationBuffer>& buffer);

      /**
       * @brief  A callback to handle buffering PointCloud messages
       * @param message The message returned from a message notifier 
       * @param buffer A pointer to the observation buffer to update
       */
      void pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message, const boost::shared_ptr<ObservationBuffer>& buffer);

      /**
       * @brief  A callback to handle buffering PointCloud2 messages
       * @param message The message returned from a message notifier 
       * @param buffer A pointer to the observation buffer to update
       */
      void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message, const boost::shared_ptr<ObservationBuffer>& buffer);

      /**
       * @brief  The loop that handles updating the costmap
       * @param  frequency The rate at which to run the loop
       */
      void mapUpdateLoop(double frequency);

      /**
       * @brief  Grab the footprint of the robot from the parameter server if available
       */
      std::vector<geometry_msgs::Point> loadRobotFootprint(ros::NodeHandle node, double inscribed_radius, double circumscribed_radius);

      /**
       * @brief  Return the shortest distance from a point to a line segment
       */
      double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1);

      inline double distance(double x0, double y0, double x1, double y1){
        return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
      }

      std::string name_;
      tf::TransformListener& tf_; ///< @brief Used for transforming point clouds
      laser_geometry::LaserProjection projector_; ///< @brief Used to project laser scans into point clouds
      Costmap2D* costmap_; ///< @brief The underlying costmap to update
      std::string global_frame_; ///< @brief The global frame for the costmap
      std::string robot_base_frame_; ///< @brief The frame_id of the robot base
      boost::thread* map_update_thread_; ///< @brief A thread for updating the map

      std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_; ///< @brief Used to make sure that transforms are available for each sensor
      std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_; ///< @brief Used for the observation message filters
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
      mutable boost::recursive_mutex lock_;
      bool map_update_thread_shutdown_;
      bool save_debug_pgm_;
      ros::Subscriber map_sub_;
      bool map_initialized_;
      std::string tf_prefix_;

      //we need this to be able to initialize the map using a latched topic approach
      //strictly speaking, we don't need the lock, but since this all happens on startup
      //and there is little overhead... we'll be careful and use it anyways just in case
      //the compiler or scheduler does something weird with the code
      boost::recursive_mutex map_data_lock_;
      nav_msgs::MapMetaData map_meta_data_;
      std::vector<unsigned char> input_data_;
      bool costmap_initialized_;

      bool robot_stopped_, setup_, static_map_;

      dynamic_reconfigure::Server<costmap_2d::Costmap2DConfig> *dsrv_;
      boost::mutex map_update_mutex_;
      boost::recursive_mutex configuration_mutex_;
      costmap_2d::Costmap2DConfig last_config_;
      costmap_2d::Costmap2DConfig maptype_config_;
      costmap_2d::Costmap2DConfig default_config_;
      ros::Timer timer_;
      tf::Stamped<tf::Pose> old_pose_;
  };
};

#endif

