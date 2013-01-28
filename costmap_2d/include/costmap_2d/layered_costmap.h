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
#ifndef LAYERED_COSTMAP_H_
#define LAYERED_COSTMAP_H_

#include <costmap_2d/cost_values.h>
#include <costmap_2d/plugin_base.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>
#include <pluginlib/class_loader.h>
#include <boost/thread.hpp>
#include <vector>
#include <string>

namespace costmap_2d {
  class CostmapPlugin;

  /**
   * @class LayeredCostmap
   * @brief Instantiates different layer plugins and aggregates them into one score
   */
  class LayeredCostmap {
    public:
      /**
       * @brief  Constructor for a costmap
       */
      LayeredCostmap(std::string name, tf::TransformListener& tf);

      /**
       * @brief  Destructor
       */
      ~LayeredCostmap();

      /**
       * @brief  Update the underlying costmap with new data.
       * If you want to update the map outside of the update loop that runs, you can call this.
       */
      void updateMap();

      /**
       * @brief  The loop that handles updating the costmap
       * @param  frequency The rate at which to run the loop
       */
      void mapUpdateLoop(double frequency);

      /**
       * @brief  Starts costmap updates and activates plugins,
       * can be called to restart the costmap after calls to either
       * stop() or pause()
       */
      void start();

      /**
       * @brief  Stops costmap updates and inactivates plugins
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

      std::string getGlobalFrameID() const {
        return global_frame_;
      }

      std::string getBaseFrameID() const {
        return robot_base_frame_;
      }

      tf::TransformListener* getTFListener() const {
        return &tf_;
      }

      double getTFTolerance() const { return transform_tolerance_; }

      void movementCB(const ros::TimerEvent &event);

      /**
       * @brief Get the pose of the robot in the global frame of the costmap
       * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
       * @return True if the pose was set successfully, false otherwise
       */
      bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;

      void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y);

      void getUpdatedBounds(double& minx, double& miny, double& maxx, double& maxy) {
            minx = minx_; miny = miny_; maxx = maxx_; maxy = maxy_;
      }

      bool isCurrent();
      
      Costmap2D* getCostmap() { return &costmap_; }


    private:
      void updateUsingPlugins(std::vector<boost::shared_ptr<CostmapPlugin> > &plugins);

      Costmap2D costmap_;
      boost::recursive_mutex configuration_mutex_;
      std::string name_;
      bool track_unknown_space_;
      
      void updateOrigin();

      tf::TransformListener& tf_;  /// < @brief Used for transforming point clouds
      std::string tf_prefix_;
      std::string global_frame_;  /// < @brief The global frame for the costmap
      std::string robot_base_frame_;  /// < @brief The frame_id of the robot base
      double transform_tolerance_;  // timeout before transform errors

      bool rolling_window_;  /// < @brief Whether or not the costmap should roll with the robot

      boost::thread* map_update_thread_;  /// < @brief A thread for updating the map
      bool map_update_thread_shutdown_;
      bool stop_updates_, initialized_, stopped_;
      mutable boost::recursive_mutex lock_;

      ros::Timer timer_;
      tf::Stamped<tf::Pose> old_pose_;
      bool robot_stopped_;

      bool current_;
      double minx_, miny_, maxx_, maxy_;

      pluginlib::ClassLoader<CostmapPlugin> plugin_loader_;
      std::vector<boost::shared_ptr<CostmapPlugin> > plugins_;
  };
};  // namespace layered_costmap

#endif
