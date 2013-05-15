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
#ifndef OBSTACLE_COSTMAP_PLUGIN_H_
#define OBSTACLE_COSTMAP_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/ObstaclePluginConfig.h>

namespace costmap_2d
{
class ObstacleLayer : public Layer, public Costmap2D
{
public:
  ObstacleLayer()
  {
    costmap_ = NULL; // this is the unsigned char* member of parent class Costmap2D.
  }

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void activate();
  virtual void deactivate();
  bool isDiscretized()
  {
    return true;
  }
  virtual void matchSize();

  /**
   * @brief  A callback to handle buffering LaserScan messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                         const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

  /**
   * @brief  A callback to handle buffering PointCloud messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                          const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

  /**
   * @brief  A callback to handle buffering PointCloud2 messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                           const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

  void setResetBounds(double mx0, double mx1, double my0, double my1)
  {
    reset_min_x_ = std::min(mx0, reset_min_x_);
    reset_max_x_ = std::max(mx1, reset_max_x_);
    reset_min_y_ = std::min(my0, reset_min_y_);
    reset_max_y_ = std::max(my1, reset_max_y_);
    has_been_reset_ = true;
  }

protected:
  void initMaps();

  /**
   * @brief  Get the observations used to mark space
   * @param marking_observations A reference to a vector that will be populated with the observations
   * @return True if all the observation buffers are current, false otherwise
   */
  bool getMarkingObservations(std::vector<costmap_2d::Observation>& marking_observations) const;

  /**
   * @brief  Get the observations used to clear space
   * @param marking_observations A reference to a vector that will be populated with the observations
   * @return True if all the observation buffers are current, false otherwise
   */
  bool getClearingObservations(std::vector<costmap_2d::Observation>& clearing_observations) const;

  /**
   * @brief  Clear freespace based on one observation
   * @param clearing_observation The observation used to raytrace
   */
  virtual void raytraceFreespace(const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y,
                                 double* max_x, double* max_y);

  std::string global_frame_; ///< @brief The global frame for the costmap
  double max_obstacle_height_; ///< @brief Max Obstacle Height

  laser_geometry::LaserProjection projector_; ///< @brief Used to project laser scans into point clouds

  std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_; ///< @brief Used to make sure that transforms are available for each sensor
  std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_; ///< @brief Used for the observation message filters
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > observation_buffers_; ///< @brief Used to store observations from various sensors
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > marking_buffers_; ///< @brief Used to store observation buffers used for marking obstacles
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > clearing_buffers_; ///< @brief Used to store observation buffers used for clearing obstacles

  bool rolling_window_;
  dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig> *dsrv_;

  bool has_been_reset_;
  double reset_min_x_, reset_max_x_, reset_min_y_, reset_max_y_;

private:
  void reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level);

};
}
#endif

