/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2017, 6 River Systems, Inc.
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
 *         Daniel Grieneisen
 *********************************************************************/
#ifndef COSTMAP_2D_OBSTRUCTION_LAYER_H_
#define COSTMAP_2D_OBSTRUCTION_LAYER_H_

#include <cmath>
#include <memory>
#include <cstddef>

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
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
#include <costmap_2d/ObstructionPluginConfig.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/ObstructionMsg.h>
#include <costmap_2d/ObstructionListMsg.h>
#include <costmap_2d/obstruction.h>
#include <costmap_2d/kernel.h>

#include <srslib_timing/MasterTimingDataRecorder.hpp>


namespace costmap_2d
{

/**
 * Helper class used to clear cells in the obstruction map when ray tracing.
 */
class ClearObstructionCell
{
public:
  /**
   * Constructor
   * @param costmap The beginning of the obstruction map array
   */
  ClearObstructionCell(std::weak_ptr<Obstruction>* costmap) :
      costmap_(costmap)
  {
  }

  /**
   * Operator used by the ray tracer.
   * @param offset The index into the array.
   */
  inline void operator()(unsigned int offset)
  {
    auto obs = costmap_[offset].lock();
    if (obs && !obs->seen_this_cycle_)
    {
      obs->cleared_ = true;
      costmap_[offset].reset();
    }
  }
private:
  std::weak_ptr<Obstruction>* costmap_ = nullptr;
};

/**
 * @brief a costmap layer which tracks obstructions
 * This layer tracks obstructions, which are obstacles that are seen by a sensor but
 * are not in the static map.  Obstructions are tracked and placed into the costmap
 * with some expansion (defined by a kernel).  The obstructions decay over time, until
 * they are completely removed.
 *
 * Much of this layer is the same as the obstacle layer.
 */
class ObstructionLayer : public CostmapLayer
{
public:
  ObstructionLayer() : timingDataRecorder_("ob")
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
    obstruction_map_ = NULL;
  }

  virtual ~ObstructionLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  /**
   * @brief  A callback to handle buffering LaserScan messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                         const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

   /**
    * @brief A callback to handle buffering LaserScan messages which need filtering to turn Inf values into range_max.
    * @param message The message returned from a message notifier
    * @param buffer A pointer to the observation buffer to update
    */
  void laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& message,
                                 const boost::shared_ptr<ObservationBuffer>& buffer);

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

  // for testing purposes
  void addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing);
  void clearStaticObservations(bool marking, bool clearing);


  virtual void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);

  virtual void updateOrigin(double new_origin_x, double new_origin_y);

protected:
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

  /**
   * @brief  Get the observations used to mark space
   * @param marking_observations A reference to a vector that will be populated with the observations
   * @return True if all the observation buffers are current, false otherwise
   */
  bool getMarkingObservations(std::vector<costmap_2d::Observation>& marking_observations) const;

  /**
   * @brief  Get the observations used to clear space
   * @param clearing_observations A reference to a vector that will be populated with the observations
   * @return True if all the observation buffers are current, false otherwise
   */
  bool getClearingObservations(std::vector<costmap_2d::Observation>& clearing_observations) const;

  /**
   * @brief  Clear freespace based on one observation
   * @param clearing_observation The observation used to raytrace
   * @param min_x
   * @param min_y
   * @param max_x
   * @param max_y
   */
  virtual void raytraceFreespace(const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y,
                                 double* max_x, double* max_y);

  /**
   * Check if the observation has any new obstructions and track them if necessary.
   * Also update the bounds based on the obstructions.
   * @param observation The observation to check
   * @param min_x
   * @param min_y
   * @param max_x
   * @param max_y
   */
  virtual void checkObservation(const costmap_2d::Observation& observation, double* min_x, double* min_y,
                                 double* max_x, double* max_y);

  /**
   * Update the obstructions.  Clear out old ones, change levels, and publish a list of
   * all of them.
   * @param min_x
   * @param min_y
   * @param max_x
   * @param max_y
   */
  virtual void updateObstructions(double* min_x, double* min_y, double* max_x, double* max_y);

  /**
   * Apply the selected kernel to the grid at the given x, y location.
   */
  virtual void applyKernelAtLocation(std::shared_ptr<Kernel> kernel, float x, float y, Costmap2D& master_grid);


  void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double* min_x, double* min_y,
                            double* max_x, double* max_y);

  std::vector<geometry_msgs::Point> transformed_footprint_;
  bool footprint_clearing_enabled_;
  void updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                       double* max_x, double* max_y);


  /**
   * @brief  Deletes the costmap, static_map, and markers data structures
   */
  virtual void deleteMaps();

  /**
   * @brief  Resets the obstruction map, but does not mark the obstructions as cleared.
   */
  virtual void resetMaps();

  /**
   * @brief Resets the obstruction map and marks them as cleared.
   */
  virtual void resetMapsAndClearObstructions();

  /**
   * @brief  Initializes the costmap, static_map, and markers data structures
   * @param size_x The x size to use for map initialization
   * @param size_y The y size to use for map initialization
   */
  virtual void initMaps(unsigned int size_x, unsigned int size_y);

  /**
   * Updates to run when the footprint has changed.
   */
  virtual void onFootprintChanged();

  /**
   * Clears an individual grid cell
   * @param x x index
   * @param y y index
   */
  virtual void clearGridCell(unsigned int x, unsigned int y);

  void setInflationParameters(double inflation_radius, double cost_scaling_factor);

  // Generate all of the kernels
  void generateKernels();

  void generateKernelsByType(ObstructionType type, float inflation_radius,
    float cost_scaling_factor, int inflation_type);

  ObstructionType getObstructionTypeFromIndex(unsigned int index);

  ObstructionType getObstructionType(double px, double py);

  std::string global_frame_;  ///< @brief The global frame for the costmap
  double max_obstacle_height_;  ///< @brief Max Obstacle Height

  laser_geometry::LaserProjection projector_;  ///< @brief Used to project laser scans into point clouds

  std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;  ///< @brief Used for the observation message filters
  std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_;  ///< @brief Used to make sure that transforms are available for each sensor
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > observation_buffers_;  ///< @brief Used to store observations from various sensors
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > marking_buffers_;  ///< @brief Used to store observation buffers used for marking obstacles
  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > clearing_buffers_;  ///< @brief Used to store observation buffers used for clearing obstacles

  // Used only for testing purposes
  std::vector<costmap_2d::Observation> static_clearing_observations_, static_marking_observations_;

  bool rolling_window_;
  dynamic_reconfigure::Server<costmap_2d::ObstructionPluginConfig> *dsrv_;

  // Map for keeping track of obstructions
  std::weak_ptr<Obstruction>* obstruction_map_;
  std::list<std::shared_ptr<Obstruction>> obstruction_list_;

  std::map<ObstructionType, std::vector<std::shared_ptr<Kernel>>> kernels_; // map of vector of kernels for different obstruction levels

  bool enable_decay_ = true; // Enalbe the decay of obstructions
  ros::Duration obstruction_half_life_ = ros::Duration(1); // The time to wait before decrementing the obstruction level by half.
  unsigned int num_obstruction_levels_ = 10;  // The number of levels the obstruction should go through before disappearing

  enum inflation_type {
    EXPONENTIAL_INFLATION = 0,
    TRINOMIAL_INFLATION = 1
  };

  float ss_inflation_radius_ = 1;
  float ss_cost_scaling_factor_ = 1;

  int ss_inflation_type_ = EXPONENTIAL_INFLATION;

  float dyn_inflation_radius_ = 1;
  float dyn_cost_scaling_factor_ = 1;
  int dyn_inflation_type_ = EXPONENTIAL_INFLATION;

  double distance_threshold_ = 0.3;

  std::shared_ptr<std::vector<double>> static_distance_map_;

  ros::Publisher obstruction_publisher_;  // Publisher of obstruction data

private:
  void reconfigureCB(costmap_2d::ObstructionPluginConfig &config, uint32_t level);

  // Add timing data recorder
  srs::MasterTimingDataRecorder timingDataRecorder_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_OBSTRUCTION_LAYER_H_
