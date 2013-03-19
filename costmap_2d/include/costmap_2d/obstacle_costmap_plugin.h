#ifndef OBSTACLE_COSTMAP_PLUGIN_H_
#define OBSTACLE_COSTMAP_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/plugin_ros.h>
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

namespace common_costmap_plugins
{
  class ObstacleCostmapPlugin : public costmap_2d::CostmapPluginROS, public costmap_2d::Costmap2D
  {
    public:
      ObstacleCostmapPlugin() { costmap_ = NULL; }

      void initialize(costmap_2d::LayeredCostmap* costmap, std::string name);
      void update_bounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
      void update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

      void activate();
      void deactivate();
      bool isDiscretized() { return true; }
      void matchSize(); 


      /**
       * @brief  A callback to handle buffering LaserScan messages
       * @param message The message returned from a message notifier 
       * @param buffer A pointer to the observation buffer to update
       */
      void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message, const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

      /**
       * @brief  A callback to handle buffering PointCloud messages
       * @param message The message returned from a message notifier 
       * @param buffer A pointer to the observation buffer to update
       */
      void pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message, const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);

      /**
       * @brief  A callback to handle buffering PointCloud2 messages
       * @param message The message returned from a message notifier 
       * @param buffer A pointer to the observation buffer to update
       */
      void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message, const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer);
      
      void setResetBounds(double mx0, double mx1, double my0, double my1){
        reset_min_x_=std::min(mx0, reset_min_x_);
        reset_max_x_=std::max(mx1, reset_max_x_);
        reset_min_y_=std::min(my0, reset_min_y_);
        reset_max_y_=std::max(my1, reset_max_y_);
        has_been_reset_ = true;
      }
      

    private:
      void reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level);
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
      void raytraceFreespace(const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y, double* max_x, double* max_y);

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
  };
};
#endif

