
#ifndef DEAD_RECKONING_CONTROLLER_ROS_H_
#define DEAD_RECKONING_CONTROLLER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>

#include <base_local_planner/odometry_helper_ros.h>
#include <srslib_timing/MasterTimingDataRecorder.hpp>
#include <srslib_timing/RollingTimingStatisticsCalculator.hpp>

#include <dead_reckoning_controller/dead_reckoning_controller.h>
#include <dead_reckoning_controller/DeadReckoningControllerConfig.h>

#include <atomic>

namespace dead_reckoning_controller {
  /**
   * @class DeadReckoningController
   * @brief ROS Wrapper for the DeadReckoningController that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class DeadReckoningControllerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Constructor for DeadReckoningControllerROS wrapper
       */
      DeadReckoningControllerROS();

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the controller
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~DeadReckoningControllerROS();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();



      bool isInitialized() {
        return initialized_;
      }
      
      /**
       * @brief  Cancel the current controller thread
       * @return True if achieved, false otherwise
       */
      bool cancel();

    private:
      /**
       * @brief Callback to update the local planner's parameters based on dynamic reconfigure
       */
      void reconfigureCB(DeadReckoningControllerConfig &config, uint32_t level);

      void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      tf::TransformListener* tf_; ///< @brief Used for transforming point clouds

      // for visualisation, publishers of global and local plan
      ros::Publisher g_plan_pub_, l_plan_pub_;

      base_local_planner::LocalPlannerUtil planner_util_;

      boost::shared_ptr<DeadReckoningController> dp_; ///< @brief The trajectory controller

      costmap_2d::Costmap2DROS* costmap_ros_;

      dead_reckoning_controller::DeadReckoningControllerConfig default_config_;

      bool setup_;
      tf::Stamped<tf::Pose> current_pose_;

      base_local_planner::LatchedStopRotateController latchedStopRotateController_;
      bool initialized_;


      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;

      srs::MasterTimingDataRecorder tdr_;
      srs::RollingTimingStatisticsCalculator loopTimingStatistics_;

      std::shared_ptr<dynamic_reconfigure::Server<DeadReckoningControllerConfig>> configServer_;

      std::atomic<bool> canceled_;
  };
};
#endif
