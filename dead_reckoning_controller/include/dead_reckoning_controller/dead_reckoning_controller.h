
#ifndef DEAD_RECKONING_CONTROLLER_H_
#define DEAD_RECKONING_CONTROLLER_H_

#include <vector>
#include <Eigen/Core>

#include <srslib_timing/MasterTimingDataRecorder.hpp>

#include <dead_reckoning_controller/DeadReckoningControllerConfig.h>

//for creating a local cost grid
#include <pcl_ros/publisher.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

namespace dead_reckoning_controller {
  /**
   * @class DeadReckoningController
   * @brief A class implementing a dead reckoning controller
   */
  class DeadReckoningController {
    public:
      /**
       * @brief  Constructor for the controller
       * @param name The name of the controller
       * @param costmap_ros A pointer to the costmap instance the controller should use
       * @param global_frame the frame id of the tf frame to use
       */
      DeadReckoningController(std::string name);

      /**
       * @brief  Destructor for the controller
       */
      ~DeadReckoningController() {}

      /**
       * @brief Reconfigures the trajectory planner
       */
      void reconfigure(DeadReckoningControllerConfig &cfg);
      

      double calculateAngularVelocity(tf::Stamped<tf::Pose> current_pose,tf::Stamped<tf::Pose> current_velocity);

      double calculateLinearVelocity(tf::Stamped<tf::Pose> current_pose,tf::Stamped<tf::Pose> current_velocity);

      double distanceFromLine(tf::Stamped<tf::Pose> line_start, tf::Stamped<tf::Pose> line_end, tf::Stamped<tf::Pose> point);

      /**
       * @brief Given the current position and velocity of the robot, find the best velocity to exectue
       * @param global_pose The current position of the robot
       * @param global_vel The current velocity of the robot
       * @param drive_velocities The velocities to send to the robot base
       */
      void computeVelocity(
          tf::Stamped<tf::Pose> global_pose,
          tf::Stamped<tf::Pose> global_vel,
          tf::Stamped<tf::Pose>& drive_velocities);

      /**
       * sets new plan and resets state
       */
      bool setPlan(const tf::Stamped<tf::Pose> start_pose, const tf::Stamped<tf::Pose> end_pose, const tf::Stamped<tf::Pose> current_pose);

      bool isGoalReached(const tf::Stamped<tf::Pose>);

    private:

      std::vector<geometry_msgs::PoseStamped> global_plan_;
      std::vector<geometry_msgs::Point> robot_footprint_;
      tf::Stamped<tf::Pose> start_pose_;
      tf::Stamped<tf::Pose> end_pose_;
      tf::Pose path_;

      DeadReckoningControllerConfig config_;
  };
};
#endif
