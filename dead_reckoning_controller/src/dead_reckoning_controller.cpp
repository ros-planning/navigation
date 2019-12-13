
#include <srslib_timing/ScopedTimingSampleRecorder.hpp>

#include <dead_reckoning_controller/dead_reckoning_controller.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/map_grid_cost_point.h>
#include <base_local_planner/geometry_math_helpers.h>
#include <cmath>

//for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>

namespace dead_reckoning_controller {
  void DeadReckoningController::reconfigure(DeadReckoningControllerConfig &config)
  {

  }

  DeadReckoningController::DeadReckoningController(std::string name, base_local_planner::LocalPlannerUtil *planner_util) :
      planner_util_(planner_util)
  {
    ros::NodeHandle private_nh("~/" + name);
  }

  void DeadReckoningController::setFootprintSpec(const std::vector<geometry_msgs::Point>& footprint_spec)
  {
    ROS_INFO("Get footprint from DWAPlannerROS");
    robot_footprint_ = footprint_spec;
  }

  bool DeadReckoningController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    double divergenceDistance = planner_util_->distanceToPlanDivergence(orig_global_plan);
    if (divergenceDistance >= 0 && divergenceDistance < oscillation_reset_plan_divergence_distance_)
    {
      ROS_DEBUG("flag reset due to set plan at range %f", divergenceDistance);
      oscillation_costs_.resetOscillationFlags();
    }
    
    return planner_util_->setPlan(orig_global_plan);
  }

  base_local_planner::Trajectory computeVelocity(
          tf::Stamped<tf::Pose> global_pose,
          tf::Stamped<tf::Pose> global_vel,
          tf::Stamped<tf::Pose>& drive_velocities) {
			  
	double error_prop = dotProduct of global_vel to y axis
	double integral_error = distance from robot to y axis dou
	double derivative of y axis vel = angular velocity ?
	
	
  }


};
