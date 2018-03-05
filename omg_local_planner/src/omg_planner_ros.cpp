#include <pluginlib/class_list_macros.h>

#include <omg_local_planner/omg_planner_ros.h>

#include <omg_ros_nav_bridge/GoalReached.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(omg_local_planner::OMGPlannerROS,
                       nav_core::BaseLocalPlanner)

namespace omg_local_planner {
OMGPlannerROS::OMGPlannerROS() : initialized_(false), odom_helper_("odom") {}

void OMGPlannerROS::initialize(std::__cxx11::string name,
                               tf::TransformListener *tf,
                               costmap_2d::Costmap2DROS *costmap_ros) {
  if (!initialized_) {
    ros::NodeHandle private_nh("~/" + name);
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ros_->getRobotPose(current_pose_);

    // Initialise the service clients.
    ros::NodeHandle public_nh;
    ros::service::waitForService(goal_reached_srv_, -1);
    goal_reached_client_ =
        public_nh.serviceClient<omg_ros_nav_bridge::GoalReached>(
            goal_reached_srv_, true);
    // TODO: Add the other services and call initialise
    initialized_ = true;
  }
}

OMGPlannerROS::~OMGPlannerROS() {}

bool OMGPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("Could not get robot pose");
    return false;
  }

  if (compute_velocity_client_) {
    // TODO: Add call to service
  } else {
    ROS_ERROR_STREAM("compute_velocity_client_ is not connected.");
  }
}

bool OMGPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
  if (!isInitialized()) {
    ROS_ERROR(
        "This planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }

  ROS_INFO("Got new plan");
  if (set_plan_client_) {
    // TODO: Add call to service
  } else {
    ROS_ERROR_STREAM("set_plan_client_ is not connected.");
  }
}

bool OMGPlannerROS::isGoalReached() {
  if (!isInitialized()) {
    ROS_ERROR(
        "This planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }

  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("Could not get robot pose");
    return false;
  }

  if (goal_reached_client_) {
    omg_ros_nav_bridge::GoalReached srv;
    if (goal_reached_client_.call(srv)) {
      return srv.response.reached;
    } else {
      ROS_ERROR("Failed to call service goal_reached");
      return 1;
    }
  } else {
    ROS_ERROR_STREAM("goal_reached_client_ is not connected.");
  }
}

bool OMGPlannerROS::isInitialized() { return initialized_; }

void OMGPlannerROS::publishLocalPlan(
    std::vector<geometry_msgs::PoseStamped> &path) {
  base_local_planner::publishPlan(path, l_plan_pub_);
}

void OMGPlannerROS::publishGlobalPlan(
    std::vector<geometry_msgs::PoseStamped> &path) {
  base_local_planner::publishPlan(path, g_plan_pub_);
}
}
