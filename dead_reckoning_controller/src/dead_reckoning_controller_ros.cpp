#include <dead_reckoning_controller/dead_reckoning_controller_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

#include <srslib_timing/ScopedTimingSampleRecorder.hpp>
#include <srslib_timing/ScopedRollingTimingStatistics.hpp>

#include <std_msgs/Int32.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dead_reckoning_controller::DeadReckoningControllerROS, nav_core::BaseLocalPlanner)

namespace dead_reckoning_controller {

  void DeadReckoningControllerROS::reconfigureCB(DeadReckoningControllerConfig &main_config, uint32_t level) {

      DeadReckoningControllerConfig config = main_config;

      odom_helper_.setAccelerationRates(config.acc_lim_x, config.acc_lim_theta);
      // odom_helper_.setWheelbase(config.wheelbase);

      // update latched stop rotate controller configuration
      ROS_INFO("Updating latching to  %d", config.latch_xy_goal_tolerance);
      latchedStopRotateController_.setLatch(config.latch_xy_goal_tolerance);

      // update dwa specific configuration
      dp_->reconfigure(config);
      canceled_ = false;
  }

  DeadReckoningControllerROS::DeadReckoningControllerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false),
      tdr_("DWAPlannerROS")
      {

  }

  void DeadReckoningControllerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if (! isInitialized()) {
      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);
      planner_util_.initialize(tf, costmap_ros_, costmap_ros_->getGlobalFrameID(), name);

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DeadReckoningController>(new DeadReckoningController(name, &planner_util_));

      // set up footprint when initialization
      dp_->setFootprintSpec(costmap_ros->getRobotFootprint());

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }
      configServer_ = std::make_shared<dynamic_reconfigure::Server<DeadReckoningControllerConfig>>(private_nh);
      configServer_->setCallback(boost::bind(&DeadReckoningControllerROS::reconfigureCB, this, _1, _2));
      initialized_ = true;

    }
    else{
      ROS_WARN("This controller has already been initialized, doing nothing.");
    }
  }

  bool DeadReckoningControllerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
	if (orig_global_plan.size() != 1) {
		ROS_ERROR("PLane is not one point, failing")
		return false;
	}
    return dp_->setPlan(orig_global_plan);
  }

  bool DeadReckoningControllerROS::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    if(dp_->isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      ROS_INFO("Goal reached");
      return true;
    } else {
      return false;
    }
  }

  void DeadReckoningControllerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }

  void DeadReckoningControllerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  DWAPlannerROS::~DWAPlannerROS(){
  }

  bool DeadReckoningControllerROS::deadReckoningComputeVelocityCommands(tf::Stamped<tf::Pose> &global_pose, tf::Stamped<tf::Pose>& robot_vel,
    geometry_msgs::Twist& cmd_vel) {
    // dynamic window sampling approach to get useful velocity commands
    if(! isInitialized()){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> local_plan;

    //compute what trajectory to drive along
    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = costmap_ros_->getBaseFrameID();

    base_local_planner::Trajectory path = dp_->computeVelocity(global_pose, robot_vel, drive_cmds);

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

    odom_helper_.setCmdVel(cmd_vel);

    //if we cannot move... tell someone
    if(path.cost_ < 0) {
      ROS_DEBUG_NAMED("dead_reckoning_controller",
          "The DeadReckoningController failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      publishLocalPlan(local_plan);

      return false;
    }

    ROS_DEBUG_NAMED("dead_reckoning_controller", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

	double p_x, p_y, p_th;
	path.getPoint(0, p_x, p_y, p_th);

	tf::Stamped<tf::Pose> p =
			tf::Stamped<tf::Pose>(tf::Pose(
					tf::createQuaternionFromYaw(p_th),
					tf::Point(p_x, p_y, 0.0)),
					ros::Time::now(),
					costmap_ros_->getGlobalFrameID());
	geometry_msgs::PoseStamped pose;
	tf::poseStampedTFToMsg(p, pose);
	local_plan.push_back(pose);

    //publish information to the visualizer

    publishLocalPlan(local_plan);
    return true;
  }

  bool DeadReckoningControllerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {

	if (canceled_) {
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.angular.z = 0.0;
		odom_helper_.setCmdVel(cmd_vel);
		ROS_INFO("Cancelled called on DeadReckoningPlanner.");
		canceled_ = false;
      	return false;
    }

    odom_helper_.setForwardEstimationTime(0.0);
    odom_helper_.getEstimatedOdomPose(current_pose_);

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
      ROS_ERROR("Could not get local plan");
      return false;
    }
    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) {
      ROS_WARN_NAMED("dwa_local_planner", "Received an empty transformed plan.");
      return false;
    }
    ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getEstimatedRobotVel(robot_vel);

    dp_->updatePlan(current_pose_, robot_vel, transformed_plan);

    if (dp_->.isPositionReached(&planner_util_, current_pose_)) {
      return true;
    } else {
      bool isOk = deadReckoningComputeVelocityCommands(current_pose_, robot_vel, cmd_vel);
      if (isOk) {
        publishGlobalPlan(transformed_plan);
      } else {
        ROS_DEBUG_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }
      return isOk;
    }
  }

  bool DeadReckoningControllerROS::cancel() {
	canceled_ = true;
	return true;
  }

};
