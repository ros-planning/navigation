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

  void DeadReckoningControllerROS::reconfigureCB(DeadReckoningControllerConfig &main_config, uint32_t level)
  {

      DeadReckoningControllerConfig config = main_config;

      odom_helper_.setAccelerationRates(config.acc_lim_x, config.acc_lim_theta);

      // update latched stop rotate controller configuration

      // update dwa specific configuration
      drc_->reconfigure(config);
      canceled_ = false;
  }

  DeadReckoningControllerROS::DeadReckoningControllerROS() : initialized_(false),
      odom_helper_("odom")
  {

  }

  DeadReckoningControllerROS::~DeadReckoningControllerROS(){
  }

  void DeadReckoningControllerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros) 
  {
    if (! isInitialized())
    {
      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      drc_ = boost::shared_ptr<DeadReckoningController>(new DeadReckoningController(name));

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }
      configServer_ = std::make_shared<dynamic_reconfigure::Server<DeadReckoningControllerConfig>>(private_nh);
      configServer_->setCallback(boost::bind(&DeadReckoningControllerROS::reconfigureCB, this, _1, _2));
      initialized_ = true;

    }
    else
    {
      ROS_WARN("This controller has already been initialized, doing nothing.");
    }
  }

  bool DeadReckoningControllerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
  {
    if (! isInitialized()) 
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if (orig_global_plan.size() != 1)
    {
      ROS_ERROR("Plan is not one point, failing");
      return false;
    }
    costmap_ros_->getRobotPose(current_pose_);

    tf::Stamped<tf::Pose> goal_pose;
    tf::poseStampedMsgToTF(orig_global_plan[0], goal_pose);

    //get goal pose in costmap Frame ID
    tf::Stamped<tf::Pose> end_point;
    tf::Stamped<tf::Pose> global_pose_time_zero(goal_pose, ros::Time(0), goal_pose.frame_id_);
    tf_->transformPose(costmap_ros_->getGlobalFrameID(), global_pose_time_zero, end_point);

    //translate point back in x to get a path vector
    tf::Pose offsetTransform;
    offsetTransform.setOrigin( tf::Vector3(-1.0, 0, 0));
    offsetTransform.setRotation( tf::Quaternion(0,0,0,1));

    //create start point of path vector from offset
    tf::Stamped<tf::Pose> start_point(end_point * offsetTransform, end_point.stamp_, end_point.frame_id_);

    std::vector<geometry_msgs::PoseStamped> transformedPlan;
    geometry_msgs::PoseStamped start_point_msg;
    tf::poseStampedTFToMsg(start_point, start_point_msg);

    geometry_msgs::PoseStamped end_point_msg;
    tf::poseStampedTFToMsg(end_point, end_point_msg);

    transformedPlan.push_back(start_point_msg);
    transformedPlan.push_back(end_point_msg);

    publishGlobalPlan(transformedPlan);

    return drc_->setPlan(start_point, end_point, current_pose_);
  }

  bool DeadReckoningControllerROS::isGoalReached()
  {
    if (! isInitialized()) 
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_))
    {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    if(drc_->isGoalReached(current_pose_))
    {
      ROS_INFO("Goal reached DRC");
      return true;
    } else {
      return false;
    }
  }

  void DeadReckoningControllerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path)
  {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }

  void DeadReckoningControllerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path)
  {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  bool DeadReckoningControllerROS::deadReckoningComputeVelocityCommands(tf::Stamped<tf::Pose> &global_pose, tf::Stamped<tf::Pose>& robot_vel,
    geometry_msgs::Twist& cmd_vel)
  {
    // dynamic window sampling approach to get useful velocity commands
    if (! isInitialized())
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> local_plan;

    //compute what trajectory to drive along
    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = costmap_ros_->getBaseFrameID();

    drc_->computeVelocity(global_pose, robot_vel, drive_cmds);

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

    odom_helper_.setCmdVel(cmd_vel);


    ROS_DEBUG_NAMED("dead_reckoning_controller", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    return true; //should be a better check in the future
  }

  bool DeadReckoningControllerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {

    if (canceled_)
    {
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


    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getEstimatedRobotVel(robot_vel);

    if (drc_->isGoalReached(current_pose_)) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      odom_helper_.setCmdVel(cmd_vel);
      return true;
    } else {
      bool isOk = deadReckoningComputeVelocityCommands(current_pose_, robot_vel, cmd_vel);
      if (isOk) {
      } else {
        ROS_DEBUG_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
      }
      return isOk;
    }
  }

  bool DeadReckoningControllerROS::cancel()
  {
	  canceled_ = true;
	  return true;
  }

};
