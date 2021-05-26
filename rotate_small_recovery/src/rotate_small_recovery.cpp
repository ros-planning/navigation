#include <rotate_small_recovery/rotate_small_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <angles/angles.h>
#include <nav_core/parameter_magic.h>

PLUGINLIB_EXPORT_CLASS(rotate_small_recovery::RotateSmallRecovery, nav_core::RecoveryBehavior)

namespace rotate_small_recovery
{
RotateSmallRecovery::RotateSmallRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void RotateSmallRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/RotateSmallRecovery");
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    // we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);

    acc_lim_th_ = nav_core::loadParameterWithDeprecation(blp_nh, "acc_lim_theta", "acc_lim_th", 3.2);
    max_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_theta", "max_rotational_vel", 1.0);
    min_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_in_place_vel_theta", "min_in_place_rotational_vel", 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);
    blp_nh.param("rotate_small_angle", angle_to_rot_, 30.0);
    angle_to_rot_ *= M_PI/180.0;

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

RotateSmallRecovery::~RotateSmallRecovery()
{
  delete world_model_;
}

void RotateSmallRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if (local_costmap_ == NULL)
  {
    ROS_ERROR("The costmap passed to the RotateRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Rotate recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  geometry_msgs::PoseStamped global_pose;
  local_costmap_->getRobotPose(global_pose);

  double current_angle = tf2::getYaw(global_pose.pose.orientation);
  double start_angle = current_angle;

  bool is_goal_reached = false;

  while (n.ok() && !is_goal_reached)
  {
    // Update Current Angle
    local_costmap_->getRobotPose(global_pose);
    current_angle = tf2::getYaw(global_pose.pose.orientation);

    // compute the distance left to rotate
    // TODO: using this function might induce a bug when you want the robot to rotate more than 180 deg
    double angle_to_goal = std::fabs(angles::shortest_angular_distance(current_angle, start_angle + angle_to_rot_));
    double angle_left = angle_to_goal;

    if (angle_to_goal < tolerance_)
    {
    is_goal_reached = true;
    }

    double x = global_pose.pose.position.x, y = global_pose.pose.position.y;

    // check if that velocity is legal by forward simulating
    double sim_angle = 0.0;
    while (sim_angle < angle_left)
    {
      double theta = current_angle + sim_angle;

      // make sure that the point is legal, if it isn't... we'll abort
      double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
      if (footprint_cost < 0.0)
      {
        ROS_ERROR("Rotate recovery can't rotate in place because there is a potential collision. Cost: %.2f",
                  footprint_cost);
        return;
      }

      sim_angle += sim_granularity_;
    }

    // compute the velocity that will let us stop by the time we reach the goal
    double vel = sqrt(2 * acc_lim_th_ * angle_left);

    // make sure that this velocity falls within the specified limits
    vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = vel;

    vel_pub.publish(cmd_vel);

    r.sleep();
  }
}
};  // namespace rotate_small_recovery
