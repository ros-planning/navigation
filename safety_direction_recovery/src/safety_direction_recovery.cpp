#include <safety_direction_recovery/safety_direction_recovery.h>
#include <nav_core/parameter_magic.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <angles/angles.h>

lexxauto_msgs::safety_status safety_status_;
static void safetyStatusCallback(const lexxauto_msgs::safety_status::ConstPtr& msg)
{
  safety_status_ = *msg;
}

PLUGINLIB_EXPORT_CLASS(safety_direction_recovery::SafetyDirectionRecovery, nav_core::RecoveryBehavior)

  namespace safety_direction_recovery{
    SafetyDirectionRecovery::SafetyDirectionRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
    {
    }

    void SafetyDirectionRecovery::initialize(std::string name, tf2_ros::Buffer*,
        costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
    {
      if(!initialized_)
      {
        local_costmap_ = local_costmap;

        // get some parameters from the parameter server
        ros::NodeHandle private_nh("~/" + name);
        ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

        private_nh.param("sim_granularity", sim_granularity_, 0.01);
        private_nh.param("frequency", frequency_, 20.0);

        blp_nh.param("max_vel_x", max_vel_x_, 2.0); 
        blp_nh.param("min_vel_x", min_vel_x_, 0.05);
        blp_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
        blp_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
        blp_nh.param("x_goal_tolerance", x_goal_tolerance_, 0.10);
        blp_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.10);

        ROS_INFO("x_goal_tolerance_: %f, y_goal_tolerance_: %f", x_goal_tolerance_, yaw_goal_tolerance_);
        ROS_INFO("sim_granularity__: %f, frequency_: %f", sim_granularity_, frequency_);

        acc_lim_th_ = nav_core::loadParameterWithDeprecation(blp_nh, "acc_lim_theta", "acc_lim_th", 3.2);
        max_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_theta", "max_rotational_vel", 1.0);
        min_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_in_place_vel_theta", "min_in_place_rotational_vel", 0.4);
        angle_to_rot_ = M_PI/2.0f;

        world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

        initialized_ = true;

        ros::NodeHandle nh;
        ros::Subscriber safety_status_sub =
          nh.subscribe<lexxauto_msgs::safety_status>("safety_status_", 1, safetyStatusCallback);
      }
      else
      {
        ROS_ERROR("You should not call initialize twice on this object, doing nothing");
      }
    }

    SafetyDirectionRecovery::~SafetyDirectionRecovery()
    {
      delete world_model_;
    }
 
    double SafetyDirectionRecovery::calculateDist(geometry_msgs::PoseStamped initial, geometry_msgs::PoseStamped current)
    {
      return sqrt(pow((initial.pose.position.x - current.pose.position.x), 2.0) +
                  pow((initial.pose.position.y - current.pose.position.y), 2.0));
    }

    void SafetyDirectionRecovery::runBehavior()
    {
      ROS_INFO("Safety direction recovery mode is called.");
      ros::NodeHandle n;
      ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

      if(!initialized_)
      {
        ROS_ERROR("This object must be initialized before runBehavior is called");
        return;
      }

      if(local_costmap_ == NULL)
      {
        ROS_ERROR("The costmap passed to the SafetyDirectionRecovery object cannnot be NULL. Doing nothing.");
        return;
      }

      double angle_rotated  = 0.0;
      double dist_travelled = 0.0;

      int recovery_cnt_ = rand();
/*
      ROS_INFO("safety_status_.back: %s", (char*)safety_status_.back.c_str());
      ROS_INFO("safety_status_.front: %s", (char*)safety_status_.front.c_str());
      ROS_INFO("safety_status_.side_left: %s", (char*)safety_status_.side_left.c_str());
      ROS_INFO("safety_status_.side_right: %s", (char*)safety_status_.side_right.c_str());
*/

      if (safety_status_.back != "stop" && recovery_cnt_ % 4 == 0)
      {
        ROS_INFO("Robot will start moving backward.");
        const double backward_direction = -1;
        dist_travelled = go_straight(backward_direction);
      }
      else if (safety_status_.front != "stop" && recovery_cnt_ % 4 == 1)
      {
        ROS_INFO("Robot will start moving forward.");
        const double forward_direction = 1;
        dist_travelled = go_straight(forward_direction);
      }
      else if (safety_status_.side_left != "stop" && recovery_cnt_ % 4 == 2)
      {
        ROS_INFO("Robot will start rotating to left.");
        const double counter_clockwise_direction = 1;
        angle_rotated = rotate(counter_clockwise_direction);

        ROS_INFO("Robot will start moving forward.");
        const double forward_direction = 1;
        dist_travelled = go_straight(forward_direction);
      }
      else if (safety_status_.side_right != "stop" && recovery_cnt_ % 4 == 3)
      {
        ROS_INFO("Robot will start rotating to right.");
        const double clockwise_direction = 1;
        angle_rotated = rotate(clockwise_direction);

        ROS_INFO("Robot will start moving forward.");
        const double forward_direction = 1;
        dist_travelled = go_straight(forward_direction);
      }
      else
      {
        ROS_INFO("Safety recovery is not conducted because the robot is surrounded by obstacles.\n");
        return;
      }

      ROS_INFO("Safety direction recovery ended because the robot rotated %f and travelled %f.\n", angle_rotated, dist_travelled);
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      vel_pub.publish(cmd_vel);

      return;
    }

    double SafetyDirectionRecovery::go_straight(const double direction)
    {
      ros::Rate r(frequency_);
      ros::NodeHandle n;
      ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

      geometry_msgs::PoseStamped global_pose;
      geometry_msgs::PoseStamped initial_pose;
      local_costmap_->getRobotPose(initial_pose);
      double current_angle = tf2::getYaw(initial_pose.pose.orientation);
      const double dist_to_move = 0.25;
      double dist_travelled = 0.0;
      double dist_left = dist_to_move - dist_travelled;

      while(n.ok() && dist_left > x_goal_tolerance_)
      {
        // update current position of the robot
        local_costmap_->getRobotPose(global_pose);
        double x = global_pose.pose.position.x, y = global_pose.pose.position.y;
        current_angle = tf2::getYaw(global_pose.pose.orientation);
        dist_travelled = SafetyDirectionRecovery::calculateDist(initial_pose, global_pose);

        // conduct simulation
        dist_left = dist_to_move - dist_travelled;
/*
        double sim_distance = 0.0;
        while(sim_distance < dist_left)
        {
          double sim_x = x + direction * sim_distance * cos(current_angle);
          double sim_y = y + direction * sim_distance * sin(current_angle);

          // make sure that the point is legal. Else, abort
          double footprint_cost = world_model_->footprintCost(sim_x, sim_y, current_angle, local_costmap_->getRobotFootprint(), inscribed_radius_, circumscribed_radius_);
          if(footprint_cost < 0.0)
          {
            ROS_ERROR("Safety direction recovery can't be conducted because there is a potential collision. Cost: %.2f", footprint_cost);
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;

            vel_pub.publish(cmd_vel);

            ROS_INFO("The robot travelled  %f.\n", dist_travelled);
            return dist_travelled;
          }

          sim_distance += sim_granularity_;
        }
*/

        double vel = direction * 0.1;
        vel = std::min(std::max(vel, min_vel_x_), max_vel_x_);

        cmd_vel.linear.x = vel;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;

        vel_pub.publish(cmd_vel);

        r.sleep();
      }

      ROS_INFO("The robot travelled  %f.\n", dist_travelled);
      return dist_travelled;
    }

    double SafetyDirectionRecovery::rotate(const double direction)
    {
      ros::Rate r(frequency_);
      ros::NodeHandle n;
      ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

      geometry_msgs::PoseStamped global_pose;
      local_costmap_->getRobotPose(global_pose);
      double current_angle = tf2::getYaw(global_pose.pose.orientation);
      double start_angle = current_angle;
      bool is_goal_reached = false;
      double angle_left = 0;

      while (n.ok() && !is_goal_reached)
      {
        // Update Current Angle
        local_costmap_->getRobotPose(global_pose);
        current_angle = tf2::getYaw(global_pose.pose.orientation);

        // TODO: using this function might induce a bug when you want the robot to rotate more than 180 deg
        angle_left = std::fabs(angles::shortest_angular_distance(current_angle, start_angle + angle_to_rot_ * direction));
        if (angle_left < yaw_goal_tolerance_) is_goal_reached = true;

        double x = global_pose.pose.position.x, y = global_pose.pose.position.y;

        // check if that velocity is legal by forward simulating
/*
        double sim_angle = 0.0;
        while (sim_angle < angle_left)
        {
          double theta = current_angle + sim_angle * direction;

          // make sure that the point is legal, if it isn't... we'll abort
          double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
          if (footprint_cost < 0.0)
          {
            ROS_ERROR("Rotate recovery can't rotate in place because there is a potential collision. Cost: %.2f",
                       footprint_cost);
            return M_PI/2.0f - angle_left;
          }

          sim_angle += sim_granularity_;
        }
*/

        // compute the velocity that will let us stop by the time we reach the goal
        double vel = direction * sqrt(2 * acc_lim_th_ * angle_left);

        // make sure that this velocity falls within the specified limits
        vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = vel;

        vel_pub.publish(cmd_vel);

        r.sleep();
      }

      return M_PI/2.0f - angle_left;
    }

  };  // namespace safety_direction_recovery
