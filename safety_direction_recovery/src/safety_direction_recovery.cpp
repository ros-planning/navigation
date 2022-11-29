#include <safety_direction_recovery/safety_direction_recovery.h>
#include <nav_core/parameter_magic.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <angles/angles.h>
#include <vector>

lexxauto_msgs::safety_status safety_status_;
float front_lidar_distance_;
float front_left_lidar_distance_;
float left_lidar_distance_;
float rear_left_lidar_distance_;
float rear_lidar_distance_;
float rear_right_lidar_distance_;
float right_lidar_distance_;
float front_right_lidar_distance_;

void safetyStatusCallback(const lexxauto_msgs::safety_status::ConstPtr& msg)
{
  safety_status_ = *msg;
}
void frontLidarDistanceCallback(const std_msgs::Float32::ConstPtr& msg)
{
  front_lidar_distance_ = msg->data;
//  ROS_INFO("test front_lidar_distance: %f", front_lidar_distance_);
}
void frontLeftLidarDistanceCallback(const std_msgs::Float32::ConstPtr& msg)
{
  front_left_lidar_distance_ = msg->data;
//  ROS_INFO("test front_left_lidar_distance: %f", front_left_lidar_distance_);
}
void leftLidarDistanceCallback(const std_msgs::Float32::ConstPtr& msg)
{
  left_lidar_distance_ = msg->data;
//  ROS_INFO("test left_lidar_distance: %f", left_lidar_distance_);
}
void rearLeftLidarDistanceCallback(const std_msgs::Float32::ConstPtr& msg)
{
  rear_left_lidar_distance_ = msg->data;
//  ROS_INFO("test rear_left_lidar_distance: %f", rear_left_lidar_distance_);
}
void rearLidarDistanceCallback(const std_msgs::Float32::ConstPtr& msg)
{
  rear_lidar_distance_ = msg->data;
//  ROS_INFO("test rear_lidar_distance: %f", rear_lidar_distance_);
}
void rearRightLidarDistanceCallback(const std_msgs::Float32::ConstPtr& msg)
{
  rear_right_lidar_distance_ = msg->data;
//  ROS_INFO("test rear_right_lidar_distance: %f", rear_right_lidar_distance_);
}
void rightLidarDistanceCallback(const std_msgs::Float32::ConstPtr& msg)
{
  right_lidar_distance_ = msg->data;
//  ROS_INFO("test right_lidar_distance: %f", right_lidar_distance_);
}
void frontRightLidarDistanceCallback(const std_msgs::Float32::ConstPtr& msg)
{
  front_right_lidar_distance_ = msg->data;
//  ROS_INFO("test front_right_lidar_distance: %f", front_right_lidar_distance_);
}

PLUGINLIB_EXPORT_CLASS(safety_direction_recovery::SafetyDirectionRecovery, nav_core::RecoveryBehavior)
  namespace safety_direction_recovery{
    SafetyDirectionRecovery::SafetyDirectionRecovery(): local_costmap_(NULL), global_costmap_(NULL), initialized_(false),
                                                        local_world_model_(NULL), global_world_model_(NULL)
    {
    }

    void SafetyDirectionRecovery::initialize(std::string name, tf2_ros::Buffer*,
        costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
    {
      if(!initialized_)
      {
        global_costmap_ = global_costmap;
        local_costmap_ = local_costmap;

        // get some parameters from the parameter server
        ros::NodeHandle private_nh("~/" + name);
        ros::NodeHandle blp_nh("~/DWAPlannerROS");

        private_nh.param("sim_granularity", sim_granularity_, 0.01);
        private_nh.param("frequency", frequency_, 10.0);
        ROS_INFO("sim_granularity_: %f, frequency_: %f", sim_granularity_, frequency_);

        blp_nh.param("max_vel_x", max_vel_x_, 1.0);
        blp_nh.param("min_vel_x", min_vel_x_, -1.0);
        ROS_INFO("max_vel_x_: %f, min_vel_x__: %f", max_vel_x_, min_vel_x_);

        blp_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
        blp_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
        ROS_INFO("inscribed_radius_: %f, circumscribed_radius_: %f", inscribed_radius_, circumscribed_radius_);

        blp_nh.param("x_goal_tolerance", x_goal_tolerance_, 0.10);
        blp_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.10);
        ROS_INFO("x_goal_tolerance_: %f, yaw_goal_tolerance_: %f", x_goal_tolerance_, yaw_goal_tolerance_);

        acc_lim_th_ = nav_core::loadParameterWithDeprecation(blp_nh, "acc_lim_theta", "acc_lim_th", 3.2);
        blp_nh.param("max_rotational_vel_", max_rotational_vel_, 1.0);
        blp_nh.param("min_rotational_vel_", min_rotational_vel_, -1.0);
        ROS_INFO("acc_lim_th_: %f, max_rotational_vel__: %f, min_rotational_vel__: %f", acc_lim_th_, max_rotational_vel_, min_rotational_vel_);
//        max_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_theta", "max_rotational_vel", 1.0);
//        min_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_in_place_vel_theta", "min_in_place_rotational_vel", 0.4);
//        max_rotational_vel_ = 1.0f;
//        min_rotational_vel_ = -1.0f;
//        max_vel_x_ = 1.0f;
//        min_vel_x_ = -1.0f;

        local_world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());
        global_world_model_ = new base_local_planner::CostmapModel(*global_costmap_->getCostmap());

        initialized_ = true;

        ros::NodeHandle nh;
        //ros::Subscriber safety_status_sub =
        //  nh.subscribe<lexxauto_msgs::safety_status>("safety_status", 1, safetyStatusCallback);
        front_lidar_distance_sub_ =
          nh.subscribe<std_msgs::Float32>("front_lidar_distance", 1, frontLidarDistanceCallback);
        front_left_lidar_distance_sub_ =
          nh.subscribe<std_msgs::Float32>("front_left_lidar_distance", 1, frontLeftLidarDistanceCallback);
        left_lidar_distance_sub_ =
          nh.subscribe<std_msgs::Float32>("left_lidar_distance", 1, leftLidarDistanceCallback);
        rear_left_lidar_distance_sub_ =
          nh.subscribe<std_msgs::Float32>("rear_left_lidar_distance", 1, rearLeftLidarDistanceCallback);
        rear_lidar_distance_sub_ =
          nh.subscribe<std_msgs::Float32>("rear_lidar_distance", 1, rearLidarDistanceCallback);
        rear_right_lidar_distance_sub_ =
          nh.subscribe<std_msgs::Float32>("rear_right_lidar_distance", 1, rearRightLidarDistanceCallback);
        right_lidar_distance_sub_ =
          nh.subscribe<std_msgs::Float32>("right_lidar_distance", 1, rightLidarDistanceCallback);
        front_right_lidar_distance_sub_ =
          nh.subscribe<std_msgs::Float32>("front_right_lidar_distance", 1, frontRightLidarDistanceCallback);
      }
      else
      {
        ROS_ERROR("You should not call initialize twice on this object, doing nothing");
      }
    }

    SafetyDirectionRecovery::~SafetyDirectionRecovery()
    {
      delete local_world_model_;
      delete global_world_model_;
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

      double best_attitude = 0;
      double best_dist_to_move = 0.75;
      calc_angle_distance_to_safest_place_via_simulation(best_attitude, best_dist_to_move);
      //calc_angle_distance_to_safest_place_via_lidar(best_attitude, best_dist_to_move);

      double rotate_direction = 1;
      double recovery_rotate_angle = 0;
      double straight_direction = 1;
      calc_recovery_move(best_attitude, rotate_direction, recovery_rotate_angle, straight_direction);

      double angle_rotated = rotate((double)rotate_direction, recovery_rotate_angle);
      double dist_travelled = go_straight(straight_direction, best_dist_to_move);

      ROS_INFO("Safety direction recovery ended because the robot rotated %f and travelled %f.\n", angle_rotated, dist_travelled);
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      vel_pub.publish(cmd_vel);

      return;
    }

    double limit_distance (double lidar_distance)
    {
      double measurable_distance = 40;
      return (lidar_distance > measurable_distance) ? 0 : lidar_distance;
    }

    void SafetyDirectionRecovery::calc_angle_distance_to_safest_place_via_lidar(double& best_attitude, double& best_dist_to_move)
    {
      int lidar_direction_num = 8;
      std::vector<double> distance_array(lidar_direction_num);
      distance_array[0] = limit_distance(front_lidar_distance_);
      distance_array[1] = limit_distance(front_left_lidar_distance_);
      distance_array[2] = limit_distance(left_lidar_distance_);
      distance_array[3] = limit_distance(rear_left_lidar_distance_);
      distance_array[4] = limit_distance(rear_lidar_distance_);
      distance_array[5] = limit_distance(rear_right_lidar_distance_);
      distance_array[6] = limit_distance(right_lidar_distance_);
      distance_array[7] = limit_distance(front_right_lidar_distance_);

      double max_distance = 0;
      double max_dir_num  = 0;
      for (int i=0; i<lidar_direction_num; i++)
      {
        if (max_distance < distance_array[i])
        {
          max_distance = distance_array[i];
          max_dir_num  = i;
        }
        ROS_INFO("distance measured by lidar [i=%d]: %f", i, distance_array[i]);
      }
      //std::vector<double>::iterator max_it = max_element(distance_array.begin(), distance_array.end());
      //size_t max_index = distance(distance_array.begin(), max_it);
 
      best_attitude = 2 * M_PI * (double)max_dir_num/(double)lidar_direction_num;
      best_dist_to_move = std::min(max_distance, 0.25);

      ROS_INFO("Robot will rotate 2pi * %d/%d [rad].", (int)max_dir_num, lidar_direction_num);
      ROS_INFO("Robot will move %f [m].", best_dist_to_move);
      return;
    }

    void SafetyDirectionRecovery::calc_angle_distance_to_safest_place_via_simulation(double& best_attitude, double& best_dist_to_move)
    {
//      sleep(1); // wait for costmap update

      int simulate_direction_num = 16;
      std::vector<double> total_cost_array(simulate_direction_num);
      for (int i=0; i<simulate_direction_num; i++)
      {
        double sim_angle = 2 * M_PI * (double)i/(double)simulate_direction_num;
        double sim_distance = best_dist_to_move * 1.2;
        total_cost_array[i] = simulate_cost_around_robot(sim_angle, sim_distance);
        ROS_INFO("total_cost_array[i=%d]: %f", i, total_cost_array[i]);
      }
/*
      std::vector<double>::iterator max_it = max_element(total_cost_array.begin(), total_cost_array.end());
      size_t max_index = distance(total_cost_array.begin(), max_it);
*/

      std::vector<double> total_cost_sum_array(simulate_direction_num);
      for (int i=0; i<simulate_direction_num; i++)
      {
        int ip = (i==0) ? simulate_direction_num-1 : i-1;
        int in = (i==simulate_direction_num-1) ? 0 : i+1;
        total_cost_sum_array[i] = total_cost_array[ip] + total_cost_array[i] + total_cost_array[in];
        /*
        if (i < simulate_direction_num/4 || (simulate_direction_num - i) < simulate_direction_num/4)
        {
          total_cost_sum_array[i] /= 4.0;
        }
        */
        ROS_INFO("total_cost_sum_array[i=%d]: %f", i, total_cost_sum_array[i]);
      }
      std::vector<double>::iterator max_it = max_element(total_cost_sum_array.begin(), total_cost_sum_array.end());
      size_t max_index = distance(total_cost_sum_array.begin(), max_it);

      best_attitude = 2 * M_PI * (double)max_index/(double)simulate_direction_num;
      ROS_INFO("Robot will rotate 2pi * %d/%d [rad].", (int)max_index, simulate_direction_num);
      ROS_INFO("Robot will move 0.25 [m].");

      return;
    }

    void SafetyDirectionRecovery::calc_recovery_move(const double best_attitude, double& rotate_direction, double& recovery_rotate_angle, double& straight_direction)
    {
      if (0 <= best_attitude && best_attitude <= M_PI/2.0f)
      { // counter-clockwise, forward
        recovery_rotate_angle = best_attitude;
        rotate_direction = 1;
        straight_direction = 1;
        ROS_INFO("Range in [0, pi/2]: best_attitude: %f", best_attitude);
      }
      else if (M_PI/2.0f < best_attitude && best_attitude <= M_PI)
      { // clockwise, backward
        recovery_rotate_angle = M_PI - best_attitude;
        rotate_direction = -1;
        straight_direction = -1;
        ROS_INFO("Range in [pi/2, pi]: best_attitude: %f", best_attitude);
      }
      else if (M_PI < best_attitude && best_attitude <= 3.0f*M_PI/2.0f)
      { // counter-clockwise, backward
        recovery_rotate_angle = best_attitude - M_PI;
        rotate_direction = 1;
        straight_direction = -1;
        ROS_INFO("Range in [pi, 3*pi/2]: best_attitude: %f", best_attitude);
      }
      else if (3.0f*M_PI/2.0f < best_attitude && best_attitude <= 2.0f*M_PI)
      { // clockwise, forward
        recovery_rotate_angle = 2*M_PI - best_attitude;
        rotate_direction = -1;
        straight_direction = 1;
        ROS_INFO("Range in [3*pi/2, 2*pi]: best_attitude: %f", best_attitude);
      }
      else
      {
        ROS_ERROR("Input angle error: best_attitude: %f [rad].", best_attitude);
        return;
      }

      ROS_INFO("Robot will move %s.", (straight_direction==1)?"forward":"backward");
      return;
    }

    double SafetyDirectionRecovery::simulate_cost_around_robot(const double sim_angle, const double sim_distance)
    {
      geometry_msgs::PoseStamped robot_pose;
      local_costmap_->getRobotPose(robot_pose);
      double robot_angle = tf2::getYaw(robot_pose.pose.orientation);
      double robot_x = robot_pose.pose.position.x;
      double robot_y = robot_pose.pose.position.y;

      double total_cost = 0;
      double tmp_distance = 0;

      while(tmp_distance < sim_distance)
      {
        double sim_x = robot_x + tmp_distance * cos(robot_angle + sim_angle);
        double sim_y = robot_y + tmp_distance * sin(robot_angle + sim_angle);
        double local_tmp_cost = local_world_model_->footprintCost(sim_x, sim_y, robot_angle + sim_angle, local_costmap_->getRobotFootprint(), inscribed_radius_, circumscribed_radius_);
        double global_tmp_cost = global_world_model_->footprintCost(sim_x, sim_y, robot_angle + sim_angle, global_costmap_->getRobotFootprint(), inscribed_radius_, circumscribed_radius_);

        tmp_distance += sim_granularity_;

        if (local_tmp_cost == -1) total_cost -= 1000; // LETHAL_OBSTACLE case
        else if (local_tmp_cost < 0) total_cost += local_tmp_cost;
        else total_cost -= local_tmp_cost;

        if (global_tmp_cost == -1) total_cost -= 1000; // LETHAL_OBSTACLE case
        else if (global_tmp_cost < 0) total_cost += global_tmp_cost;
        else total_cost -= global_tmp_cost;
      }

      return total_cost;
    }

    double SafetyDirectionRecovery::go_straight(const double direction, const double dist_to_move)
    {
      ros::Rate r(frequency_);
      ros::NodeHandle n;
      ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

      geometry_msgs::PoseStamped global_pose;
      geometry_msgs::PoseStamped initial_pose;
      local_costmap_->getRobotPose(initial_pose);
      double dist_travelled = 0.0;
      double dist_left = dist_to_move - dist_travelled;

      double time_passed = 0.0;
      double vel = direction * 0.25;
      double time_timeout = 3.0 * dist_to_move / abs(vel);

      //while(n.ok() && dist_left > x_goal_tolerance_)
      while(n.ok() && dist_left > 0)
      {
        // the robot is inside unmovable area
        double lcurrent_angle = tf2::getYaw(local_pose_.pose.orientation);
        double lx = local_pose_.pose.position.x;
        double ly = local_pose_.pose.position.y;
        double gcurrent_angle = tf2::getYaw(global_pose_.pose.orientation);
        double gx = global_pose_.pose.position.x;
        double gy = global_pose_.pose.position.y;
        int local_footprint_cost = local_world_model_->footprintCost(lx, ly, lcurrent_angle, local_costmap_->getRobotFootprint(),
                                                                        inscribed_radius_, circumscribed_radius_);
        int global_footprint_cost = global_world_model_->footprintCost(gx, gy, gcurrent_angle, global_costmap_->getRobotFootprint(),
                                                                          inscribed_radius_, circumscribed_radius_);
        std::cerr << "local_footprint_cost value is " << local_footprint_cost << std::endl;
        std::cerr << "global_footprint_cost value is " << global_footprint_cost << std::endl;
        if (is_near_unmovable_area_or_obstacle(local_footprint_cost, global_footprint_cost))
        {
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;
          vel_pub.publish(cmd_vel);

          ROS_WARN("robot is inside unmovable area or near obstacles.");
          break;
        }

        // break the loop if time is sufficienty passed. ex): blocked by obstacle
        time_passed += 1.0 / frequency_;
        if (time_passed > time_timeout)
        {
          ROS_WARN("safety_direction_recovery: timeout");
          break;
        }

        // update current position of the robot
        local_costmap_->getRobotPose(global_pose);
        dist_travelled = SafetyDirectionRecovery::calculateDist(initial_pose, global_pose);
        dist_left = dist_to_move - dist_travelled;

        // conduct simulation
        /*
        double sim_distance = 0.0;
        while(sim_distance < dist_left)
        {
          double sim_x = x + direction * sim_distance * cos(current_angle);
          double sim_y = y + direction * sim_distance * sin(current_angle);

          // make sure that the point is legal. Else, abort
          int local_footprint_cost = local_world_model_->footprintCost(sim_x, sim_y, current_angle, local_costmap_->getRobotFootprint(), inscribed_radius_, circumscribed_radius_);
          int global_footprint_cost = global_world_model_->footprintCost(sim_x, sim_y, current_angle, global_costmap_->getRobotFootprint(), inscribed_radius_, circumscribed_radius_);
          if (is_near_unmovable_area_or_obstacle(local_footprint_cost, global_footprint_cost))
          {
            ROS_ERROR("[safety_direction_recovery] the robot is going to unmovable area or obstacles.");
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

    bool SafetyDirectionRecovery::is_near_unmovable_area_or_obstacle(int local_footprint_cost, int global_footprint_cost)
    {
      if(local_footprint_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || local_footprint_cost == -1 ||
         global_footprint_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || global_footprint_cost == -1)
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    double SafetyDirectionRecovery::rotate(const double direction, const double rotate_angle)
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
        // the robot is inside unmovable area
        double current_angle = tf2::getYaw(global_pose.pose.orientation);
        double x = global_pose.pose.position.x;
        double y = global_pose.pose.position.y;
        int local_footprint_cost = local_world_model_->footprintCost(x, y, current_angle, local_costmap_->getRobotFootprint(),
                                                                        inscribed_radius_, circumscribed_radius_);
        int global_footprint_cost = global_world_model_->footprintCost(x, y, current_angle, global_costmap_->getRobotFootprint(),
                                                                          inscribed_radius_, circumscribed_radius_);
        if (is_near_unmovable_area_or_obstacle(local_footprint_cost, global_footprint_cost))
        {
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;
          vel_pub.publish(cmd_vel);

          ROS_WARN("robot is inside unmovable area or near obstacles.");
          break;
        }

        // Update Current Angle
        local_costmap_->getRobotPose(global_pose);
        current_angle = tf2::getYaw(global_pose.pose.orientation);

        // TODO: using this function might induce a bug when you want the robot to rotate more than 180 deg
        angle_left = std::fabs(angles::shortest_angular_distance(current_angle, start_angle + rotate_angle * direction));
        if (angle_left < yaw_goal_tolerance_) is_goal_reached = true;

        // check if that velocity is legal by forward simulating
/*
        double x = global_pose.pose.position.x, y = global_pose.pose.position.y;
        double sim_angle = 0.0;
        while (sim_angle < angle_left)
        {
          double theta = current_angle + sim_angle * direction;

          // make sure that the point is legal, if it isn't... we'll abort
          double footprint_cost = local_world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
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

      return current_angle - start_angle;
    }

  };  // namespace safety_direction_recovery
