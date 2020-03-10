
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

  //             ^line_end
  //             |
  //     *_______|
  //         d   |
  //             |
  //             |
  //             |
  //             |line_start
  //    
  // Calculate minimum distance from a line. If the point is left to the line, d is postivie, if to the right d is negative

  double DeadReckoningController::distanceFromLine(tf::Stamped<tf::Pose> line_start, tf::Stamped<tf::Pose> line_end, tf::Stamped<tf::Pose> point)
  {
    //calculation is essentially taking the hypotnuse(line start -> point) and subtracting one of the sides (dot(line, point)) to get the resulting vector from line to point.
    double numerator = -1 * ((line_end.getOrigin().getY() - line_start.getOrigin().getY()) * point.getOrigin().getX() - 
                          (line_end.getOrigin().getX() - line_start.getOrigin().getX()) * point.getOrigin().getY() +
                          line_end.getOrigin().getX() * line_start.getOrigin().getY() - 
                          line_end.getOrigin().getY() * line_start.getOrigin().getX());
    //denominator is making sure that line_start to line_end is normalized.
    double denominator = sqrt(pow(line_end.getOrigin().getY() - line_start.getOrigin().getY(), 2) + pow(line_end.getOrigin().getX() - line_start.getOrigin().getX(), 2));
    if( denominator != 0) {
      return numerator/denominator;
    } else{
      return 0;
    }
  }
  void DeadReckoningController::reconfigure(DeadReckoningControllerConfig &config)
  {
    config_ = config;
  }

  DeadReckoningController::DeadReckoningController(std::string name)
  {
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle global_nh;

    std::string controller_frequency_param_name;
    double default_frequency = 0.05;
    if(!global_nh.searchParam("controller_frequency", controller_frequency_param_name)) {
      sim_period_ = default_frequency;
    } else {
      double controller_frequency = 0;
      global_nh.param(controller_frequency_param_name, controller_frequency, 1.0 / default_frequency);
      if(controller_frequency > 0) {
        sim_period_ = 1.0 / controller_frequency;
      } else {
        ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
        sim_period_ = default_frequency;
      }
    }
  }

  bool DeadReckoningController::setPlan(const tf::Stamped<tf::Pose> start_pose, const tf::Stamped<tf::Pose> end_pose, const tf::Stamped<tf::Pose> current_pose) {
    start_pose_ = start_pose;
    end_pose_ = end_pose;
    path_ = start_pose_.inverseTimes(end_pose_);
    //ROS_INFO_STREAM(distanceFromLine(start_pose, end_pose, current_pose));
    //check if the start distance in the y direction is too far from the path vector.
    if (std::fabs(distanceFromLine(start_pose, end_pose, current_pose)) > config_.max_path_correction) {
      ROS_ERROR("Robot too far from path, can't proceed");
      return false;
    }
    do_turn_in_place_ = config_.tip_first;
    return true;
  }

  double DeadReckoningController::calculateLinearVelocity(tf::Stamped<tf::Pose> current_pose, tf::Stamped<tf::Pose> current_velocity){
    //current distance from the goal position.
    double currentDistanceAway = sqrt(pow(current_pose.getOrigin().getX() - end_pose_.getOrigin().getX(),2) + pow(current_pose.getOrigin().getY() - end_pose_.getOrigin().getY(), 2));
    //base velocity on the acceleration curve
    double p_error = sqrt(2*std::fabs(currentDistanceAway) * config_.acc_lim_x);
    
    if(do_turn_in_place_){
      return 0;
    }
    return std::min(config_.max_vel_x, p_error * config_.p_weight_linear); //reconfigurable and better variable names

  }

  double DeadReckoningController::calculateAngularVelocity(tf::Stamped<tf::Pose> current_pose,tf::Stamped<tf::Pose> current_velocity){


    double ang_vel_out;
    

    double current_yaw;
    double end_pose_yaw;
    double current_vel_ang;

    current_yaw = tf::getYaw(current_pose.getRotation());
    end_pose_yaw = tf::getYaw(end_pose_.getRotation());
    current_vel_ang = tf::getYaw(current_velocity.getRotation());
    
    //just get it in {-pi to pi} format
    double angleDiff;
    angleDiff = current_yaw - end_pose_yaw;
    if (angleDiff > M_PI){
        angleDiff = angleDiff - 2*M_PI;
    } else if (angleDiff < -1*M_PI){
        angleDiff = angleDiff + 2*M_PI;
    }

    if(do_turn_in_place_){
      if(std::fabs(angleDiff) < config_.tip_difference_threshold) {
        do_turn_in_place_ = false;
      }
      else{
        //coming from the latched stop rotate code.
        //this is actually the gain conversion from distance to velocity
        double v_theta_samp = std::min(config_.max_tip_vel, std::max(config_.max_tip_vel, fabs(angleDiff)));

        double max_acc_vel = fabs(current_vel_ang) + config_.acc_lim_tip * sim_period_;
        double min_acc_vel = fabs(current_vel_ang) - config_.acc_lim_tip * sim_period_;

        v_theta_samp = std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

        double max_speed_to_stop = sqrt(2 * config_.acc_lim_tip * fabs(angleDiff));
        v_theta_samp = std::min(max_speed_to_stop, fabs(v_theta_samp));

        v_theta_samp = std::min(config_.max_tip_vel, std::max(config_.max_tip_vel, v_theta_samp));

        if (angleDiff < 0) {
          v_theta_samp = - v_theta_samp;
        }
        ang_vel_out = - v_theta_samp;
      }
    }
    else{

      double Perror = sin(angleDiff)*current_velocity.getOrigin().getX(); //essentially the y velocity of the robot away from the path vector
      double Ierror = distanceFromLine(start_pose_, end_pose_, current_pose); //essentially the y distance from the path vector
      //ROS_INFO_STREAM("Angle diff" << angleDiff << " Ierror: " << Ierror << " " << "Perror: " << Perror);

      double angular_velocity = -1 * config_.p_weight_angular * Perror + -1 * config_.i_weight_angular * Ierror;

      ang_vel_out = angular_velocity;

      if(angleDiff > config_.max_sweep_angle && angular_velocity > 0)
      {
        ang_vel_out =  0;
      } else if (angleDiff < -1 * config_.max_sweep_angle && angular_velocity < 0)
      {
        ang_vel_out =  0;
      }
    }
    
    //make sure it's not above the max ang z limits
    if (std::fabs(ang_vel_out) < config_.max_rot_vel){
      return ang_vel_out;
    }
    else if (ang_vel_out < 0) {
      return -1 * config_.max_rot_vel;
    }
    else{
      return config_.max_rot_vel;
    }
  }

  bool DeadReckoningController::isGoalReached(const tf::Stamped<tf::Pose> current_pose){

    //this does a check to see if the robot is past the goal vector's y axis.
    tf::StampedTransform transform;
    tf::Pose current_pose_transform = end_pose_.inverseTimes(current_pose);
    if(current_pose_transform.getOrigin().getX() > 0){
      return true;
    }
    else{
      return false;
    }
  };

  bool DeadReckoningController::computeVelocity(
          tf::Stamped<tf::Pose> global_pose,
          tf::Stamped<tf::Pose> global_vel,
          tf::Stamped<tf::Pose>& drive_velocities) {

    double angular_velocity = calculateAngularVelocity(global_pose, global_vel);
    double linear_velocity = calculateLinearVelocity(global_pose, global_vel);
    tf::Vector3 origin(linear_velocity, 0, 0);
    tf::Quaternion rotation;
    rotation.setRPY(0 ,0, angular_velocity);
    drive_velocities.setOrigin(origin);
    drive_velocities.setRotation(rotation);

    //TO-DO do a check if we are in a good position and return false
    return true;
  }


};
