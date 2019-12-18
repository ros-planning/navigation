
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

  double DeadReckoningController::distanceFromLine(tf::Stamped<tf::Pose> line_start, tf::Stamped<tf::Pose> line_end, tf::Stamped<tf::Pose> point)
  {
    //Calcluation to get distance from line. Will return postiive if left of line, and negative if right of line.
    double numerator = -1 * ((line_end.getOrigin().getY() - line_start.getOrigin().getY()) * point.getOrigin().getX() - 
                          (line_end.getOrigin().getX() - line_start.getOrigin().getX()) * point.getOrigin().getY() +
                          line_end.getOrigin().getX() * line_start.getOrigin().getY() - 
                          line_end.getOrigin().getY() * line_start.getOrigin().getX());

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
  }

  bool DeadReckoningController::setPlan(const tf::Stamped<tf::Pose> start_pose, const tf::Stamped<tf::Pose> end_pose, const tf::Stamped<tf::Pose> current_pose) {
    start_pose_ = start_pose;
    end_pose_ = end_pose;
    path_ = start_pose_.inverseTimes(end_pose_);
    ROS_INFO_STREAM(distanceFromLine(start_pose, end_pose, current_pose));
    //check if the start distance in the y direction is too far from the path vector.
    if (std::fabs(distanceFromLine(start_pose, end_pose, current_pose)) > config_.max_path_correction) {
      ROS_ERROR("Robot too far from path, can't proceed");
      return false;
    }
    return true;
  }

  double DeadReckoningController::calculateLinearVelocity(tf::Stamped<tf::Pose> current_pose, tf::Stamped<tf::Pose> current_velocity){
    //current distance from the goal position.
    double currentDistanceAway = sqrt(pow(current_pose.getOrigin().getX() - end_pose_.getOrigin().getX(),2) + pow(current_pose.getOrigin().getY() - end_pose_.getOrigin().getY(), 2));
    //base velocity on the acceleration curve
    double p_error = sqrt(2*std::fabs(currentDistanceAway) * config_.acc_lim_x);
    
    return std::min(config_.max_vel_x, p_error * config_.p_weight_linear); //reconfigurable and better variable names

  }

  double DeadReckoningController::calculateAngularVelocity(tf::Stamped<tf::Pose> current_pose,tf::Stamped<tf::Pose> current_velocity){

    //I dont like these lines, but I can't figure a non buggy way tf to get the angle diff with correct signs
    double current_yaw;
    double path_yaw;

    //yaw of current pose orientation
    tf::Matrix3x3 current_pose_matrix(current_pose.getRotation());
    double roll, pitch, yaw;
    current_pose_matrix.getRPY(roll, pitch, yaw);
    current_yaw = yaw;
    

    //yaw of path orientation
    tf::Matrix3x3 path_matrix(path_.getRotation());
    path_matrix.getRPY(roll, pitch, yaw);
    path_yaw = yaw;
    
    //just get it in {-pi to pi} format
    double angleDiff;
    angleDiff = current_yaw - path_yaw;
    if (angleDiff > M_PI){
        angleDiff = angleDiff - 2*M_PI;
    } else if (angleDiff < -1*M_PI){
        angleDiff = angleDiff + 2*M_PI;
    }

    double Perror = sin(angleDiff)*current_velocity.getOrigin().getX(); //essentially the y velocity of the robot away from the path vector
    double Ierror = distanceFromLine(start_pose_, end_pose_, current_pose); //essentially the y distance from the path vector
    //ROS_INFO_STREAM("Angle diff" << angleDiff << " Ierror: " << Ierror << " " << "Perror: " << Perror);

    double angular_velocity = -1 * config_.p_weight_angular * Perror + -1 * config_.i_weight_angular * Ierror;

    //make sure it's not above the max ang z limits
    if (std::fabs(angular_velocity) < config_.max_ang_z){
      return angular_velocity;
    }
    else if (angular_velocity < 0) {
      return -1 * config_.max_ang_z;
    }
    else{
      return config_.max_ang_z;
    }
  }

  bool DeadReckoningController::isGoalReached(const tf::Stamped<tf::Pose> current_pose){
    //need this to get a proper vector form current pose to the start pose.
    tf::Pose current_pose_transform = start_pose_.inverseTimes(current_pose);

    //this does a check to see if the robot is past the goal vector's y axis.
    //this is comparing start_pose -> current_pose and start_pose -> end_pose
    if((current_pose_transform.getOrigin().getX() * path_.getOrigin().getX() + current_pose_transform.getOrigin().getY() * path_.getOrigin().getY()) > 
      (path_.getOrigin().getX() * path_.getOrigin().getX() + path_.getOrigin().getY() * path_.getOrigin().getY())){
        return true;

    } else{
      return false;
    }
    
  };

  void DeadReckoningController::computeVelocity(
          tf::Stamped<tf::Pose> global_pose,
          tf::Stamped<tf::Pose> global_vel,
          tf::Stamped<tf::Pose>& drive_velocities) {

    double angular_velocity = calculateAngularVelocity(global_pose, global_vel);
    double lienar_velocity = calculateLinearVelocity(global_pose, global_vel);
    tf::Vector3 origin(lienar_velocity, 0, 0);
    tf::Quaternion rotation;
    rotation.setRPY(0,0,angular_velocity);
    drive_velocities.setOrigin(origin);
    drive_velocities.setRotation(rotation);
  }


};
