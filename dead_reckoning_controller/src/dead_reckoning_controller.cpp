
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
    config_ = config;
  }

  DeadReckoningController::DeadReckoningController(std::string name)
  {
    ros::NodeHandle private_nh("~/" + name);
  }

  bool DeadReckoningController::setPlan(const tf::Stamped<tf::Pose> start_pose, const tf::Stamped<tf::Pose> end_pose) {

    start_pose_ = start_pose;
    end_pose_ = end_pose;
    path_ = start_pose_.inverseTimes(end_pose_);
    return true;
  }

  double DeadReckoningController::calculateLinearVelocity(tf::Stamped<tf::Pose> current_pose, tf::Stamped<tf::Pose> current_velocity){
    double currentDistanceAway = sqrt(pow(current_pose.getOrigin().getX() - end_pose_.getOrigin().getX(),2) + pow(current_pose.getOrigin().getY() - end_pose_.getOrigin().getY(), 2));
    double constantAccelerationValue = config_.acc_lim_x; //Dyanmic Reconfigurable???
    double p_error = sqrt(2*currentDistanceAway);
    return std::min(config_.max_vel_x, p_error * config_.p_weight_linear); //reconfigurable and better variable names

  }
  double DeadReckoningController::calculateAngularVelocity(tf::Stamped<tf::Pose> current_pose,tf::Stamped<tf::Pose> current_velocity){
    tf::Quaternion diff = current_pose.getRotation() * path_.getRotation().inverse();
    
    double angleDiff = diff.getAngle();//find angle difference between current heading and heading of path vector
    double Perror = sin(angleDiff)*current_velocity.getOrigin().getX(); //essentially the y velocity of the robot away from the path vector
    double numerator = -1 * ((end_pose_.getOrigin().getY() - start_pose_.getOrigin().getY()) * current_pose.getOrigin().getX() - 
                          (end_pose_.getOrigin().getX() - start_pose_.getOrigin().getX()) * current_pose.getOrigin().getY() +
                          end_pose_.getOrigin().getX() * start_pose_.getOrigin().getY() - 
                          end_pose_.getOrigin().getY() * start_pose_.getOrigin().getX());
                      
                      
                      //distance of robot away from closest point on path vector

    double denominator = sqrt(pow(end_pose_.getOrigin().getY() - start_pose_.getOrigin().getY(), 2) + pow(end_pose_.getOrigin().getX() - start_pose_.getOrigin().getX(), 2));
    double Ierror = numerator/denominator;
    return -1 * config_.p_weight_angular * Perror + -1 * config_.i_weight_angular * Ierror;
  }

  bool DeadReckoningController::isGoalReached(const tf::Stamped<tf::Pose> current_pose){
    if((current_pose.getOrigin().getX() * path_.getOrigin().getX() + current_pose.getOrigin().getY() * path_.getOrigin().getY()) > 
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
