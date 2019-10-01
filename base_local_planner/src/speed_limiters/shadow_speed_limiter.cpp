/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, 6 River Systems
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Daniel Grieneisen
 *********************************************************************/

#include <base_local_planner/speed_limiters/shadow_speed_limiter.h>
#include <base_local_planner/geometry_math_helpers.h>
#include <tf/transform_datatypes.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/Vector3.h>

namespace base_local_planner {

void ShadowSpeedLimiter::initialize(std::string name)
{
  map_grid_ = MapGrid(costmap_->getCostmap()->getSizeInCellsX(), costmap_->getCostmap()->getSizeInCellsY());
  map_grid_.reject_inscribed_cost_ = false;

  ros::NodeHandle private_nh(name + "/shadow");
  configClient_ = std::make_shared<dynamic_reconfigure::Client<ShadowSpeedLimiterConfig>>(name + "/shadow");
  configClient_->setConfigurationCallback(boost::bind(&ShadowSpeedLimiter::reconfigure, this, _1));
  initialized_ = true;
}

std::string ShadowSpeedLimiter::getName(){
  return std::string("Shadow");
}

bool ShadowSpeedLimiter::calculateLimits(double& max_allowed_linear_vel, double& max_allowed_angular_vel) {
  if (!initialized_)
  {
    return false;
  }
  // Reset the maximum allowed velocity
  max_allowed_linear_vel = max_linear_velocity_;
  max_allowed_angular_vel = max_angular_velocity_;

  // Get the objects
  auto objects = costmap_->getLayeredCostmap()->getShadowedObjects();
  if (!objects)
  {
    ROS_WARN_THROTTLE(1.0, "No objects in shadow speed limiter");
    return false;
  }

  tf::Stamped<tf::Pose> current_pose;
  if (!getCurrentPose(current_pose))
  {
    ROS_WARN_THROTTLE(1.0, "No pose in shadow speed limiter");
    return false;
  }
  // Adjust the pose to be at the front of the robot.
  geometry_msgs::PoseStamped pose;
  tf::poseStampedTFToMsg(current_pose, pose);

  geometry_msgs::PoseStamped pose_no_offset;
  tf::poseStampedTFToMsg(current_pose, pose_no_offset);

  double pose_yaw = tf::getYaw(pose.pose.orientation);
  pose.pose.position.x += cos(pose_yaw) * params_.forward_offset;
  pose.pose.position.y += sin(pose_yaw) * params_.forward_offset;

  // calculate the brushfire grid.
  map_grid_.resetPathDist();
  map_grid_.setUnadjustedGoal(*(costmap_->getCostmap()), pose);

  double points_inside = 0;
  double points_total = 0;
  double max_vel = max_linear_velocity_;
  // Find nearest object via brushfire distance

  double max_cost = map_grid_.obstacleCosts();
  for (const auto& obj : (*objects))
  {	
	// Get the brushfire distance to the obstacle
   	double distance = getMapGridDistance(obj)/costmap_->getCostmap()->getResolution();
    if (fabs(getAngle(pose_no_offset, obj)) < params_.half_angle && distance < max_cost)
    {	
        // Convert it to a velocity
      double distance = getMapGridDistance(obj);
      double velocity = distanceToVelocity(distance);
      max_vel = std::min(velocity, max_vel);

      points_inside++;
    }
    points_total++;
  }
  if (points_inside / points_total > params_.shadow_threshold)
  {
		max_allowed_linear_vel = max_vel;
  }
  double percentage = points_inside/points_total;
  //ROS_INFO_THROTTLE(0.1, "Shadow at %f, of %f points", percentage, points_total);
  ROS_DEBUG_THROTTLE(0.2, "Setting shadow max speed to %f, %f", max_allowed_linear_vel, max_allowed_angular_vel);

  return true;
}

double ShadowSpeedLimiter::getAngle(geometry_msgs::PoseStamped pose, geometry_msgs::Point obj)
{ 
  double pose_yaw = tf::getYaw(pose.pose.orientation);
  geometry_msgs::Vector3 pose_vec;
  pose_vec.x = cos(pose_yaw);
  pose_vec.y = sin(pose_yaw);
  geometry_msgs::Vector3 obj_vec;
  obj_vec.x = obj.x - pose.pose.position.x;
  obj_vec.y = obj.y - pose.pose.position.y;
  double angle = acos((pose_vec.x * obj_vec.x + pose_vec.y * obj_vec.y) / (sqrt(obj_vec.x * obj_vec.x + obj_vec.y * obj_vec.y)));
  return angle;
}
double ShadowSpeedLimiter::getMapGridDistance(geometry_msgs::Point obj) 
{

  unsigned int px, py;
  if (!costmap_->getCostmap()->worldToMap(obj.x, obj.y, px, py)) 
  {
    //we're off the map
    ROS_WARN("Off Map %f, %f", obj.x, obj.y);
    return std::numeric_limits<double>::max();
  }
  double grid_dist = map_grid_(px, py).target_dist;
  double out_dist = grid_dist * costmap_->getCostmap()->getResolution();
  
  return out_dist;
}

double ShadowSpeedLimiter::distanceToVelocity(double dist)
{
  return threeLevelInterpolation(dist, 
    params_.min_range, params_.nominal_range_min,
    params_.nominal_range_max, params_.max_range,
    std::min(params_.min_linear_velocity, max_linear_velocity_),
    std::min(params_.nominal_linear_velocity, max_linear_velocity_),
    max_linear_velocity_
    );
}

} /* namespace base_local_planner */
