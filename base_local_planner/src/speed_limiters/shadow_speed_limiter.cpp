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

namespace base_local_planner {

void ShadowSpeedLimiter::initialize(std::string name)
{
  map_grid_ = MapGrid(costmap_->getCostmap()->getSizeInCellsX(), costmap_->getCostmap()->getSizeInCellsY());
  map_grid_.reject_inscribed_cost_ = false;

  ros::NodeHandle private_nh(name + "/shadow");
  configServer_ = std::make_shared<dynamic_reconfigure::Server<ShadowSpeedLimiterConfig>>(private_nh);
  configServer_->setCallback(boost::bind(&ShadowSpeedLimiter::reconfigure, this, _1, _2));

  initialized_ = true;
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

  double pose_yaw = tf::getYaw(pose.pose.orientation);
  pose.pose.position.x += cos(pose_yaw) * params_.forward_offset;
  pose.pose.position.y += sin(pose_yaw) * params_.forward_offset;

  // calculate the brushfire grid.
  map_grid_.resetPathDist();
  map_grid_.setUnadjustedGoal(*(costmap_->getCostmap()), pose);

  // Find nearest object via brushfire distance
  for (const auto& obj : (*objects))
  {
    // Get the brushfire distance to the obstacle
    double distance = getMapGridDistance(obj);
    // Convert it to a velocity
    double velocity = distanceToVelocity(distance);

    max_allowed_linear_vel = std::min(velocity, max_allowed_linear_vel);
  }

  ROS_DEBUG_THROTTLE(0.2, "Setting shadow max speed to %f, %f", max_allowed_linear_vel, max_allowed_angular_vel);

  return true;
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

  ROS_DEBUG("x: %f, y: %f, grid_dist %f, out_dist %f", obj.x, obj.y, grid_dist, out_dist);
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
