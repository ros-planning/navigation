/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, 6 River Systems
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

#include <base_local_planner/speed_limiters/path_speed_limiter.h>
#include <base_local_planner/geometry_math_helpers.h>
#include <tf/transform_datatypes.h>

namespace base_local_planner {

void PathSpeedLimiter::initialize(std::string name) {
  ros::NodeHandle private_nh(name + "/path");
  configServer_ = std::make_shared<dynamic_reconfigure::Server<PathSpeedLimiterConfig>>(private_nh);
  configServer_->setCallback(boost::bind(&PathSpeedLimiter::reconfigure, this, _1, _2));
}

std::string PathSpeedLimiter::getName(){
  return std::string("Path");
}

bool PathSpeedLimiter::calculateLimits(double& max_allowed_linear_vel, double& max_allowed_angular_vel) {
  // Reset the maximum allowed velocity
  max_allowed_linear_vel = max_linear_velocity_;
  max_allowed_angular_vel = max_angular_velocity_;

  if (plan_.empty())
  {
    ROS_WARN_THROTTLE(1.0, "No global plan.");
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

  // Determine the closest point on the global plan.
  // Then look forwards along the path a fixed distance.
  // Find the greatest heading difference to the robot's current heading.
  // Scale the speed based on the heading.

  Eigen::Vector3f current_pose_vec = poseToVector3f(pose.pose);
  Eigen::Vector2f pos2 = Eigen::Vector2f::Zero();
  pos2[0] = current_pose_vec[0];
  pos2[1] = current_pose_vec[1];
  double current_heading = current_pose_vec[2];
  
  // Vectors for storage
  Eigen::Vector2f p0 = Eigen::Vector2f::Zero();
  Eigen::Vector2f p1 = Eigen::Vector2f::Zero();
  Eigen::Vector2f segment_vec = Eigen::Vector2f::Zero();
  Eigen::Vector2f pose_to_point_vec = Eigen::Vector2f::Zero();

  // Other storage
  double distance_from_robot = 0;
  double minimum_distance = params_.max_distance_from_path; // it needs to be at least this close
  double max_heading_diff = 0;

  Eigen::Vector3f plan_vec = poseToVector3f(plan_[0].pose);

  for (size_t k = 0; k < plan_.size() - 1; ++k)
  {
    ROS_DEBUG("Searching along plan at k=%zu", k);
    // Pull out the datas
    p0 = poseStampedToVector(plan_[k]);
    p1 = poseStampedToVector(plan_[k + 1]);
    segment_vec = p1 - p0;
    double segment_length = segment_vec.norm();
    double dist_from_path = distanceToLineSegment(pos2, p0, p1);
    
    pose_to_point_vec = p0 - pos2;
    double bearing_to_point = atan2(pose_to_point_vec[1], pose_to_point_vec[0]);

    if (dist_from_path < minimum_distance)
    {
      minimum_distance = dist_from_path;
      // Reset distance
      distance_from_robot = std::max(0.0, segment_length - distanceAlongLineSegment(pos2, p0, p1));
    }
    else
    {
      if (distance_from_robot >= params_.min_lookahead_distance 
        && distance_from_robot < params_.max_lookahead_distance)
      {
        max_heading_diff = std::max(max_heading_diff, 
          std::fabs(angleMinusPiToPi(bearing_to_point - current_heading)));
      }
      else if (distance_from_robot > params_.max_lookahead_distance)
      {
        break;
      }
      distance_from_robot += segment_length;
    }
  }

  ROS_DEBUG_THROTTLE(0.2, "Max heading diff: %f", max_heading_diff);
  max_allowed_linear_vel = calculateAllowedLinearSpeed(max_heading_diff);

  ROS_DEBUG_THROTTLE(0.2, "Setting path max speed to %f, %f", max_allowed_linear_vel, max_allowed_angular_vel);

  return true;
}

double PathSpeedLimiter::calculateAllowedLinearSpeed(double heading_diff)
{
  return twoLevelInterpolation(heading_diff, 
    params_.min_heading_difference, params_.max_heading_difference,
    max_linear_velocity_,
    std::min(params_.min_linear_velocity, max_linear_velocity_)
    );
}

} /* namespace base_local_planner */
