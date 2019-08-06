#include <base_local_planner/align_with_path_function.h>
#include <math.h>       /* atan2 */
#include <angles/angles.h>

namespace base_local_planner {

const double MAX_ANGLE_ERROR = 0.8;
const double MIN_ROT_SPEED = 0.1;
const double MIN_GOAL_DIST_SQ = 0.7;

AlignWithPathFunction::AlignWithPathFunction() : target_pose_valid_(false) {}

void AlignWithPathFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped>& target_poses) {
  const int num_points = target_poses.size();
  if (num_points <= 0) {
    target_pose_valid_ = false;
    return;
  }
  target_pose_valid_ = true;
  // todo: decide which point to pick
  geometry_msgs::PoseStamped path_node =  *(target_poses.begin() + std::min(3, num_points-1));
  path_yaw_ = 2 * atan2 (path_node.pose.orientation.z, path_node.pose.orientation.w);
  goal_x_ = target_poses.back().pose.position.x;
  goal_y_ = target_poses.back().pose.position.y;
}

bool AlignWithPathFunction::prepare() {
  return true;
}

double AlignWithPathFunction::scoreTrajectory(Trajectory &traj) {
  if (!target_pose_valid_) {
    return 0;
  }

  if (traj.getPointsSize() <= 0) {
    return 0;
  }

  double px, py, pth;
  traj.getPoint(0, px, py, pth);
  const double squared_dist = (goal_x_ - px) * (goal_x_ - px) + (goal_y_ - py) * (goal_y_ - py);
  // allow backwards motion close to goal
  if (squared_dist < MIN_GOAL_DIST_SQ) {
    return 0;
  }

  const double angle_diff = angles::normalize_angle(path_yaw_ - pth);
  // if angle off by more than MAX_ANGLE_ERROR, force to rotate towards path with more than MIN_ROT_SPEED
  if (angle_diff > MAX_ANGLE_ERROR && traj.thetav_ < MIN_ROT_SPEED) {
    return -1;
  }
  if (angle_diff < -MAX_ANGLE_ERROR && traj.thetav_ > -MIN_ROT_SPEED) {
    return -1;
  }
  return 0;
}

} /* namespace base_local_planner */