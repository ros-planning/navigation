#include <base_local_planner/align_with_path_function.h>
#include <math.h>       /* atan2 */
#include <angles/angles.h>

namespace base_local_planner {

constexpr double MAX_ANGLE_ERROR = 0.8;
constexpr double MIN_ROT_SPEED = 0.1;
constexpr double MIN_GOAL_DIST_SQ = 0.7;
constexpr double PENALTY_COST = 1e6;

AlignWithPathFunction::AlignWithPathFunction() : current_yaw_diff_positive_(true) {}

void AlignWithPathFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped>& target_poses, const geometry_msgs::PoseStamped& global_pose) {
  setScale(0);

  const int num_points = target_poses.size();
  if (num_points <= 0) {
    return;
  }
  // todo: decide which point to pick
  geometry_msgs::PoseStamped path_node =  *(target_poses.begin() + std::min(3, num_points-1));
  const double path_yaw = 2 * atan2 (path_node.pose.orientation.z, path_node.pose.orientation.w);
  const double goal_dx = target_poses.back().pose.position.x - global_pose.pose.position.x;
  const double goal_dy = target_poses.back().pose.position.y - global_pose.pose.position.y;

  const double squared_dist = goal_dx * goal_dx + goal_dy * goal_dy;
  if (squared_dist < MIN_GOAL_DIST_SQ)
  {
    return;
  }

  const double current_yaw = 2 * atan2(global_pose.pose.orientation.z, global_pose.pose.orientation.w);
  const double current_yaw_diff = angles::normalize_angle(path_yaw - current_yaw);

  current_yaw_diff_positive_ = current_yaw_diff > 0;

  if (std::abs(current_yaw_diff) < MAX_ANGLE_ERROR)
  {
    return;
  }
  setScale(1);
}

bool AlignWithPathFunction::prepare() {
  return true;
}

double AlignWithPathFunction::scoreTrajectory(Trajectory &traj) {
  // if angle off by more than MAX_ANGLE_ERROR, force to rotate towards path with more than MIN_ROT_SPEED
  if (current_yaw_diff_positive_ && traj.thetav_ < MIN_ROT_SPEED) {
    return PENALTY_COST;
  }
  if (!current_yaw_diff_positive_ && traj.thetav_ > -MIN_ROT_SPEED) {
    return PENALTY_COST;
  }
  return 0;
}

} /* namespace base_local_planner */
