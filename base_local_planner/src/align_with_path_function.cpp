#include <base_local_planner/align_with_path_function.h>
#include <math.h>       /* atan2 */
#include <angles/angles.h>

namespace base_local_planner {

constexpr double MIN_ROT_SPEED = 0.1;
constexpr double PENALTY_COST = 1e6;

AlignWithPathFunction::AlignWithPathFunction() : current_yaw_diff_(0) {}

void AlignWithPathFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped>& target_poses, const geometry_msgs::PoseStamped& global_pose) {
  current_yaw_diff_ = 0;
  const int num_points = target_poses.size();
  if (num_points <= 0) {
    return;
  }
  // todo: decide which point to pick
  geometry_msgs::PoseStamped path_node =  *(target_poses.begin() + std::min(3, num_points-1));
  const double path_yaw = 2 * atan2 (path_node.pose.orientation.z, path_node.pose.orientation.w);

  const double current_yaw = 2 * atan2(global_pose.pose.orientation.z, global_pose.pose.orientation.w);
  current_yaw_diff_ = angles::normalize_angle(path_yaw - current_yaw);
}

mbf_msgs::ExePathResult::_outcome_type AlignWithPathFunction::prepare() {
  return mbf_msgs::ExePathResult::SUCCESS;;
}

double AlignWithPathFunction::scoreTrajectory(Trajectory &traj) {
  if (!isTurningRequired()) {
    return 0;
  }

  return std::abs(angles::normalize_angle(current_yaw_diff_ - traj.thetav_ * PREDICTION_TIME));
}

} /* namespace base_local_planner */
