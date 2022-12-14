#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/trajectory_cost_function.h>

namespace base_local_planner {

constexpr double MAX_ANGLE_ERROR = 0.8;
constexpr double PREDICTION_TIME = 1.5;

class AlignWithPathFunction : public base_local_planner::TrajectoryCostFunction {
public:
  AlignWithPathFunction();

  void setTargetPoses(std::vector<geometry_msgs::PoseStamped>& target_poses, const geometry_msgs::PoseStamped& global_pose);

  ExePathOutcome prepare();

  double scoreTrajectory(Trajectory &traj);

  bool isTurningRequired() const {
    return std::abs(current_yaw_diff_) > MAX_ANGLE_ERROR;
  }

private:
  double current_yaw_diff_;
};

} /* namespace base_local_planner */
