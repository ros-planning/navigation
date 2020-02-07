#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/trajectory_cost_function.h>

namespace base_local_planner {

class AlignWithPathFunction : public base_local_planner::TrajectoryCostFunction {
public:
  AlignWithPathFunction();

  void setTargetPoses(std::vector<geometry_msgs::PoseStamped>& target_poses, const geometry_msgs::PoseStamped& global_pose);

  bool prepare();

  double scoreTrajectory(Trajectory &traj);

private:
  bool current_yaw_diff_positive_;
};

} /* namespace base_local_planner */
