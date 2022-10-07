#ifndef CURVATURE_COST_FUNCTION_H
#define CURVATURE_COST_FUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>
#include <ros/ros.h>
namespace base_local_planner {

/**
 * This class provides a cost based on how much a robot "twirls" on its
 * way to the goal. With differential-drive robots, there isn't a choice,
 * but with holonomic or near-holonomic robots, sometimes a robot spins
 * more than you'd like on its way to a goal. This class provides a way
 * to assign a penalty purely to rotational velocities.
 */
class CurvatureCostFunction: public base_local_planner::TrajectoryCostFunction {
public:

  CurvatureCostFunction() {}
  ~CurvatureCostFunction() {}

  double scoreTrajectory(Trajectory &traj);
  void setCargoAngle(double cargo_angle);
  void setCargoEnabled(bool is_cargo_enabled_);

  bool prepare() {return true;};

private:
  double cargo_angle_ = 0.0;
  bool is_cargo_enabled_;
};

} /* namespace base_local_planner */
#endif /* CURVATURE_COST_FUNCTION_H_ */
