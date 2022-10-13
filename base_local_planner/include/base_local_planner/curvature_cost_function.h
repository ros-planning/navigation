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

  CurvatureCostFunction() :
    cargo_limit_angle_deg_(90.0), curvature_radius_(0.9), cargo_angle_(0.0), is_cargo_enabled_(false) {}
  ~CurvatureCostFunction() {}

  double scoreTrajectory(Trajectory &traj);

  void setCargoLimitAngleDeg(double cargo_limit_angle_deg);
  void setCurvatureRadius(double curvature_radius);
  void setCargoAngle(double cargo_angle);
  void setCargoEnabled(bool is_cargo_enabled_);

  bool prepare() {return true;};

private:
  double cargo_limit_angle_deg_;
  double curvature_radius_;
  double cargo_angle_;
  bool is_cargo_enabled_;
};

} /* namespace base_local_planner */
#endif /* CURVATURE_COST_FUNCTION_H_ */
