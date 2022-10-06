/*
 * curvature_cost_function.cpp
 */

#include <base_local_planner/curvature_cost_function.h>
#include <ros/ros.h>
#include <math.h>

namespace base_local_planner {

void CurvatureCostFunction::setCargoAngle(double cargo_angle)
{
  this->cargo_angle_ = cargo_angle;
}

double CurvatureCostFunction::scoreTrajectory(Trajectory &traj) {
  double norm_cargo_angle = std::fmod(cargo_angle_ + 2 * M_PI, 2 * M_PI) - M_PI;

  // ROS_WARN_STREAM("xv:" << traj.xv_ << " yv:" << traj.yv_ << " thetav:" << traj.thetav_ << " cargo_angle_:" << cargo_angle_ << " norm_cargo_angle:" << norm_cargo_angle);

  if (traj.thetav_ == 0.0)
  {
    return 0.0;
  }
  else if (std::fabs(traj.xv_ / traj.thetav_) < 0.9)
  {
    

    if (std::fabs(norm_cargo_angle) < 90 * M_PI / 180)
    {
      return 0.0;
    }
    else if (norm_cargo_angle < 0 && traj.thetav_ < 0)
    {
      return 0.0;
    }
    else if (0 < norm_cargo_angle && 0 < traj.thetav_)
    {
      return 0.0;
    }
    return -11;
  }
  return 0.0;
}

} /* namespace base_local_planner */
