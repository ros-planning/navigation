/*
 * curvature_cost_function.cpp
 */

#include <base_local_planner/curvature_cost_function.h>
#include <math.h>

namespace base_local_planner {

void CurvatureCostFunction::setCargoLimitAngleDeg(double cargo_limit_angle_deg)
{
  this->cargo_limit_angle_deg_ = cargo_limit_angle_deg;
}

void CurvatureCostFunction::setCurvatureRadius(double curvature_radius)
{
  this->curvature_radius_ = curvature_radius;
}

void CurvatureCostFunction::setCargoAngle(double cargo_angle)
{
  this->cargo_angle_ = cargo_angle;
}

void CurvatureCostFunction::setCargoEnabled(bool is_cargo_enabled)
{
  this->is_cargo_enabled_ = is_cargo_enabled;
}

double CurvatureCostFunction::scoreTrajectory(Trajectory &traj) {

  if (!this->is_cargo_enabled_ || this->curvature_radius_ <= 0.0)
  {
    return 0.0;
  }

  double norm_cargo_angle = std::fmod(cargo_angle_ + 2 * M_PI, 2 * M_PI) - M_PI;

  if (traj.thetav_ == 0.0)
  {
    return 0.0;
  }
  else if (std::fabs(traj.xv_ / traj.thetav_) < this->curvature_radius_)
  {
    if (std::fabs(norm_cargo_angle) < this->cargo_limit_angle_deg_ * M_PI / 180)
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
