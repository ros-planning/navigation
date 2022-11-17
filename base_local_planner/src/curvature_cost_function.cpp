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

  
  double delta_angle, next_cargo_angle;
  if (traj.getPointsSize() <= 1)
  {
    delta_angle = traj.thetav_;
    next_cargo_angle = base_local_planner::normalize_angle(cargo_angle_ + traj.thetav_);
  }
  else
  {
    double px0, py0, pth0, px1, py1, pth1;
    traj.getPoint(0, px0, py0, pth0);
    traj.getPoint(1, px1, py1, pth1);

    if (px0 == px1 && py0 == py1)
    {
      delta_angle = 0.0;
    }
    else
    {
      double cargo_global = base_local_planner::normalize_angle(pth0 + this->cargo_angle_);
      double cargo_rear_x = px0 + std::cos(cargo_global) * 0.95;
      double cargo_rear_y = py0 + std::sin(cargo_global) * 0.95;
      double cur_cargo_th = std::atan2(py0 - cargo_rear_y, px0 - cargo_rear_x);
      double next_cargo_th = std::atan2(py1 - cargo_rear_y, px1 - cargo_rear_x);
      delta_angle = base_local_planner::normalize_angle(next_cargo_th - cur_cargo_th);
      next_cargo_angle = base_local_planner::normalize_angle(cargo_global + delta_angle - pth1);
    }
  }

  if (std::fabs(next_cargo_angle) < this->cargo_limit_angle_deg_ * M_PI / 180)
  {
    if (std::fabs(cargo_angle_) < std::fabs(next_cargo_angle))
    {
      return 0.0;
    }
    return -12;
  }

  if (delta_angle == 0.0)
  {
    return 0.0;
  }
  else if (std::fabs(traj.xv_ / delta_angle) < this->curvature_radius_)
  {
    return -11;
  }
  return 0.0;
}

} /* namespace base_local_planner */
