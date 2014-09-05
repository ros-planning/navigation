#include "base_local_planner/cmd_vel_cost_function.h"

namespace base_local_planner {

CmdVelCostFunction::CmdVelCostFunction() :
  px_(0.0),
  nx_(0.0),
  py_(0.0),
  ny_(0.0),
  ptheta_(0.0),
  ntheta_(0.0)
{

}

CmdVelCostFunction::~CmdVelCostFunction()
{

}

double CmdVelCostFunction::scoreTrajectory(Trajectory &traj)
{
  double costs = 0.0;
  if (fabs(traj.xv_) > 0)
    costs += px_*traj.xv_;
  if (fabs(traj.xv_) < 0)
    costs += nx_*fabs(traj.xv_);
  if (fabs(traj.yv_) > 0)
    costs += py_*traj.yv_;
  if (fabs(traj.yv_) < 0)
    costs += ny_*fabs(traj.yv_);
  if (fabs(traj.thetav_) > 0)
    costs += ptheta_*traj.thetav_;
  if (fabs(traj.thetav_) < 0)
    costs += ntheta_*fabs(traj.thetav_);
  return costs;
}

}
