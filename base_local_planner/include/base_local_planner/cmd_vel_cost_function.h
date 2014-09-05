#ifndef base_local_planner_cmd_vel_cost_function_h_
#define base_local_planner_cmd_vel_cost_function_h_

#include "trajectory_cost_function.h"
#include <Eigen/Core>

namespace base_local_planner
{

class CmdVelCostFunction: public TrajectoryCostFunction
{

public:

  CmdVelCostFunction();
  virtual ~CmdVelCostFunction();

  double scoreTrajectory(Trajectory &traj);

  bool prepare() { return true; }

  /** Sets coefficients of the cost function
    * @param px Positive x direction
    * @param nx Negative x direction
    * @param py Positive y direction
    * @param ny Negative y direction
    * @param ptheta Positive theta direction
    * @param ntheta Negative theta direction */
  void setCoefficients(double px, double nx, double py, double ny, double ptheta, double ntheta)
  {
    px_ = px;
    nx_ = nx;
    py_ = py;
    ny_ = ny;
    ptheta_ = ptheta;
    ntheta_ = ntheta;
  }

private:

  //! Cost function coefficients
  double px_, nx_, py_, ny_, ptheta_, ntheta_;

};

}// namespace

#endif
