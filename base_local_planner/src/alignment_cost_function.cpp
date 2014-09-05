#include "base_local_planner/alignment_cost_function.h"

#define PI 3.14159

namespace base_local_planner {

AlignmentCostFunction::AlignmentCostFunction():
  theta_desired_(0.0)
{

}

AlignmentCostFunction::~AlignmentCostFunction()
{

}

double AlignmentCostFunction::scoreTrajectory(Trajectory &traj)
{
  /// Check trajectory length
  if (traj.getPointsSize() == 0)
  {
    return 0.0;
  }
  else
  {
    double x, y, theta;
    traj.getEndpoint(x, y, theta);
    double error = theta_desired_-theta;
    /// Keep error between -pi and pi
    if (error < -PI) {error += 2*PI; }
    else if (error >  PI) {error -= 2*PI; }

    return fabs(error);
  }
}

}
