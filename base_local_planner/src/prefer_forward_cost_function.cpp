/*
 * prefer_forward_cost_function.cpp
 *
 *  Created on: Apr 4, 2012
 *      Author: tkruse
 */

#include <base_local_planner/prefer_forward_cost_function.h>

#include <math.h>

namespace base_local_planner {


double PreferForwardCostFunction::scoreTrajectory(Trajectory &traj) {
  // backward motions bad on a robot without backward sensors
  if (traj.xv_ < 0.0) {
    return -traj.xv_;
  }
  return 0;
}

} /* namespace base_local_planner */
