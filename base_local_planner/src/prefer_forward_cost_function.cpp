/*
 * prefer_forward_cost_function.cpp
 *
 *  Created on: Apr 4, 2012
 *      Author: tkruse
 */

#include <base_local_planner/prefer_forward_cost_function.h>

namespace base_local_planner {


double PreferForwardCostFunction::scoreTrajectory(Trajectory &traj) {
  if (traj.xv_ < 0.0) {
    return penalty_;
  }
  if (traj.xv_ < 0.1 && traj.thetav_ < 0.2) {
    return penalty_;
  }
  return 0.0;
}

} /* namespace base_local_planner */
