#ifndef base_local_planner_alignment_cost_function_h_
#define base_local_planner_alignment_cost_function_h_

#include "trajectory_cost_function.h"
#include <Eigen/Core>

namespace base_local_planner
{

class AlignmentCostFunction: public TrajectoryCostFunction
{

public:

  AlignmentCostFunction();
  virtual ~AlignmentCostFunction();

  double scoreTrajectory(Trajectory &traj);

  bool prepare() {return true;}

  void setDesiredOrientation(double theta) {theta_desired_ = theta;}
  double getDesiredOrientation() { return theta_desired_; }

private:

  //! Desired orientation
  double theta_desired_;

};

}// namespace

#endif
