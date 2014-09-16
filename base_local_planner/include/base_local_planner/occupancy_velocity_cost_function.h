/*********************************************************************
 * Author: Rein Appeldoorn
 *********************************************************************/

#ifndef OCCUPANCY_VELOCITY_COST_FUNCTION_H_
#define OCCUPANCY_VELOCITY_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d.h>

namespace base_local_planner {

/**
 * class OccupancyVelocityCostFunction
 * @brief Uses costmap 2d to assign negative costs if robot footprint in trajectory crosses a cell in which this velocity is not allowed
 */
class OccupancyVelocityCostFunction : public TrajectoryCostFunction {

public:
    OccupancyVelocityCostFunction(costmap_2d::Costmap2D* costmap);
    ~OccupancyVelocityCostFunction();

    bool prepare() { return true; }
    double scoreTrajectory(Trajectory &traj);

    void setParams(double max_trans_vel) { max_trans_vel_ = max_trans_vel; }
    void setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec) { footprint_spec_ = footprint_spec; }

private:
    costmap_2d::Costmap2D* costmap_;
    std::vector<geometry_msgs::Point> footprint_spec_;
    base_local_planner::WorldModel* world_model_;
    double max_trans_vel_;
};

} /* namespace base_local_planner */

#endif /* OCCUPANCY_VELOCITY_COST_FUNCTION_H_ */
