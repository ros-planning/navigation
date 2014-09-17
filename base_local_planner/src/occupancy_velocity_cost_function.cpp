/*********************************************************************
 * Author: Rein Appeldoorn
 *********************************************************************/

#include <base_local_planner/occupancy_velocity_cost_function.h>
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>

namespace base_local_planner {

OccupancyVelocityCostFunction::OccupancyVelocityCostFunction(costmap_2d::Costmap2D* costmap) : costmap_(costmap)
{
    if (costmap != NULL)
    {
        world_model_ = new base_local_planner::CostmapModel(*costmap_);
    }
}

OccupancyVelocityCostFunction::~OccupancyVelocityCostFunction()
{
    if (world_model_ != NULL)
    {
        delete world_model_;
    }
}

double OccupancyVelocityCostFunction::scoreTrajectory(Trajectory &traj)
{
    if (footprint_spec_.size() == 0)
    {
        ROS_ERROR("Footprint spec is empty, maybe missing call to setFootprint?");
        return -9;
    }

    double x, y, th;
    for (unsigned int i = 0; i < traj.getPointsSize(); ++i)
    {
        traj.getPoint(i, x, y, th);

        double footprint_cost = world_model_->footprintCost(x, y, th, footprint_spec_);

        // We are in collision: discard trajectory
        if (footprint_cost < 0)
            return -6.0;

        // Check the occupancy of the center point
        unsigned int cell_x, cell_y;
        if ( ! costmap_->worldToMap(x, y, cell_x, cell_y))
            return -7.0;

        double center_cost = costmap_->getCost(cell_x, cell_y);
        double max_vel = (1 - center_cost / 255.0) * max_trans_vel_;

        if (hypot(traj.xv_, traj.yv_) > max_vel && i == traj.getPointsSize() - 1) // Only last point
            return -5.0;
    }

    // Trajectory is valid
    return 0.0;
}

} /* namespace base_local_planner */
