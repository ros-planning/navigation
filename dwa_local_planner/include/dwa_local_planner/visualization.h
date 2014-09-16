#ifndef dwa_local_planner_visualization_h_
#define dwa_local_planner_visualization_h_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

#include <pcl_ros/publisher.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/map_grid_visualizer.h>
#include <base_local_planner/obstacle_cost_function.h>

#include <base_local_planner/trajectory.h>

#include <costmap_2d/costmap_2d.h>

namespace dwa_local_planner
{

/**
 * @class Class with visualization functions for the dwa local planner
 * @brief Implements some visualization functions (ROS vis messages)
 */
class Visualization
{

public:
    /**
     * @brief  Constructor for the Visualization object (inits the publishers)
     */
    Visualization(costmap_2d::Costmap2D* costmap,
                  base_local_planner::MapGridCostFunction& goal_cost,
                  base_local_planner::MapGridCostFunction& plan_cost,
                  const std::string& frame);

    /**
     * @brief  Publishes goal pose marker
     * @param  yaw desired yaw
     * @param  robot_pose current robot pose in specified frame
     * @param  frame coordinate frame
     */
    void publishDesiredOrientation(double yaw, const tf::Stamped<tf::Pose>& robot_pose, const std::string& frame = "/map");

    /**
     * @brief  Visualization generated trajectories
     * @param  trajectories All trajectories
     */
    void publishTrajectoryCloud(const std::vector<base_local_planner::Trajectory>& trajectories, const std::string& frame = "/map");   

    void publishCostGrid();
    bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

private:
    ros::Publisher desired_orientation_pub_;
    pcl_ros::Publisher<base_local_planner::MapGridCostPoint> traj_cloud_pub_;

    base_local_planner::MapGridVisualizer map_grid_visualizer_;
    costmap_2d::Costmap2D* costmap_;

    base_local_planner::MapGridCostFunction& goal_cost_;
    base_local_planner::MapGridCostFunction& plan_cost_;

    std::string frame_;

};

}
#endif

