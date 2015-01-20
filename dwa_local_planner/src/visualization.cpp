#include "dwa_local_planner/visualization.h"

namespace dwa_local_planner {

Visualization::Visualization(costmap_2d::Costmap2D* costmap,
                             base_local_planner::MapGridCostFunction& goal_cost,
                             base_local_planner::MapGridCostFunction& plan_cost,
                             const std::string& frame) :
    costmap_(costmap),
    goal_cost_(goal_cost),
    plan_cost_(plan_cost),
    frame_(frame)
{
    ros::NodeHandle nh("~/visualization");
    desired_orientation_pub_ = nh.advertise<visualization_msgs::Marker>("markers/desired_orientation_marker",1);

    // Visualization
    traj_cloud_pub_.advertise(nh, "trajectory_cloud", 1);

    map_grid_visualizer_.initialize("dwa_planner", frame_ , boost::bind(&Visualization::getCellCosts, this, _1, _2, _3, _4, _5, _6));
}

bool Visualization::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {

    path_cost = plan_cost_.getCellCosts(cx, cy);
    goal_cost = goal_cost_.getCellCosts(cx, cy);
    if (goal_cost > 300.0) {
        goal_cost = 255.0;
    }
    if (path_cost > 300.0) {
        path_cost = 255.0;
    }
    occ_cost = costmap_->getCost(cx, cy);
    total_cost = goal_cost + path_cost + occ_cost;
    if (occ_cost == 0)
    {
        path_cost = plan_cost_.getCellCosts(cx, cy);
        goal_cost = goal_cost_.getCellCosts(cx, cy);
    }

//    if (path_cost == path_costs_.obstacleCosts() ||
//            path_cost == path_costs_.unreachableCellCosts() ||
//            occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
//        return false;
//    }

//    double resolution = planner_util_->getCostmap()->getResolution();
//    total_cost =
//            pdist_scale_ * resolution * path_cost +
//            gdist_scale_ * resolution * goal_cost +
//            occdist_scale_ * occ_cost;
    return true;
}

void Visualization::publishCostGrid()
{
    map_grid_visualizer_.publishCostCloud(costmap_);
}

void Visualization::publishDesiredOrientation(double yaw, const tf::Stamped<tf::Pose>& robot_pose, const std::string& frame)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.scale.x = .8;
    marker.scale.y = .1;
    marker.scale.z = .1;
    marker.color.a = 1;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.type = visualization_msgs::Marker::ARROW;
    tf::poseTFToMsg(robot_pose, marker.pose);
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    marker.action = visualization_msgs::Marker::ADD;
    desired_orientation_pub_.publish(marker);
}

void Visualization::publishTrajectoryCloud(const std::vector<base_local_planner::Trajectory>& trajectories, const std::string& frame)
{
    pcl::PointCloud<base_local_planner::MapGridCostPoint> traj_cloud;
    traj_cloud.header.frame_id = frame;

    base_local_planner::MapGridCostPoint pt;
    traj_cloud.width = 0;
    traj_cloud.height = 0;
    std_msgs::Header header;
    pcl_conversions::fromPCL(traj_cloud.header, header);
    header.stamp = ros::Time::now();
    traj_cloud.header = pcl_conversions::toPCL(header);
    bool valid = false;
    for(std::vector<base_local_planner::Trajectory>::const_iterator t=trajectories.begin(); t != trajectories.end(); ++t)
    {
        if (t->cost_ < 0)
            continue;

        valid = true;

        // Fill out the plan
        for (unsigned int i = 0; i < t->getPointsSize(); ++i) {
            double p_x, p_y, p_th;
            t->getPoint(i, p_x, p_y, p_th);
            pt.x = p_x;
            pt.y = p_y;
            pt.z = 0;
            pt.total_cost = t->cost_;
            traj_cloud.push_back(pt);
        }
    }
    traj_cloud_pub_.publish(traj_cloud);
}

}
