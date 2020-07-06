#include "dwa_local_planner/visualization.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf2/utils.h>

#include <visualization_msgs/Marker.h>

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
    traj_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("trajectory_cloud", 1);

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

//    path_cost = plan_costs_.getCellCosts(cx, cy);
//    goal_cost = goal_costs_.getCellCosts(cx, cy);
//    occ_cost = planner_util_->getCostmap()->getCost(cx, cy);
//    if (path_cost == plan_costs_.obstacleCosts() ||
//        path_cost == plan_costs_.unreachableCellCosts() ||
//        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
//      return false;
//    }

//    total_cost =
//        path_distance_bias_ * path_cost +
//        goal_distance_bias_ * goal_cost +
//        occdist_scale_ * occ_cost;
    return true;
}

void Visualization::publishCostGrid()
{
    map_grid_visualizer_.publishCostCloud(costmap_);
}

void Visualization::publishDesiredOrientation(double yaw, const geometry_msgs::PoseStamped& robot_pose, const std::string& frame)
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
    marker.pose = robot_pose.pose;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    marker.pose.orientation = tf2::toMsg(q);
    marker.action = visualization_msgs::Marker::ADD;
    desired_orientation_pub_.publish(marker);
}

void Visualization::publishTrajectoryCloud(const std::vector<base_local_planner::Trajectory>& trajectories, const std::string& frame)
{
  sensor_msgs::PointCloud2 traj_cloud;
  traj_cloud.header.frame_id = frame;
  traj_cloud.header.stamp = ros::Time::now();

  sensor_msgs::PointCloud2Modifier cloud_mod(traj_cloud);
  cloud_mod.setPointCloud2Fields(5, "x", 1, sensor_msgs::PointField::FLOAT32,
                                    "y", 1, sensor_msgs::PointField::FLOAT32,
                                    "z", 1, sensor_msgs::PointField::FLOAT32,
                                    "theta", 1, sensor_msgs::PointField::FLOAT32,
                                    "cost", 1, sensor_msgs::PointField::FLOAT32);

  unsigned int num_points = 0;
  for(std::vector<base_local_planner::Trajectory>::const_iterator t=trajectories.cbegin(); t != trajectories.cend(); ++t)
  {
      if (t->cost_<0)
        continue;
      num_points += t->getPointsSize();
  }

  cloud_mod.resize(num_points);
  sensor_msgs::PointCloud2Iterator<float> iter_x(traj_cloud, "x");
  for(std::vector<base_local_planner::Trajectory>::const_iterator t=trajectories.cbegin(); t != trajectories.cend(); ++t)
  {
      if(t->cost_<0)
          continue;
      // Fill out the plan
      for(unsigned int i = 0; i < t->getPointsSize(); ++i) {
          double p_x, p_y, p_th;
          t->getPoint(i, p_x, p_y, p_th);
          iter_x[0] = p_x;
          iter_x[1] = p_y;
          iter_x[2] = 0.0;
          iter_x[3] = p_th;
          iter_x[4] = t->cost_;
          ++iter_x;
      }
  }
  traj_cloud_pub_.publish(traj_cloud);
}

}
