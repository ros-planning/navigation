#ifndef SAFETY_DIRECTION_RECOVERY_H
#define SAFETY_DIRECTION_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <ros/ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <lexxauto_msgs/safety_status.h>

namespace safety_direction_recovery
{
  class SafetyDirectionRecovery : public nav_core::RecoveryBehavior
  {
    public:
      SafetyDirectionRecovery();

      void initialize(std::string name, tf2_ros::Buffer*,
          costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

      void runBehavior();

      ~SafetyDirectionRecovery();
    private:
      costmap_2d::Costmap2DROS* local_costmap_;
      base_local_planner::CostmapModel* world_model_;

      bool initialized_;
      double frequency_, sim_granularity_, min_vel_x_, max_vel_x_, inscribed_radius_, circumscribed_radius_;
      double x_goal_tolerance_, yaw_goal_tolerance_;
      double min_rotational_vel_, max_rotational_vel_, acc_lim_th_, angle_to_rot_;

      geometry_msgs::Twist cmd_vel;

      double calculateDist(geometry_msgs::PoseStamped initial, geometry_msgs::PoseStamped current);
      double go_straight(const double direction);
      double rotate(const double direction);
      //void safetyStatusCallback(const lexxauto_msgs::safety_status::ConstPtr& msg);
  };
};  // namespace safety_direction_recovery

#endif
