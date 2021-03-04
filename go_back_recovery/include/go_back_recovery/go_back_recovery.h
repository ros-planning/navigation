#ifndef GO_BACK_RECOVERY_H
#define GO_BACK_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <ros/ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

namespace go_back_recovery
{
class GoBackRecovery : public nav_core::RecoveryBehavior
{
    public:
        GoBackRecovery();

        void initialize(std::string name, tf2_ros::Buffer*,
                        costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

        void runBehavior();

        ~GoBackRecovery();
    private:
        costmap_2d::Costmap2DROS* local_costmap_;
        bool initialized_;
        double frequency_, sim_granularity_, min_vel_x_, max_vel_x_, inscribed_radius_, circumscribed_radius_;
        base_local_planner::CostmapModel* world_model_;
        geometry_msgs::Twist cmd_vel;
        double calculateDist(geometry_msgs::PoseStamped initial, geometry_msgs::PoseStamped current);
};
};  // namespace go_back_recovery

#endif