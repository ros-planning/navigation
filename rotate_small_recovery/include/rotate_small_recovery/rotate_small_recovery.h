#ifndef ROTATE_SMALL_RECOVERY_H
#define ROTATE_SMALL_RECOVERY_H
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_core/recovery_behavior.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

namespace rotate_small_recovery
{
class RotateSmallRecovery : public nav_core::RecoveryBehavior
{
    public:
        RotateSmallRecovery();

        void initialize(std::string name, tf2_ros::Buffer*,
                        costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

        void runBehavior();

        ~RotateSmallRecovery();
    private:
        costmap_2d::Costmap2DROS* local_costmap_;
        bool initialized_;
        double sim_granularity_, min_rotational_vel_, max_rotational_vel_, acc_lim_th_, tolerance_, frequency_;
        double angle_to_rot_;
        base_local_planner::CostmapModel* world_model_;
};
}; // namespace rotate_small_recovery

#endif
