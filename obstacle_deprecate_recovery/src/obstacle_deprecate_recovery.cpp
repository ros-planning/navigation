#include <obstacle_deprecate_recovery/obstacle_deprecate_recovery.h>
#include <pluginlib/class_list_macros.h>

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(obstacle_deprecate_recovery::ObstacleDeprecateRecovery, nav_core::RecoveryBehavior);

namespace obstacle_deprecate_recovery{
    ObstacleDeprecateRecovery::ObstacleDeprecateRecovery(): global_costmap_(NULL), local_costmap_(NULL), initialized_(false) {}

    void ObstacleDeprecateRecovery::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
        if(!initialized_){
            global_costmap_ = global_costmap;
            local_costmap_ = local_costmap;

            // get some parameters from the parameter server
            ros::NodeHandle private_nh("~/" + name);
            ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

            private_nh.param("min_inflation", min_inflation_, 0.1);
            private_nh.param("min_occdist_scale", min_occdist_scale_, 0.0);

            initialized_ = true;
        }
        else{
            ROS_ERROR("You should not call initialize twice on this object, doing nothing");
        }
    }
}