#ifndef OBSTACLE_DEPRECATE_RECOVERY_H_
#define OBSTACLE_DEPRECATE_RECOVERY_H_
#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <base_local_planner/costmap_model.h>

namespace obstacle_deprecate_recovery{
    class ObstacleDeprecateRecovery : public nav_core::RecoveryBehavior{
        public:
            ObstacleDeprecateRecovery();

            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

            void runBehavior();

            //~ObstacleDeprecateRecovery();

        private:
            bool initialized_;
            double min_occdist_scale_, max_sim_time_;
            double occdist_scale_old_, sim_time_old_;
    };
};
#endif