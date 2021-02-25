#include <obstacle_deprecate_recovery/obstacle_deprecate_recovery.h>
#include <pluginlib/class_list_macros.h>

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(obstacle_deprecate_recovery::ObstacleDeprecateRecovery, nav_core::RecoveryBehavior);

namespace obstacle_deprecate_recovery{
    ObstacleDeprecateRecovery::ObstacleDeprecateRecovery(): global_costmap_(NULL), local_costmap_(NULL), initialized_(false) 
    {
    }

    void ObstacleDeprecateRecovery::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
        if(!initialized_){
            global_costmap_ = global_costmap;
            local_costmap_ = local_costmap;

            // get some parameters from the parameter server
            ros::NodeHandle private_nh("~/" + name);
            ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

            private_nh.param("min_inflation", min_inflation_, 0.1);
            private_nh.param("min_occdist_scale", min_occdist_scale_, 0.0);
            private_nh.param("max_sim_time", max_sim_time_, 4.0);

            initialized_ = true;
        }
        else{
            ROS_ERROR("You should not call initialize twice on this object, doing nothing");
        }
    }

    void ObstacleDeprecateRecovery::runBehavior(){
        if(!initialized_){
            ROS_ERROR("This object must be initialized before runBehavior is called");
            return;
        }

        if(blp_nh.getParam("sim_time", sim_time_old_)){
            ROS_INFO("Parameter sim_time is set to %d", sim_time_old_);
            if(sim_time_old_ < max_sim_time_){
                blp_nh.setParam("sim_time", max_sim_time_);
                ROS_INFO("Updated sim_time from %d to %d", sim_time_old_, max_sim_time_);
            }
            else{
                ROS_INFO("Parameter sim_time is already greater than or equal to its max value");
            }
        }
        else{
            ROS_ERROR("Parameter of name sim_time could not be found, doing nothing");
            return;
        }

        if(blp_nh.getParam("occdist_scale", occdist_scale_old_)){
            ROS_INFO("Parameter occdist_scale is set to %d", occdist_scale_old_);
            if(occdist_scale_old_ < min_occdist_scale_){
                blp_nh.setParam("occdist_scale", min_occdist_scale_);
                ROS_INFO("Updated occdist_scale from %d to %d", occdist_scale_old_, min_occdist_scale_);
            }
            else{
                ROS_INFO("Parameter occdist_scale is already greater than or equal to its max value");
            }
        }
        else{
            ROS_ERROR("Parameter of name occdist_scale could not be found, doing nothing");
            return;
        }

    }
}