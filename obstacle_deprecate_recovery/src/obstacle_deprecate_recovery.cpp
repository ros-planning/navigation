#include <obstacle_deprecate_recovery/obstacle_deprecate_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(obstacle_deprecate_recovery::ObstacleDeprecateRecovery, nav_core::RecoveryBehavior);

namespace obstacle_deprecate_recovery{
ObstacleDeprecateRecovery::ObstacleDeprecateRecovery(): initialized_(false) 
{
}

void ObstacleDeprecateRecovery::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
    if(!initialized_){
        // get some parameters from the parameter server
        ros::NodeHandle private_nh("~/" + name);
        ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

        max_sim_time_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_sim_time", "max_sim_time_", 4.0);
        min_occdist_scale_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_occdist_scale", "min_occdist_scale_", 0.0);

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

    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");
    //ros::NodeHandle blp_nh("~/");

    if(blp_nh.getParam("sim_time", sim_time_old_)){
        //ROS_INFO("Parameter sim_time is set to %f", sim_time_old_);
        if(sim_time_old_ < max_sim_time_){
            blp_nh.setParam("sim_time", max_sim_time_);
            //ROS_INFO("Updated sim_time from %f to %f", sim_time_old_, max_sim_time_);
            std::cout << "Updated sim_time from " << sim_time_old_ << " to " << max_sim_time_ << "\n";
        }
        else{
            //ROS_INFO("Parameter sim_time is already greater than or equal to its max value");
            std::cout << "Parameter sim_time is already greater than or equal to its max value\n";
        }
    }
    else{
        //ROS_ERROR("Parameter of name sim_time could not be found, doing nothing");
        std::cout << "Parameter of name sim_time could not be found, doing nothing\n";
    }

    if(blp_nh.getParam("occdist_scale", occdist_scale_old_)){
        //ROS_INFO("Parameter occdist_scale is set to %f", occdist_scale_old_);
        if(occdist_scale_old_ > min_occdist_scale_){
            blp_nh.setParam("occdist_scale", min_occdist_scale_);
            //ROS_INFO("Updated occdist_scale from %f to %f", occdist_scale_old_, min_occdist_scale_);
            std::cout << "Updated occdist_scale from " << occdist_scale_old_ << " to " << min_occdist_scale_ << "\n";
        }
        else{
            //ROS_INFO("Parameter occdist_scale is already greater than or equal to its max value");
            std::cout << "Parameter occdist_scale is already greater than or equal to its max value\n";
        }
    }
    else{
        //ROS_ERROR("Parameter of name occdist_scale could not be found, doing nothing");
        std::cout << "Parameter of name occdist_scale could not be found, doing nothing\n";
    }

    return;
}
}; // namespace obstacle_deprecate_recovery