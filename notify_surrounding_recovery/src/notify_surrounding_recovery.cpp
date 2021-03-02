#include <notify_surrounding_recovery/notify_surrounding_recovery.h>
#include <pluginlib/class_list_macros.h>

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(notify_surrounding_recovery::NotifySurroundingRecovery, nav_core::RecoveryBehavior)

namespace notify_surrounding_recovery
{
    NotifySurroundingRecovery::NotifySurroundingRecovery(): initialized_(false)
    {
    }

    void NotifySurroundingRecovery::initialize(std::string name, tf2_ros::Buffer*, costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS*)
    {
        if(!initialized_)
        {
            ros::NodeHandle private_nh("~/" + name);

            private_nh.param<std::string>("led_pattern", led_pattern_, "emergency_stop");

            initialized_ = true;
        }
        else
        {
            ROS_ERROR("You should not call initialized twice on this object, doing nothing");
        }
    }

    void NotifySurroundingRecovery::runBehavior()
    {
        if(!initialized_)
        {
            ROS_ERROR("This object must be initialized before runBehavior is called");
            return;
        }

        led_recovery_client = n.serviceClient<lexxauto_msgs::Led>("/led");
        srv.request.pattern = led_pattern_;
        
        if(led_recovery_client.call(srv))
        {
        }
        else
        {
            srv.request.pattern = "unknown";
        }
    }
};  // namespace notify_surrounding_recovery
