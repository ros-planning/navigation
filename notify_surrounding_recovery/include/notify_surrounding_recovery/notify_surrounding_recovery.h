#ifndef NOTIFY_SURROUNDING_RECOVERY_H_
#define NOTIFY_SURROUNDING_RECOVERY_H_
#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <lexxauto_msgs/Led.h>

namespace notify_surrounding_recovery
{
    class NotifySurroundingRecovery : public nav_core::RecoveryBehavior
    {
        public:
            NotifySurroundingRecovery();

            void initialize(std::string name, tf2_ros::Buffer*,
                            costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS*);

            void runBehavior();

            //~NotifySurroundingRecovery();

        private:
            bool initialized_;
            std::string led_pattern_;
            ros::NodeHandle n;
            ros::ServiceClient led_recovery_client;
            lexxauto_msgs::Led srv;
    };
};  // namespace notify_surrounding_recovery
#endif
