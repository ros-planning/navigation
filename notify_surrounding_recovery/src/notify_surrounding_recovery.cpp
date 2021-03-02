#include <notify_surrounding_recovery/notify_surrounding_recovery.h>
#include <pluginlib/class_list_macros.h>

// register this planner as a RecoveryBehavior plugin
PLUGIN_EXPORT_CLASS(notify_surrounding_recovery::NotifySurroundingRecovery, nav_core::RecoveryBehavior);

namespace notify_surrounding_recovery
{
    
};  // namespace notify_surrounding_recovery
