
#include <rotate_recovery/negative_rotate_recovery.h>
#include <pluginlib/class_list_macros.h>

 // register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(rotate_recovery::NegativeRotateRecovery, nav_core::RecoveryBehavior)

 namespace rotate_recovery {

 NegativeRotateRecovery::NegativeRotateRecovery() {rotate_positive_= false;}

 };  // namespace rotate_recovery