#ifndef MOVE_BACKWARDS_RECOVERY_H_
#define MOVE_BACKWARDS_RECOVERY_H_

#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <costmap_2d/footprint.h>

namespace move_backwards_recovery{

  class MoveBackRecovery : public nav_core::RecoveryBehavior {
    public:

      MoveBackRecovery();

      void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

      void runBehavior();

      ~MoveBackRecovery();

      void newGoalReceived();

      void cancel();

    private:
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
      costmap_2d::Costmap2D costmap_;
      std::string name_;
      tf::TransformListener* tf_;
      bool initialized_;
      double distance_backwards_;
      double backwards_velocity_;
      std::vector<geometry_msgs::Point> footprint_;

      double lastEndingX_;
      double lastEndingY_;
      double lastEndingTheta_;
      double linearStartTolerance_;
      double angularStartTolerance_;
      bool runOnce_;

      int numRecoveriesSinceLastGoal_;
      int maxRecoveriesPerGoal_;
      double maxRecoveriesResetDistance_;
      int inPlaceRecoveryCount_;
      int maxInPlaceRecoveries_;
      bool canceled_;

      base_local_planner::CostmapModel* world_model_;
      static constexpr auto RECOVERY_VELOCITY_CMD = "/cmd_vel";

  };
};
#endif
