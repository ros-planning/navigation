#include <go_back_recovery/go_back_recovery.h>
#include <nav_core/parameter_magic.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include "move_base/move_base.h"

PLUGINLIB_EXPORT_CLASS(go_back_recovery::GoBackRecovery, nav_core::RecoveryBehavior)

namespace go_back_recovery{
GoBackRecovery::GoBackRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void GoBackRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
    if(!initialized_)
    {
        local_costmap_ = local_costmap;

        // get some parameters from the parameter server
        ros::NodeHandle private_nh("~/" + name);
        ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

        private_nh.param("sim_granularity", sim_granularity_, 0.01);
        private_nh.param("frequency", frequency_, 20.0);

        blp_nh.param("max_vel_x", max_vel_x_, 2.0); 
        blp_nh.param("min_vel_x", min_vel_x_, 0.05);
        blp_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
        blp_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
        blp_nh.param("x_goal_tolerance", tolerance_, 0.10);
        
        world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());
        
        initialized_ = true;
    }
    else
    {
        ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
}

GoBackRecovery::~GoBackRecovery()
{
    delete world_model_;
}

double GoBackRecovery::calculateDist(geometry_msgs::PoseStamped initial, geometry_msgs::PoseStamped current)
{
    return sqrt(pow((initial.pose.position.x - current.pose.position.x), 2.0) + pow((initial.pose.position.y - current.pose.position.y), 2.0));
}

void GoBackRecovery::runBehavior()
{
    if(!initialized_)
    {
        ROS_ERROR("This object must be initialized before runBehavior is called");
    }

    if(local_costmap_ == NULL)
    {
        ROS_ERROR("The costmap passed to the GoBackRecovery object cannnot be NULL. Doing nothing.");
        return;
    }
    ROS_WARN("Robot will start moving backwards.");

    ros::Rate r(frequency_);
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    geometry_msgs::PoseStamped global_pose;
    geometry_msgs::PoseStamped initial_pose;
    local_costmap_->getRobotPose(initial_pose);
    double current_angle = tf2::getYaw(initial_pose.pose.orientation);
    const double dist_to_move = 1.0;
    double dist_travelled = 0.0;
    double dist_left = dist_to_move - dist_travelled;

    ///By default, make the robot move 1m backwards
    while(n.ok() && dist_left > tolerance_)
    {
        // update current position of the robot
        local_costmap_->getRobotPose(global_pose);
        double x = global_pose.pose.position.x, y = global_pose.pose.position.y;
        current_angle = tf2::getYaw(global_pose.pose.orientation);
        dist_travelled = GoBackRecovery::calculateDist(initial_pose, global_pose);

        // conduct forward simulation
        dist_left = dist_to_move - dist_travelled;
        double sim_distance = 0.0;
        while(sim_distance < dist_left)
        {
            double sim_x = x - sim_distance * cos(current_angle);
            double sim_y = y - sim_distance * sin(current_angle);

            // make sure that the point is legal. Else, abort
            double footprint_cost = world_model_->footprintCost(sim_x, sim_y, current_angle, local_costmap_->getRobotFootprint(), inscribed_radius_, circumscribed_radius_);
            if(footprint_cost < 0.0)
            {
                ROS_ERROR("Go back recovery can't be conducted because there is a potential collision. Cost: %.2f", footprint_cost);
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;

                vel_pub.publish(cmd_vel);
                return;
            }

            sim_distance += sim_granularity_;
        }

        double vel = 0.05;
        vel = std::min(std::max(vel, min_vel_x_), max_vel_x_);

        cmd_vel.linear.x = -abs(vel);
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;

        vel_pub.publish(cmd_vel);

        r.sleep();
    }

    ROS_INFO("Go back recovery ended because the robot travelled %fm\n", dist_travelled);
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;

    vel_pub.publish(cmd_vel);
    return;
}
};  // namespace go_back_recovery