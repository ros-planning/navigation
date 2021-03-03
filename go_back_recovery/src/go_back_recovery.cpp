#include <go_back_recovery/go_back_recovery.h>
#include <nav_core/parameter_magic.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>

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
        ros::NodeHandle nh("~/" + name);
        ros::NodeHandle blp_nh("~/TrahectoryPlannerROS");

        nh.param("sim_granularity", sim_granularity_, 0.1);
        frequency_ = nav_core::loadParameterWithDeprecation(blp_nh, "controller_frequency", "frequency", 20.0);
        min_vel_x_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_vel_x", "min_trans_x", 0.05);
        max_vel_x_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_x", "max_trans_x", 2.0);

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
    local_costmap_->getRobotPose(global_pose);
    double current_angle = tf2::getYaw(global_pose.pose.orientation);

    while(n.ok())
    {
        // update current position of the robot
        local_costmap_->getRobotPose(global_pose);
        double x = global_pose.pose.position.x, y = global_pose.pose.position.y;

        // conduct forward simulation
        double dist_left = 1.0;
        double sim_distance = 0.0;
        while(sim_distance < dist_left)
        {
            double sim_x = x + sim_distance * cos(current_angle);
            double sim_y = y + sim_distance * sin(current_angle);

            // make sure that the point is legal. Else, abort
            double footprint_cost = world_model_->footprintCost(sim_x, sim_y, current_angle, local_costmap_->getRobotFootprint(), 0.0, 0.0);
            if(footprint_cost < 0.0)
            {
                ROS_ERROR("Go back recovery can't be conducted because there is a potential collision. Cost: %.2f", footprint_cost);
                return;
            }

            sim_distance += sim_granularity_;
        }

        double vel = 0.1;
        vel = std::min(std::max(vel, min_vel_x_), max_vel_x_);

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = vel;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;

        vel_pub.publish(cmd_vel);

        r.sleep();
    }
}
};  // namespace go_back_recovery