#include <move_backwards_recovery/move_backwards_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(move_backwards_recovery, MoveBackRecovery, move_backwards_recovery::MoveBackRecovery, nav_core::RecoveryBehavior)

namespace move_backwards_recovery {
MoveBackRecovery::MoveBackRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
  tf_(NULL), initialized_(false), world_model_(NULL) {} 

void MoveBackRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;
    
    /*
    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    //we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);

    blp_nh.param("acc_lim_th", acc_lim_th_, 3.2);
    blp_nh.param("max_rotational_vel", max_rotational_vel_, 1.0);
    blp_nh.param("min_in_place_rotational_vel", min_rotational_vel_, 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);
    */
    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

MoveBackRecovery::~MoveBackRecovery(){
  delete world_model_;
}

void MoveBackRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the RotateRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Rotate recovery behavior started.");

  ros::Rate r(20);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/internal/sensors/odometry/velocity/cmd", 10);

  tf::Stamped<tf::Pose> global_pose;
  local_costmap_->getRobotPose(global_pose);
  double originX = global_pose.getOrigin().x(); 
  double originY = global_pose.getOrigin().y();   

  double distance_traveled = 0.0;

  bool done = false;

  //double start_offset = 0 - angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
  while(n.ok()){
    local_costmap_->getRobotPose(global_pose);

    distance_traveled = (global_pose.getOrigin().x()- originX)* (global_pose.getOrigin().x()- originX)+ 
                        (global_pose.getOrigin().y()- originY)* (global_pose.getOrigin().y()- originY); 
    //compute the distance left to rotate
    double dist_left = 0.225 - distance_traveled;
    ROS_DEBUG_NAMED("move_back", "dist left %lf", dist_left);
    if(dist_left <= 0)
      return;
    
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = -0.1;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;

    vel_pub.publish(cmd_vel);

    r.sleep();
  }
}
};
