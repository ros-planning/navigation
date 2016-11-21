#include <move_backwards_recovery/move_backwards_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <math.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(move_backwards_recovery, MoveBackRecovery, move_backwards_recovery::MoveBackRecovery, nav_core::RecoveryBehavior)

namespace move_backwards_recovery {
MoveBackRecovery::MoveBackRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
  tf_(NULL), initialized_(false), world_model_(NULL){} 

void MoveBackRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;
    
    //get parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);

    private_nh.param("distance_backward", distance_backwards_, 0.15);
    private_nh.param("backwards_velocity", backwards_velocity_, -0.05);

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
    ROS_ERROR("The costmaps passed to the MoveBackRecovery object cannot be NULL. Doing nothing.");
    return;
  }

  if(backwards_velocity_ == 0 || distance_backwards_ ==0){
    ROS_ERROR("Bad input. Either velocity or distance is not setup correctly");
    return;
  }

  if(fabs(backwards_velocity_) > 0.1){
    ROS_ERROR("Velocity is too high to implement move_backwards_recovery");
    return;
  }

  // make sure the distance value >0 and backwards velocity < 0
  distance_backwards_ = (distance_backwards_> 0) ? distance_backwards_ : ((-1) * distance_backwards_) ;
  backwards_velocity_ = (backwards_velocity_ > 0) ? ((-1)*backwards_velocity_) : backwards_velocity_ ;

  ROS_DEBUG_NAMED("move_backwards_recovery", "MoveBackRecovery started.");

  ros::Rate r(20);

  // setup Nodehandle and publisher
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>(RECOVERY_VELOCITY_CMD, 10);

  tf::Stamped<tf::Pose> global_pose;
  local_costmap_->getRobotPose(global_pose);

  // the original location
  double originX = global_pose.getOrigin().x(); 
  double originY = global_pose.getOrigin().y();   

  double distance_traveled = 0.0;

  while(n.ok()){
    local_costmap_->getRobotPose(global_pose);

    double cur_x = global_pose.getOrigin().x();
    double cur_y = global_pose.getOrigin().y();
    double cur_theta = tf::getYaw(global_pose.getRotation());

    distance_traveled = sqrt((cur_x- originX)* (cur_x- originX) + (cur_y- originY)* (cur_y- originY));

    double dist_left = distance_backwards_ - distance_traveled;
    ROS_DEBUG_NAMED("move_backwards_recovery", "Distance left: %lf", dist_left);

    // if robot has already traveled enough, return
    if(dist_left <= 0)
      return;
    
    double sim_dist = 0.0;
    while(dist_left >= sim_dist){
      double sim_x = cur_x - sim_dist * cos(cur_theta);
      double sim_y = cur_y - sim_dist * sin(cur_theta);

      double footprint_cost = world_model_->footprintCost(sim_x, sim_y, cur_theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
      if(footprint_cost < 0.0){
    	ROS_ERROR("Backwards recovery can't move because there is a potential collision. Cost: %.2f", footprint_cost);
    	return;
      }

      sim_dist += 0.01;
    }

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = backwards_velocity_;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;

    vel_pub.publish(cmd_vel);

    r.sleep();
  }
}
};
