#include <move_backwards_recovery/move_backwards_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <math.h>
#include <angles/angles.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(move_backwards_recovery, MoveBackRecovery, move_backwards_recovery::MoveBackRecovery, nav_core::RecoveryBehavior)

namespace move_backwards_recovery {
MoveBackRecovery::MoveBackRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  tf_(NULL), initialized_(false), world_model_(NULL),
  lastEndingX_(0), lastEndingY_(0), lastEndingTheta_(0),
  linearStartTolerance_(-1), angularStartTolerance_(-1), runOnce_(false),
  maxRecoveriesPerGoal_(-1), numRecoveriesSinceLastGoal_(0),
  maxRecoveriesResetDistance_(1.0), inPlaceRecoveryCount_(0),
  maxInPlaceRecoveries_(2) {}

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
    private_nh.param("linear_start_tolerance", linearStartTolerance_, 0.05);
    private_nh.param("angular_start_tolerance", angularStartTolerance_, 0.1);
    private_nh.param("max_recoveries_per_goal", maxRecoveriesPerGoal_, 3);
    private_nh.param("max_recoveries_reset_distance", maxRecoveriesResetDistance_, 1.5);
    private_nh.param("max_in_place_recoveries", maxInPlaceRecoveries_, 2);

    footprint_ = costmap_2d::makeFootprintFromParams(private_nh);
    if (footprint_.size() < 3) {
      ROS_INFO_NAMED("move_backwards_recovery", "Using local costmap footprint for move backwards");
    } else {
      ROS_INFO_NAMED("move_backwards_recovery", "Using move backwards footprint for move backwards");
    }

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());
    initialized_ = true;
    canceled_ = false;
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

  canceled_ = false;
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
  double originTheta = tf::getYaw(global_pose.getRotation());

  // check to see if the robot has moved since the last backup recovery.
  // if not, return.
  if (runOnce_
    && std::fabs(originX - lastEndingX_) < linearStartTolerance_
    && std::fabs(originY - lastEndingY_) < linearStartTolerance_
    && std::fabs(angles::normalize_angle(originTheta - lastEndingTheta_)) < angularStartTolerance_)
  {
    if (inPlaceRecoveryCount_ >= maxInPlaceRecoveries_)
    {
      ROS_INFO_NAMED("move_backwards_recovery",
        "Not moving backwards because the robot has not moved since the recovery last triggered.");
      return;
    }
    inPlaceRecoveryCount_++;
  }
  else
  {
    inPlaceRecoveryCount_ = 0;
  }
  runOnce_ = true;

  double distance_traveled_sq = 0.0;
  double distance_max_sq = distance_backwards_ * distance_backwards_;

  std::vector<geometry_msgs::Point> footprint = footprint_;
  if (footprint.size() < 3) {
    footprint = local_costmap_->getRobotFootprint();
  }

  while(n.ok()){

    if (canceled_) {
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      vel_pub.publish(cmd_vel);
      ROS_DEBUG_NAMED("move_backwards_recovery","Cancelled called on move backwards recovery behavior publishing zero velocity");
      return;
    }

    bool should_stop = false;

    local_costmap_->getRobotPose(global_pose);

    double cur_x = global_pose.getOrigin().x();
    double cur_y = global_pose.getOrigin().y();
    double cur_theta = tf::getYaw(global_pose.getRotation());

    // Store thet values for reset check later.
    lastEndingX_ = cur_x;
    lastEndingY_ = cur_y;
    lastEndingTheta_ = cur_theta;


    distance_traveled_sq = (cur_x - originX) * (cur_x - originX) + (cur_y - originY) * (cur_y - originY);

    ROS_DEBUG_NAMED("move_backwards_recovery", "Square of distance traveled: %lf", distance_traveled_sq);

    double distance_left_sq = distance_max_sq - distance_traveled_sq;

    // if robot has already traveled enough, return
    if(distance_left_sq <= 0){
      should_stop = true;
    }

    // forward simulation to avoid collision
    double sim_dist = 0.0;
    while(distance_left_sq >= (sim_dist * sim_dist)){
      double sim_x = cur_x - sim_dist * cos(cur_theta);
      double sim_y = cur_y - sim_dist * sin(cur_theta);

      double footprint_cost = world_model_->footprintCost(sim_x, sim_y, cur_theta, footprint, 0.0, 0.0);
      if(footprint_cost < 0.0){
    	  ROS_ERROR("Backwards recovery can't move because there is a potential collision. Cost: %.2f", footprint_cost);
        should_stop = true;
        break;
      }

      sim_dist += 0.01;
    }

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = should_stop ? 0 : backwards_velocity_;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;

    vel_pub.publish(cmd_vel);

    if (should_stop) {
      return;
    }
    
    r.sleep();
  }
}

bool MoveBackRecovery::cancel() {
  canceled_ = true;
  return true;
}

void MoveBackRecovery::newGoalReceived() {
  numRecoveriesSinceLastGoal_ = 0;
}
};
