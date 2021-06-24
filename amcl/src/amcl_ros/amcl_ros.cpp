#include "amcl_ros.h"

using namespace amcl;
static const std::string scan_topic_ = "scan";

AmclRosNode::AmclRosNode():
    sent_first_transform_(false),
    latest_tf_valid_(false),
    private_nh_("~"),
    first_reconfigure_call_(true),
    tdr_("AMCL"),
    first_map_received_(false),
    set_inital_pose_action_(nh_, "initial_pose_server", boost::bind(&AmclRosNode::executeInitialPoseCB, this, _1), false)
{
    std::cout << "construtor amcl_ros" <<std::endl;
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);
    initializeParams();
    updatePoseFromServer();
    cloud_pub_interval.fromSec(1.0);
    tfb_ = new tf::TransformBroadcaster();
    tf_  = new TransformListenerWrapper();

    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2, true);
    analytics_pub_ = nh_.advertise<move_base_msgs::amcl_analytics>("amcl_analytics", 2, true);
    data_pub_ = nh_.advertise<move_base_msgs::amcl_data>("amcl_data",2,true);
    invalid_pose_percent_pub_ = nh_.advertise<std_msgs::Float32>("amcl_invalid_poses", 2);
    ready_pub_ = nh_.advertise<std_msgs::Bool>("/amcl/ready", 2, true);
    status_on_new_work_area_pub_ = nh_.advertise<srslib_framework::AmclStatusOnNewWorkArea>("/amcl/status_on_new_work_area", 2, true);
    particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
    global_loc_srv_ = nh_.advertiseService("global_localization",
                        &AmclRosNode::globalLocalizationCallback,
                                            this);
    nomotion_update_srv_= nh_.advertiseService("request_nomotion_update", &AmclRosNode::nomotionUpdateCallback, this);
    set_map_srv_= nh_.advertiseService("set_map", &AmclRosNode::setMapCallback, this);

    laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100);
    laser_scan_filter_ =
            new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                            *tf_,
                                                            amclParams_.odom_frame_id_,
                                                            100);
    laser_scan_filter_->registerCallback(boost::bind(&AmclRosNode::laserReceived,
                                                    this, _1));
    initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclRosNode::initialPoseReceived, this);
    initial_poses_sub_ = nh_.subscribe("initialposes", 2, &AmclRosNode::initialPosesReceived, this);

    if(amclParams_.use_map_topic_) {
        map_sub_ = nh_.subscribe("map", 1, &AmclRosNode::mapReceived, this);
        ROS_INFO("Subscribed to map topic.");
    } else {
        requestMap();
    }
    force_update_ = false;

    dsrv_ = new dynamic_reconfigure::Server<amcl::AMCLConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<amcl::AMCLConfig>::CallbackType cb = boost::bind(&AmclRosNode::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    // 15s timer to warn on lack of receipt of laser scans, #5209
    laser_check_interval_ = ros::Duration(15.0);
    check_laser_timer_ = nh_.createTimer(laser_check_interval_,
                                        boost::bind(&AmclRosNode::checkLaserReceived, this, _1));

    

    //start action server
    set_inital_pose_action_.start();

    brainstem_driver_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(srs::ChuckTopics::internal::ODOMETRY_INITIAL_POSE, 2, true);

    publishAmclReadySignal(true);
}

AmclRosNode::~AmclRosNode()
{
    std::cout << "destructor" << std::endl;
    delete dsrv_;
    delete laser_scan_filter_;
    delete laser_scan_sub_;
    delete tfb_;
    delete tf_;
}

void AmclRosNode::initializeParams()
{
    std::cout << "initialize params" << std::endl;
    // Grab params off the param server
    private_nh_.param("use_map_topic", amclParams_.use_map_topic_, false);
    private_nh_.param("first_map_only", amclParams_.first_map_only_, false);

    double tmp;
    private_nh_.param("gui_publish_rate", tmp, -1.0);
    amclParams_.gui_publish_period = ros::Duration(1.0/tmp);
    private_nh_.param("save_pose_rate", tmp, 0.5);
    amclParams_.save_pose_period = ros::Duration(1.0/tmp);
    private_nh_.param("laser_min_range", amclParams_.laser_min_range_, -1.0);
    private_nh_.param("laser_max_range", amclParams_.laser_max_range_, -1.0);
    private_nh_.param("laser_max_beams", amclParams_.max_beams_, 30);
    private_nh_.param("min_particles", amclParams_.min_particles_, 100);
    private_nh_.param("max_particles", amclParams_.max_particles_, 5000);
    private_nh_.param("kld_err", amclParams_.pf_err_, 0.01);
    private_nh_.param("kld_z", amclParams_.pf_z_, 0.99);
    private_nh_.param("odom_alpha1", amclParams_.alpha1_, 0.2);
    private_nh_.param("odom_alpha2", amclParams_.alpha2_, 0.2);
    private_nh_.param("odom_alpha3", amclParams_.alpha3_, 0.2);
    private_nh_.param("odom_alpha4", amclParams_.alpha4_, 0.2);
    private_nh_.param("odom_alpha5", amclParams_.alpha5_, 0.2);
    private_nh_.param("do_beamskip", amclParams_.do_beamskip_, false);
    private_nh_.param("beam_skip_distance", amclParams_.beam_skip_distance_, 0.5);
    private_nh_.param("beam_skip_threshold", amclParams_.beam_skip_threshold_, 0.3);
    private_nh_.param("beam_skip_error_threshold", amclParams_.beam_skip_error_threshold_, 0.9);
    private_nh_.param("use_emulator", amclParams_.use_emulator_, false);

    private_nh_.param("laser_z_hit", amclParams_.z_hit_, 0.95);
    private_nh_.param("laser_z_short", amclParams_.z_short_, 0.1);
    private_nh_.param("laser_z_max", amclParams_.z_max_, 0.05);
    private_nh_.param("laser_z_rand", amclParams_.z_rand_, 0.05);
    private_nh_.param("laser_sigma_hit", amclParams_.sigma_hit_, 0.2);
    private_nh_.param("laser_lambda_short", amclParams_.lambda_short_, 0.1);
    private_nh_.param("laser_likelihood_max_dist", amclParams_.laser_likelihood_max_dist_, 2.0);
    std::string tmp_model_type;
    private_nh_.param("laser_model_type", tmp_model_type, std::string("likelihood_field"));
    
    if(tmp_model_type == "beam")
        amclParams_.laser_model_type_ = LASER_MODEL_BEAM;
    else if(tmp_model_type == "likelihood_field")
        amclParams_.laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
    else if(tmp_model_type == "likelihood_field_prob"){
        amclParams_.laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
    }
    else
    {
        ROS_WARN("Unknown laser model type \"%s\"; defaulting to likelihood_field model",
                tmp_model_type.c_str());
        amclParams_.laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
    }
    private_nh_.param("odom_model_type", tmp_model_type, std::string("diff"));
    if(tmp_model_type == "diff")
        amclParams_.odom_model_type_ = ODOM_MODEL_DIFF;
    else if(tmp_model_type == "omni")
        amclParams_.odom_model_type_ = ODOM_MODEL_OMNI;
    else if(tmp_model_type == "diff-corrected")
        amclParams_.odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
    else if(tmp_model_type == "omni-corrected")
        amclParams_.odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
    else
    {
        ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model",
                tmp_model_type.c_str());
        amclParams_.odom_model_type_ = ODOM_MODEL_DIFF;
    }

    private_nh_.param("update_min_d", amclParams_.d_thresh_, 0.2);
    private_nh_.param("update_min_a", amclParams_.a_thresh_, M_PI/6.0);
    private_nh_.param("odom_frame_id", amclParams_.odom_frame_id_, std::string("odom"));
    private_nh_.param("base_frame_id", amclParams_.base_frame_id_, std::string("base_link"));
    private_nh_.param("global_frame_id", amclParams_.global_frame_id_, std::string("map"));
    private_nh_.param("resample_interval", amclParams_.resample_interval_, 2);
    double tmp_tol;
    private_nh_.param("transform_tolerance", tmp_tol, 0.1);
    private_nh_.param("recovery_alpha_slow", amclParams_.alpha_slow_, 0.001);
    private_nh_.param("recovery_alpha_fast", amclParams_.alpha_fast_, 0.1);
    private_nh_.param("tf_broadcast", amclParams_.tf_broadcast_, true);
    amclParams_.transform_tolerance_.fromSec(tmp_tol);
    double bag_scan_period;
    private_nh_.param("bag_scan_period", bag_scan_period, -1.0);
    amclParams_.bag_scan_period_.fromSec(bag_scan_period);
}

void AmclRosNode::updatePoseFromServer()
{
    // Check for NAN on input from param server, #5239
    double tmp_pos;
    double init_pose[3], init_cov[3];
    private_nh_.param("initial_pose_x", tmp_pos, 0.0);
    if(!std::isnan(tmp_pos))
        init_pose[0] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial pose X position");
    private_nh_.param("initial_pose_y", tmp_pos, 0.0);
    if(!std::isnan(tmp_pos))
        init_pose[1] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial pose Y position");
    private_nh_.param("initial_pose_a", tmp_pos, 0.0);
    if(!std::isnan(tmp_pos))
        init_pose[2] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial pose Yaw");
    private_nh_.param("initial_cov_xx", tmp_pos, 0.5 * 0.5);
    if(!std::isnan(tmp_pos))
        init_cov[0] =tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial covariance XX");
    private_nh_.param("initial_cov_yy", tmp_pos, 0.5 * 0.5);
    if(!std::isnan(tmp_pos))
        init_cov[1] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial covariance YY");
    private_nh_.param("initial_cov_aa", tmp_pos, (M_PI/12.0) * (M_PI/12.0));
    if(!std::isnan(tmp_pos))
        init_cov[2] = tmp_pos;
    else
        ROS_WARN("ignoring NAN in initial covariance AA");
    
    setInitialPose(init_pose, init_cov);
}

void AmclRosNode::publishAmclReadySignal(bool signal)
{
  // publish ready signal
  std_msgs::Bool ready_msg;
  ready_msg.data = signal;
  ready_pub_.publish(ready_msg);
}

void AmclRosNode::executeInitialPoseCB(const move_base_msgs::SetInitialPoseGoalConstPtr &goal)
{
  ROS_INFO("Executing, inital pose server action");
  if(amclParams_.use_emulator_)
  {
    brainstem_driver_pose_pub_.publish(goal->initialPose);
    ROS_INFO("Sleeping for brainstem to publish new tf/odometry");
    ros::Duration(1.0).sleep();
  }
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg = goal->initialPose;
  msg.header.stamp = ros::Time::now();
  set_initial_pose_action_result_.initialPoseSet = handleInitialPoseMessage(msg);

  if(set_initial_pose_action_result_.initialPoseSet == true) {
    ROS_INFO("Succeeded. Result: %d", set_initial_pose_action_result_.initialPoseSet);
    set_inital_pose_action_.setSucceeded(set_initial_pose_action_result_);
  } else {
    ROS_INFO("Aborted. Result: %d", set_initial_pose_action_result_.initialPoseSet);
    set_inital_pose_action_.setAborted(set_initial_pose_action_result_);
  }
}

void AmclRosNode::savePoseToServer()
{
  // We need to apply the last transform to the latest odom pose to get
  // the latest map pose to store.  We'll take the covariance from
  // last_published_pose.
  tf::Pose map_pose = latest_tf_.inverse() * latest_odom_pose_;
  double yaw,pitch,roll;
  map_pose.getBasis().getEulerYPR(yaw, pitch, roll);

  ROS_DEBUG("Saving pose to server. x: %.3f, y: %.3f", map_pose.getOrigin().x(), map_pose.getOrigin().y() );

  private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
  private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
  private_nh_.setParam("initial_pose_a", yaw);
  // TODO: FIGURE OUT WHERE TO GET LAST_PUBLICHED_POSE FROM
//   private_nh_.setParam("initial_cov_xx",
//                                   last_published_pose.pose.covariance[6*0+0]);
//   private_nh_.setParam("initial_cov_yy",
//                                   last_published_pose.pose.covariance[6*1+1]);
//   private_nh_.setParam("initial_cov_aa",
//                                   last_published_pose.pose.covariance[6*5+5]);
}

void AmclRosNode::deletePoseFromServer()
{
  private_nh_.deleteParam("initial_pose_x");
  private_nh_.deleteParam("initial_pose_y");
  private_nh_.deleteParam("initial_pose_a");
  private_nh_.deleteParam("initial_cov_xx");
  private_nh_.deleteParam("initial_cov_yy");
  private_nh_.deleteParam("initial_cov_aa");
  ROS_INFO("Delete pose from server");
}

bool AmclRosNode::globalLocalizationCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
    // TODO: Update getting variables
//   if( map_ == NULL ) {
//     return true;
//   }
//   boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
//   ROS_INFO("Initializing with uniform distribution");
//   pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
//                 (void *)map_);
//   ROS_INFO("Global initialisation done!");
//   pf_init_ = false;
//   return true;
}

// force nomotion updates (amcl updating without requiring motion)
bool AmclRosNode::nomotionUpdateCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
    force_update_ = true;
	//ROS_INFO("Requesting no-motion update");
	return true;
}

bool AmclRosNode::setMapCallback(nav_msgs::SetMap::Request& req,
                         nav_msgs::SetMap::Response& res)
{
  handleMapMessage(req.map);
  handleInitialPoseMessage(req.initial_pose);
  res.success = true;
  return true;
}

void AmclRosNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  handleInitialPoseMessage(*msg);
}

void AmclRosNode::initialPosesReceived(const move_base_msgs::PoseWithCovarianceStampedArrayConstPtr& msg)
{
  handleInitialPosesMessage(*msg);

  // Force one no motion update
  force_update_ = true;
}

bool AmclRosNode::handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	move_base_msgs::PoseWithCovarianceStampedArray arrayMsg;
	arrayMsg.header = msg.header;
	arrayMsg.poses.push_back(msg.pose);
	return handleInitialPosesMessage(arrayMsg);
}

void AmclRosNode::publishStatusOnNewWorkArea(bool mapUpdated, bool initialPoseUpdated)
{
  // publish status signal for core
  srslib_framework::AmclStatusOnNewWorkArea amcl_status_msg;
  amcl_status_msg.mapUpdated = mapUpdated;
  amcl_status_msg.initialPoseUpdated = initialPoseUpdated;
  status_on_new_work_area_pub_.publish(amcl_status_msg);
}

bool AmclRosNode::checkPoseParamOnServer()
{
  if(private_nh_.hasParam("initial_pose_x") &&
      private_nh_.hasParam("initial_pose_y") &&
      private_nh_.hasParam("initial_pose_a") &&
      private_nh_.hasParam("initial_cov_xx") &&
      private_nh_.hasParam("initial_cov_yy") &&
      private_nh_.hasParam("initial_cov_aa"))
  {
    ROS_INFO("Using pose from server since we have all params.");
    return true;
  }
  return false;
}

void AmclRosNode::requestMap()
{
  boost::recursive_mutex::scoped_lock ml(configuration_mutex_);

  // get map via RPC
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  handleMapMessage( resp.map );
}

void AmclRosNode::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if (amclParams_.first_map_only_ && first_map_received_) {
    return;
  }
    // check if we can localize from initial pose params on server or not
  bool initial_pose_found_from_server = checkPoseParamOnServer();
  // Initialize the filter
  updatePoseFromServer();

  handleMapMessage( *msg );

  first_map_received_ = true;
  // delete all the params for next map update
  deletePoseFromServer();

  //if we didn't initialize using param server tell core (publish)
  publishStatusOnNewWorkArea(true, initial_pose_found_from_server);
}

bool AmclRosNode::getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                      double& x, double& y, double& yaw,
                      const ros::Time& t, const std::string& f)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                           tf::Vector3(0,0,0)), t, f);
  try
  {
    this->tf_->transformPose(amclParams_.odom_frame_id_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.getOrigin().x();
  y = odom_pose.getOrigin().y();
  double pitch,roll;
  odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);

  return true;
}

void AmclRosNode::checkLaserReceived(const ros::TimerEvent& event)
{
  ros::Duration d = ros::Time::now() - last_laser_received_ts_;
  if(d > laser_check_interval_)
  {
    ROS_WARN("No laser scan received (and thus no pose updates have been published) for %f seconds.  Verify that data is being published on the %s topic.",
             d.toSec(),
             ros::names::resolve(scan_topic_).c_str());
  }
}

void AmclRosNode::reconfigureCB(AMCLConfig &config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  //we don't want to do anything on the first call
  //which corresponds to startup
  if(first_reconfigure_call_)
  {
    first_reconfigure_call_ = false;
    default_config_ = config;
    return;
  }

  if(config.restore_defaults) {
    config = default_config_;
    //avoid looping
    config.restore_defaults = false;
  }

  amclParams_.d_thresh_ = config.update_min_d;
  amclParams_.a_thresh_ = config.update_min_a;

  amclParams_.resample_interval_ = config.resample_interval;

  amclParams_.laser_min_range_ = config.laser_min_range;
  amclParams_.laser_max_range_ = config.laser_max_range;

  amclParams_.gui_publish_period = ros::Duration(1.0/config.gui_publish_rate);
  amclParams_.save_pose_period = ros::Duration(1.0/config.save_pose_rate);

  amclParams_.transform_tolerance_.fromSec(config.transform_tolerance);

  amclParams_.max_beams_ = config.laser_max_beams;
  amclParams_.alpha1_ = config.odom_alpha1;
  amclParams_.alpha2_ = config.odom_alpha2;
  amclParams_.alpha3_ = config.odom_alpha3;
  amclParams_.alpha4_ = config.odom_alpha4;
  amclParams_.alpha5_ = config.odom_alpha5;

  amclParams_.z_hit_ = config.laser_z_hit;
  amclParams_.z_short_ = config.laser_z_short;
  amclParams_.z_max_ = config.laser_z_max;
  amclParams_.z_rand_ = config.laser_z_rand;
  amclParams_.sigma_hit_ = config.laser_sigma_hit;
  amclParams_.lambda_short_ = config.laser_lambda_short;
  amclParams_.laser_likelihood_max_dist_ = config.laser_likelihood_max_dist;

  if(config.laser_model_type == "beam")
    amclParams_.laser_model_type_ = LASER_MODEL_BEAM;
  else if(config.laser_model_type == "likelihood_field")
    amclParams_.laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  else if(config.laser_model_type == "likelihood_field_prob")
    amclParams_.laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;

  if(config.odom_model_type == "diff")
    amclParams_.odom_model_type_ = ODOM_MODEL_DIFF;
  else if(config.odom_model_type == "omni")
    amclParams_.odom_model_type_ = ODOM_MODEL_OMNI;
  else if(config.odom_model_type == "diff-corrected")
    amclParams_.odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if(config.odom_model_type == "omni-corrected")
    amclParams_.odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;

  if(config.min_particles > config.max_particles)
  {
    ROS_WARN("You've set min_particles to be greater than max particles, this isn't allowed so they'll be set to be equal.");
    config.max_particles = config.min_particles;
  }

  amclParams_.min_particles_ = config.min_particles;
  amclParams_.max_particles_ = config.max_particles;
  amclParams_.alpha_slow_ = config.recovery_alpha_slow;
  amclParams_.alpha_fast_ = config.recovery_alpha_fast;
  amclParams_.tf_broadcast_ = config.tf_broadcast;

  amclParams_.do_beamskip_= config.do_beamskip;
  amclParams_.beam_skip_distance_ = config.beam_skip_distance;
  amclParams_.beam_skip_threshold_ = config.beam_skip_threshold;

  pf_ = pf_alloc(amclParams_.min_particles_, amclParams_.max_particles_,
                 amclParams_.alpha_slow_, amclParams_.alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);
  amclParams_.pf_err_ = config.kld_err;
  amclParams_.pf_z_ = config.kld_z;
  pf_->pop_err = amclParams_.pf_err_;
  pf_->pop_z = amclParams_.pf_z_;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = last_published_pose.pose.pose.position.x;
  pf_init_pose_mean.v[1] = last_published_pose.pose.pose.position.y;
  pf_init_pose_mean.v[2] = tf::getYaw(last_published_pose.pose.pose.orientation);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = last_published_pose.pose.covariance[6*0+0];
  pf_init_pose_cov.m[1][1] = last_published_pose.pose.covariance[6*1+1];
  pf_init_pose_cov.m[2][2] = last_published_pose.pose.covariance[6*5+5];
  pf_init(pf_, 1, &pf_init_pose_mean, &pf_init_pose_cov);
  pf_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  odom_->SetModel( amclParams_.odom_model_type_, amclParams_.alpha1_, 
    amclParams_.alpha2_, amclParams_.alpha3_, amclParams_.alpha4_, amclParams_.alpha5_ );
  // Laser
  delete laser_;
  laser_ = new AMCLLaser(amclParams_.max_beams_, map_);
  ROS_ASSERT(laser_);
  if(amclParams_.laser_model_type_ == LASER_MODEL_BEAM)
    laser_->SetModelBeam(amclParams_.z_hit_, amclParams_.z_short_, amclParams_.z_max_, amclParams_.z_rand_,
                         amclParams_.sigma_hit_, amclParams_.lambda_short_, 0.0);
  else if(amclParams_.laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldProb(amclParams_.z_hit_, amclParams_.z_rand_, amclParams_.sigma_hit_,
					amclParams_.laser_likelihood_max_dist_,
					amclParams_.do_beamskip_, amclParams_.beam_skip_distance_,
					amclParams_.beam_skip_threshold_, amclParams_.beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model with probabilities.");
  }
  else if(amclParams_.laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(amclParams_.z_hit_, amclParams_.z_rand_, amclParams_.sigma_hit_,
                                    amclParams_.laser_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }

  amclParams_.odom_frame_id_ = config.odom_frame_id;
  amclParams_.base_frame_id_ = config.base_frame_id;
  amclParams_.global_frame_id_ = config.global_frame_id;

  delete laser_scan_filter_;
  laser_scan_filter_ =
          new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                        *tf_,
                                                        amclParams_.odom_frame_id_,
                                                        100);
  laser_scan_filter_->registerCallback(boost::bind(&AmclRosNode::laserReceived,
                                                   this, _1));

  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclRosNode::initialPoseReceived, this);
}

void AmclRosNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
}