#include <amcl.h>

std::vector<std::pair<int,int> > AmclNode::free_space_indices;

using namespace amcl;

AmclNode::AmclNode():
    map_(NULL),
    pf_(NULL),
    resample_count_(0),
    odom_(NULL),
    laser_(NULL),
    latest_tf_valid_(false),
    sent_first_transform_(false),
    hasNewOdomToMapTf_(false),
    initial_poses_hyp_()
{
    std::cout << "amcl node constructor" << std::endl;
    init_pose_[0] = 0.0;
    init_pose_[1] = 0.0;
    init_pose_[2] = 0.0;
    init_cov_[0] = 0.5 * 0.5;
    init_cov_[1] = 0.5 * 0.5;
    init_cov_[2] = (M_PI/12.0) * (M_PI/12.0);
}

AmclNode::~AmclNode()
{
    std::cout << "amcl node destructor" << std::endl;
    freeMapDependentMemory();
}

void AmclNode::setInitialPose(double init_pose[3],
    double init_cov[3])
{
    init_pose_[0] = init_pose[0];
    init_pose_[1] = init_pose[1];
    init_pose_[2] = init_pose[2];
    init_cov_[0] = init_cov[0];
    init_cov_[1] = init_cov[1];
    init_cov_[2] = init_cov[2];
}

bool AmclNode::globalInitialization()
{
  if( map_ == NULL ) {
    return true;
  }
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  ROS_INFO("Initializing with uniform distribution");
  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                (void *)map_);
  ROS_INFO("Global initialisation done!");
  pf_init_ = false;
  return true;
}

/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state.  initial_pose_hyp_ is deleted
 * and set to NULL after it is used.
 */
bool AmclNode::applyInitialPose()
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  if (initial_poses_hyp_.size() && map_ != NULL ) {
	std::vector<pf_vector_t> initial_means;
	std::vector<pf_matrix_t> initial_covs;

	for (auto poses : initial_poses_hyp_)
	{
		initial_means.push_back(poses.pf_pose_mean);
		initial_covs.push_back(poses.pf_pose_cov);
	}

    pf_init(pf_, initial_poses_hyp_.size(), &initial_means[0], &initial_covs[0]);
    pf_init_ = false;

    initial_poses_hyp_.erase(initial_poses_hyp_.begin(), initial_poses_hyp_.end());
  }
  return true;
}

void AmclNode::freeMapDependentMemory()
{
  if (map_ != NULL ) {
    map_free( map_ );
    map_ = NULL;
  }
  if (pf_ != NULL ) {
    pf_free( pf_ );
    pf_ = NULL;
  }
  delete odom_;
  odom_ = NULL;
  delete laser_;
  laser_ = NULL;
}

pf_vector_t AmclNode::uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;
#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int,int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
  max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
  min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
  max_y = (map->size_y * map->scale)/2.0 + map->origin_y;

  pf_vector_t p;

  ROS_DEBUG("Generating new uniform sample");
  for(;;)
  {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i,j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
      break;
  }
#endif
  return p;
}

/**
 * Convert an OccupancyGrid map message into the internal
 * representation.  This allocates a map_t and returns it.
 */
map_t* AmclNode::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(map_msg.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if(map_msg.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }

  return map;
}

double AmclNode::getYaw(tf::Pose& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw,pitch,roll);
  return yaw;
}

void AmclNode::instantiateOdom()
{
  delete odom_;
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  odom_->SetModel( amclParams_.odom_model_type_, amclParams_.alpha1_,
    amclParams_.alpha2_, amclParams_.alpha3_, amclParams_.alpha4_, amclParams_.alpha5_ );
}

void AmclNode::instantiateLaser()
{
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
    ROS_INFO("Done initializing likelihood field model.");
  }
  else
  {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(amclParams_.z_hit_, amclParams_.z_rand_, amclParams_.sigma_hit_,
                                    amclParams_.laser_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }
}

void AmclNode::instantiateParticleFilter(pf_vector_t pf_init_pose_mean, pf_matrix_t pf_init_pose_cov)
{
  pf_ = pf_alloc(amclParams_.min_particles_, amclParams_.max_particles_,
                 amclParams_.alpha_slow_, amclParams_.alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);
  pf_->pop_err = amclParams_.pf_err_;
  pf_->pop_z = amclParams_.pf_z_;
  pf_init(pf_, 1, &pf_init_pose_mean, &pf_init_pose_cov);
  pf_init_ = false;
}

void AmclNode::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution);

  freeMapDependentMemory();
  // Clear queued laser objects because they hold pointers to the existing
  // map, #5202.
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();

  map_ = convertMap(msg);

#if NEW_UNIFORM_SAMPLING
  // Index of free space
  free_space_indices.resize(0);
  for(int i = 0; i < map_->size_x; i++)
    for(int j = 0; j < map_->size_y; j++)
      if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
        free_space_indices.push_back(std::make_pair(i,j));
#endif
  // Create the particle filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0];
  pf_init_pose_cov.m[1][1] = init_cov_[1];
  pf_init_pose_cov.m[2][2] = init_cov_[2];
  instantiateParticleFilter(pf_init_pose_mean, pf_init_pose_cov);

  // Instantiate the sensor objects
  // Odometry
  instantiateOdom();
  // Laser
  instantiateLaser();
  // In case the initial pose message arrived before the first map,
  // try to apply the initial pose now that the map has arrived.
  applyInitialPose();
}


bool AmclNode::handleInitialPosesMessage(const move_base_msgs::PoseWithCovarianceStampedArray& msg)
{
  if(msg.poses.size() == 0)
  {
	  ROS_ERROR("Received an empty initial pose array.");
	  return false;
  }

  boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
  if(msg.header.frame_id == "")
  {
    // This should be removed at some point
    ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
  }
  // We only accept initial pose estimates in the global frame, #5148.
  // NOTE: Check if this statement is correct as compared to original
  else if(msg.header.frame_id != amclParams_.global_frame_id_)
  {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             msg.header.frame_id.c_str(),
             amclParams_.global_frame_id_.c_str());
    return false;
  }

  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::StampedTransform tx_odom;
  try
  {
    ros::Time now = ros::Time::now();
    geometry_msgs::TransformStamped t_odom = tf_buffer_.lookupTransform(amclParams_.base_frame_id_, msg.header.stamp,
                         amclParams_.base_frame_id_, now,
                         amclParams_.odom_frame_id_, ros::Duration(0.5));
    tf::transformStampedMsgToTF(t_odom, tx_odom);
  }
  catch(tf::TransformException e)
  {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if(sent_first_transform_)
      ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
    tx_odom.setIdentity();
  }

  std::ostringstream stream;
  	stream << "Setting pose (" << ros::Time::now().toSec() << "):";

  initial_poses_hyp_.erase(initial_poses_hyp_.begin(), initial_poses_hyp_.end());

  for(auto pose : msg.poses)
  {
	  tf::Pose pose_old, pose_new;
	  tf::poseMsgToTF(pose.pose, pose_old);
	  pose_new = pose_old * tx_odom;

	  stream << " ";
	  stream << pose_new.getOrigin().x() << " ";
	  stream << pose_new.getOrigin().y() << " ";
	  stream << getYaw(pose_new) << ",";

	  // Re-initialize the filter
	  pf_vector_t pf_init_pose_mean = pf_vector_zero();
	  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
	  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
	  pf_init_pose_mean.v[2] = getYaw(pose_new);
	  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();

	  // Copy in the covariance, converting from 6-D to 3-D
	  for(int i=0; i<2; i++)
	  {
	    for(int j=0; j<2; j++)
	    {
	      pf_init_pose_cov.m[i][j] = pose.covariance[6*i+j];
	    }
	  }
	  pf_init_pose_cov.m[2][2] = pose.covariance[6*5+5];

	  amcl_hyp_t hyp;

	  hyp.pf_pose_mean = pf_init_pose_mean;
	  hyp.pf_pose_cov = pf_init_pose_cov;

	  initial_poses_hyp_.push_back(hyp);
  }

  std::string strData = stream.str( );

  ROS_INFO_STREAM( strData );

  return applyInitialPose();
}

bool AmclNode::getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                      double& x, double& y, double& yaw,
                      const ros::Time& t, const std::string& f)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                           tf::Vector3(0,0,0)), t, f);
  geometry_msgs::PoseStamped ident_pose;
  tf::poseStampedTFToMsg(ident, ident_pose);
  try
  {
    geometry_msgs::PoseStamped pose_out;
    tf_buffer_.transform(ident_pose, pose_out, amclParams_.odom_frame_id_);
    tf::poseStampedMsgToTF(pose_out, odom_pose);
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

void AmclNode::handleLaserScanMessage(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  if (map_ == NULL) {
    return;
  }

  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  int laser_index = -1;

  // Do we have the base->base_laser Tx yet?
  if(frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end())
  {
    ROS_DEBUG("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan->header.frame_id.c_str());
    lasers_.push_back(new AMCLLaser(*laser_));
    lasers_update_.push_back(true);
    laser_index = frame_to_laser_.size();

    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                             tf::Vector3(0,0,0)),
                                 ros::Time(), laser_scan->header.frame_id);
    geometry_msgs::PoseStamped ident_pose;
    tf::poseStampedTFToMsg(ident, ident_pose);
    tf::Stamped<tf::Pose> laser_pose;
    try
    {
      geometry_msgs::PoseStamped pose_out;
      tf_buffer_.transform(ident_pose, pose_out, std::string(amclParams_.base_frame_id_));
      tf::poseStampedMsgToTF(pose_out, laser_pose);
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                laser_scan->header.frame_id.c_str(),
                amclParams_.base_frame_id_.c_str());
      return;
    }

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.getOrigin().x();
    laser_pose_v.v[1] = laser_pose.getOrigin().y();
    // laser mounting angle gets computed later -> set to 0 here!
    laser_pose_v.v[2] = 0;
    lasers_[laser_index]->SetLaserPose(laser_pose_v);
    ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0],
              laser_pose_v.v[1],
              laser_pose_v.v[2]);

    frame_to_laser_[laser_scan->header.frame_id] = laser_index;
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if(!getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
                  laser_scan->header.stamp, amclParams_.base_frame_id_))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }

  pf_vector_t delta = pf_vector_zero();

  if(pf_init_)
  {
    // Compute change in pose
    //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

    // See if we should update the filter
    bool update = fabs(delta.v[0]) > amclParams_.d_thresh_ ||
                  fabs(delta.v[1]) > amclParams_.d_thresh_ ||
                  fabs(delta.v[2]) > amclParams_.a_thresh_;
    update = update || force_update_;
    force_update_ = false;

    // Set the laser update flags
    if(update)
      for(unsigned int i=0; i < lasers_update_.size(); i++)
        lasers_update_[i] = true;
  }

  bool force_publication = false;
  if(!pf_init_)
  {
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is now initialized
    pf_init_ = true;

    // Should update sensor data
    for(unsigned int i=0; i < lasers_update_.size(); i++)
      lasers_update_[i] = true;

    force_publication = true;

    resample_count_ = 0;
  }
  // If the robot has moved, update the filter
  else if(pf_init_ && lasers_update_[laser_index])
  {
    //printf("pose\n");
    //pf_vector_fprintf(pose, stdout, "%.3f");

    AMCLOdomData odata;
    odata.pose = pose;
    // HACK
    // Modify the delta in the action data so the filter gets
    // updated correctly
    odata.delta = delta;

    // Use the action data to update the filter
    odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);

    // Pose at last filter update
    //this->pf_odom_pose = pose;
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if(lasers_update_[laser_index])
  {
    // srs::ScopedTimingSampleRecorder stsr_cb(tdr_.getRecorder("-LaserCB"));

    AMCLLaserData ldata;
    ldata.sensor = lasers_[laser_index];
    ldata.range_count = laser_scan->ranges.size();

    // To account for lasers that are mounted upside-down, we determine the
    // min, max, and increment angles of the laser in the base frame.
    // Construct min and max angles of laser, in the base_link frame.
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, laser_scan->angle_min);
    tf::Stamped<tf::Quaternion> min_q(q, laser_scan->header.stamp,
                                      laser_scan->header.frame_id);
    q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
    tf::Stamped<tf::Quaternion> inc_q(q, laser_scan->header.stamp,
                                      laser_scan->header.frame_id);
    geometry_msgs::QuaternionStamped min_q_msg;
    tf::quaternionStampedTFToMsg(min_q, min_q_msg);
    geometry_msgs::QuaternionStamped inc_q_msg;
    tf::quaternionStampedTFToMsg(inc_q, inc_q_msg);
    try
    {
      tf_buffer_.transform(min_q_msg, min_q_msg, amclParams_.base_frame_id_);
      tf_buffer_.transform(inc_q_msg, inc_q_msg, amclParams_.base_frame_id_);
      tf::quaternionStampedMsgToTF(min_q_msg, min_q);
      tf::quaternionStampedMsgToTF(inc_q_msg, inc_q);
    }
    catch(tf::TransformException& e)
    {
      ROS_WARN("Unable to transform min/max laser angles into base frame: %s",
               e.what());
      return;
    }

    double angle_min = tf::getYaw(min_q);
    double angle_increment = tf::getYaw(inc_q) - angle_min;

    // wrapping angle to [-pi .. pi]
    angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;

    ROS_DEBUG("Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min, angle_increment);

    // Apply range min/max thresholds, if the user supplied them
    if(amclParams_.laser_max_range_ > 0.0)
      ldata.range_max = std::min(laser_scan->range_max, (float)amclParams_.laser_max_range_);
    else
      ldata.range_max = laser_scan->range_max;
    double range_min;
    if(amclParams_.laser_min_range_ > 0.0)
      range_min = std::max(laser_scan->range_min, (float)amclParams_.laser_min_range_);
    else
      range_min = laser_scan->range_min;
    // The AMCLLaserData destructor will free this memory
    ldata.ranges = new double[ldata.range_count][2];
    ROS_ASSERT(ldata.ranges);
    for(int i=0;i<ldata.range_count;i++)
    {
      // amcl doesn't (yet) have a concept of min range.  So we'll map short
      // readings to max range.
      if(laser_scan->ranges[i] <= range_min)
        ldata.ranges[i][0] = ldata.range_max;
      else
        ldata.ranges[i][0] = laser_scan->ranges[i];
      // Compute bearing
      ldata.ranges[i][1] = angle_min +
              (i * angle_increment);
    }

    float percent_invalid_poses = 0.0;
    lasers_[laser_index]->UpdateSensor(pf_, (AMCLSensorData*)&ldata, &percent_invalid_poses);

    amclAnalyticsData_.percent_invalid_poses_ = percent_invalid_poses;

    lasers_update_[laser_index] = false;

    pf_odom_pose_ = pose;

    // Resample the particles
    if(!(++resample_count_ % amclParams_.resample_interval_))
    {
      pf_update_resample(pf_);
      resampled = true;
    }

    pf_sample_set_t* set = pf_->sets + pf_->current_set;
    ROS_DEBUG("Num samples: %d\n", set->sample_count);

    // Publish the resulting cloud
    if (!force_update_) {
      geometry_msgs::PoseArray cloud_msg;
      cloud_msg.header.stamp = ros::Time::now();
      cloud_msg.header.frame_id = amclParams_.global_frame_id_;
      cloud_msg.poses.resize(set->sample_count);
      for(int i=0;i<set->sample_count;i++)
      {
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                                 tf::Vector3(set->samples[i].pose.v[0],
                                           set->samples[i].pose.v[1], 0)),
                        cloud_msg.poses[i]);
      }
      amclAnalyticsData_.particle_cloud_msg_ = cloud_msg;
    }
  }
  if(resampled || force_publication)
  {
    updateHypothesis(laser_scan);
  }
  else if(latest_tf_valid_)
  {
    if (amclParams_.tf_broadcast_ == true)
    {
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      ros::Time transform_expiration = (laser_scan->header.stamp +
                                        amclParams_.transform_tolerance_);
      tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                          transform_expiration,
                                          amclParams_.global_frame_id_, amclParams_.odom_frame_id_);
      odom_to_map_tf_ = tmp_tf_stamped;
      hasNewOdomToMapTf_ = true;
    }
  }
}

void AmclNode::updateHypothesis(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
    // Read out the current hypotheses
  move_base_msgs::amcl_data dp; //data_msg

  double max_weight = 0.0;
  int max_weight_hyp = -1;
  std::vector<amcl_hyp_t> hyps;
  hyps.resize(pf_->sets[pf_->current_set].cluster_count);
  int total_num_samples = 0;
  for(int hyp_count = 0;
      hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
  {
    move_base_msgs::cluster temp_cluster; //cluster to push back to data_msg.clusters
    double weight;
    int numSamples;
    pf_vector_t pose_mean;
    pf_matrix_t pose_cov;
    if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov, &numSamples))
    {
      ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
      break;
    }

    hyps[hyp_count].weight = weight;
    hyps[hyp_count].pf_pose_mean = pose_mean;
    hyps[hyp_count].pf_pose_cov = pose_cov;
    hyps[hyp_count].numSamples = numSamples;

    if(hyps[hyp_count].weight > max_weight)
    {
      max_weight = hyps[hyp_count].weight;
      max_weight_hyp = hyp_count;
    }
    temp_cluster.num_samples = numSamples;
    temp_cluster.weight = weight;
    temp_cluster.mean.position.x = hyps[hyp_count].pf_pose_mean.v[0];
    temp_cluster.mean.position.y = hyps[hyp_count].pf_pose_mean.v[1];
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[hyp_count].pf_pose_mean.v[2]),
                          temp_cluster.mean.orientation);
    temp_cluster.cov.xx = hyps[hyp_count].pf_pose_cov.m[0][0];
    temp_cluster.cov.xy = hyps[hyp_count].pf_pose_cov.m[0][1];
    temp_cluster.cov.yy = hyps[hyp_count].pf_pose_cov.m[1][1];
    temp_cluster.cov.tt = hyps[hyp_count].pf_pose_cov.m[2][2];
    dp.clusters.push_back(temp_cluster);
    total_num_samples+=numSamples;
  }
  if(max_weight > 0.0)
  {
    ROS_DEBUG("Max weight pose: %.3f %.3f %.3f",
              hyps[max_weight_hyp].pf_pose_mean.v[0],
              hyps[max_weight_hyp].pf_pose_mean.v[1],
              hyps[max_weight_hyp].pf_pose_mean.v[2]);

    /*
        puts("");
        pf_matrix_fprintf(hyps[max_weight_hyp].pf_pose_cov, stdout, "%6.3f");
        puts("");
      */

    geometry_msgs::PoseWithCovarianceStamped p;
    move_base_msgs::amcl_analytics ap;
    // Fill in the header

    p.header.frame_id = amclParams_.global_frame_id_;
    p.header.stamp = laser_scan->header.stamp;
    // Copy in the pose
    p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
    p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];

    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                          p.pose.pose.orientation);
    // Copy in the covariance, converting from 3-D to 6-D
    pf_sample_set_t* set = pf_->sets + pf_->current_set;


    ap.header.frame_id = amclParams_.global_frame_id_;
    ap.header.stamp = laser_scan->header.stamp;
    ap.set_mean.position.x = set->mean.v[0];
    ap.set_mean.position.y = set->mean.v[1];
    ap.set_mean.position.z = 0;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(set->mean.v[2]),
                          ap.set_mean.orientation);
    ap.set_covariance.xx = set->cov.m[0][0];
    ap.set_covariance.xy = set->cov.m[0][1];
    ap.set_covariance.yy = set->cov.m[1][1];
    ap.set_covariance.tt = set->cov.m[2][2];
    ap.bestCluster.mean.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
    ap.bestCluster.mean.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                          ap.bestCluster.mean.orientation);
    ap.bestCluster.cov.xx = hyps[max_weight_hyp].pf_pose_cov.m[0][0];
    ap.bestCluster.cov.xy = hyps[max_weight_hyp].pf_pose_cov.m[0][1];
    ap.bestCluster.cov.yy = hyps[max_weight_hyp].pf_pose_cov.m[1][1];
    ap.bestCluster.cov.tt = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
    ap.bestCluster.weight =  hyps[max_weight_hyp].weight;
    ap.bestCluster.num_samples = hyps[max_weight_hyp].numSamples;
    ap.num_clusters = pf_->sets[pf_->current_set].cluster_count;
    ap.tot_samples = total_num_samples;
    dp.tot_samples = total_num_samples;
    dp.set_covariance = ap.set_covariance;
    dp.set_mean = ap.set_mean;
    dp.num_clusters= ap.num_clusters;


    for(int i=0; i<2; i++)
    {
      for(int j=0; j<2; j++)
      {
        // Report the overall filter covariance, rather than the
        // covariance for the highest-weight cluster
        //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
        p.pose.covariance[6*i+j] = set->cov.m[i][j];
      }
    }
    // Report the overall filter covariance, rather than the
    // covariance for the highest-weight cluster
    //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
    p.pose.covariance[6*5+5] = set->cov.m[2][2];

    /*
        printf("cov:\n");
        for(int i=0; i<6; i++)
        {
        for(int j=0; j<6; j++)
        printf("%6.3f ", p.covariance[6*i+j]);
        puts("");
        }
      */
    amclAnalyticsData_.amcl_data_ = dp;
    amclAnalyticsData_.pose_data_ = p;
    amclAnalyticsData_.analytics_data_ = ap;
    last_published_pose = p;

    ROS_DEBUG("New pose: %6.3f %6.3f %6.3f",
              hyps[max_weight_hyp].pf_pose_mean.v[0],
              hyps[max_weight_hyp].pf_pose_mean.v[1],
              hyps[max_weight_hyp].pf_pose_mean.v[2]);

    // subtracting base to odom from map to base and send map to odom instead
    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
      tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                            tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
                                        hyps[max_weight_hyp].pf_pose_mean.v[1],
                                        0.0));
      tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                            laser_scan->header.stamp,
                                            amclParams_.base_frame_id_);
      geometry_msgs::PoseStamped tmp_tf_stamped_pose, odom_to_map_pose;
      tf::poseStampedTFToMsg(tmp_tf_stamped, tmp_tf_stamped_pose);
      tf_buffer_.transform(tmp_tf_stamped_pose,
                                odom_to_map_pose, amclParams_.odom_frame_id_);
      tf::poseStampedMsgToTF(odom_to_map_pose, odom_to_map);
    }
    catch(tf::TransformException)
    {
      ROS_DEBUG("Failed to subtract base to odom transform");
      return;
    }

    latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                tf::Point(odom_to_map.getOrigin()));
    latest_tf_valid_ = true;

    if (amclParams_.tf_broadcast_ == true)
    {
      // We want to send a transform that is good up until a
      // tolerance time so that odom can be used
      ros::Time transform_expiration = (laser_scan->header.stamp +
                                        amclParams_.transform_tolerance_);
      tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                          transform_expiration,
                                          amclParams_.global_frame_id_, amclParams_.odom_frame_id_);
      odom_to_map_tf_ = tmp_tf_stamped;
      sent_first_transform_ = true;
      hasNewOdomToMapTf_ = true;
    }
  }
  else
  {
    ROS_ERROR("No pose!");
  }
}
