#include <amcl.h>

std::vector<std::pair<int,int> > AmclNode::free_space_indices;

using namespace amcl;

AmclNode::AmclNode():
    map_(NULL),
    pf_(NULL),
    resample_count_(0),
    odom_(NULL),
    laser_(NULL),
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

// void AmclNode::setTfBuffer(tf2_ros::Buffer tf_buffer)
// {
//     tf_buffer_ = tf_buffer;
// }

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
  pf_ = pf_alloc(amclParams_.min_particles_, amclParams_.max_particles_,
                 amclParams_.alpha_slow_, amclParams_.alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);
  pf_->pop_err = amclParams_.pf_err_;
  pf_->pop_z = amclParams_.pf_z_;

  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0];
  pf_init_pose_cov.m[1][1] = init_cov_[1];
  pf_init_pose_cov.m[2][2] = init_cov_[2];
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
    ROS_INFO("Done initializing likelihood field model.");
  }
  else
  {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(amclParams_.z_hit_, amclParams_.z_rand_, amclParams_.sigma_hit_,
                                    amclParams_.laser_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }
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

//   boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
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
    // if(sent_first_transform_)
    //   ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
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
