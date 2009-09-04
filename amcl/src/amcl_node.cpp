/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */

#include <algorithm>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include "map/map.h"
#include "pf/pf.h"
#include "sensors/amcl_odom.h"
#include "sensors/amcl_laser.h"

#include "ros/assert.h"

// roscpp
#include "ros/node.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_notifier.h"

using namespace amcl;

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

class AmclNode
{
  public:
    AmclNode();
    ~AmclNode();

    int process();

  private:
    tf::TransformBroadcaster* tfb_;
    tf::TransformListener* tf_;

    tf::Transform latest_tf_;
    bool latest_tf_valid_;

    // Pose-generating function used to uniformly distribute particles over
    // the map
    static pf_vector_t uniformPoseGenerator(void* arg);

    // incoming messages
    geometry_msgs::PoseWithCovarianceStamped initial_pose_;

    // Message callbacks
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
    void laserReceived(const tf::MessageNotifier<sensor_msgs::LaserScan>::MessagePtr& laser_scan);
    void initialPoseReceived();

    double getYaw(tf::Pose& t);

    //parameter for what odom to use
    std::string odom_frame_id_;
    //parameter for what base to use
    std::string base_frame_id_;

    ros::Duration gui_publish_period;
    ros::Time save_pose_last_time;
    ros::Duration save_pose_period;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;
    bool have_laser_pose;

    tf::MessageNotifier<sensor_msgs::LaserScan>* laser_scan_notifer;

    // Particle filter
    pf_t *pf_;
    boost::mutex pf_mutex_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;
    double laser_min_range_;
    double laser_max_range_;

    AMCLOdom* odom_;
    AMCLLaser* laser_;

    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    map_t* requestMap();

    // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<btTransform>& pose,
                     double& x, double& y, double& yaw,
                     const ros::Time& t, const std::string& f);

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;
};

#define USAGE "USAGE: amcl"

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  ros::Node n("amcl");

  AmclNode an;

  n.spin();

  // To quote Morgan, Hooray!
  return(0);
}

AmclNode::AmclNode() :
        latest_tf_valid_(false),
        map_(NULL),
        have_laser_pose(false),
        pf_(NULL),
        resample_count_(0)
{
  // Grab params off the param server
  int max_beams, min_particles, max_particles;
  double alpha1, alpha2, alpha3, alpha4, alpha5;
  double alpha_slow, alpha_fast;
  double z_hit, z_short, z_max, z_rand, sigma_hit, lambda_short;
  double pf_err, pf_z;

  double tmp;
  ros::Node::instance()->param("~gui_publish_rate", tmp, -1.0);
  gui_publish_period = ros::Duration(1.0/tmp);
  ros::Node::instance()->param("~save_pose_rate", tmp, 0.5);
  save_pose_period = ros::Duration(1.0/tmp);

  ros::Node::instance()->param("~laser_min_range", laser_min_range_, -1.0);
  ros::Node::instance()->param("~laser_max_range", laser_max_range_, -1.0);
  ros::Node::instance()->param("~laser_max_beams", max_beams, 30);
  ros::Node::instance()->param("~min_particles", min_particles, 100);
  ros::Node::instance()->param("~max_particles", max_particles, 5000);
  ros::Node::instance()->param("~kld_err", pf_err, 0.01);
  ros::Node::instance()->param("~kld_z", pf_z, 0.99);
  ros::Node::instance()->param("~odom_alpha1", alpha1, 0.2);
  ros::Node::instance()->param("~odom_alpha2", alpha2, 0.2);
  ros::Node::instance()->param("~odom_alpha3", alpha3, 0.2);
  ros::Node::instance()->param("~odom_alpha4", alpha4, 0.2);
  ros::Node::instance()->param("~odom_alpha5", alpha5, 0.2);

  ros::Node::instance()->param("~laser_z_hit", z_hit, 0.95);
  ros::Node::instance()->param("~laser_z_short", z_short, 0.1);
  ros::Node::instance()->param("~laser_z_max", z_max, 0.05);
  ros::Node::instance()->param("~laser_z_rand", z_rand, 0.05);
  ros::Node::instance()->param("~laser_sigma_hit", sigma_hit, 0.2);
  ros::Node::instance()->param("~laser_lambda_short", lambda_short, 0.1);
  double laser_likelihood_max_dist;
  ros::Node::instance()->param("~laser_likelihood_max_dist",
                               laser_likelihood_max_dist, 2.0);
  std::string tmp_model_type;
  laser_model_t laser_model_type;
  ros::Node::instance()->param("~laser_model_type", tmp_model_type, std::string("likelihood_field"));
  if(tmp_model_type == "beam")
    laser_model_type = LASER_MODEL_BEAM;
  else if(tmp_model_type == "likelihood_field")
    laser_model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  else
  {
    ROS_WARN("Unknown laser model type \"%s\"; defaulting to likelihood_field model",
             tmp_model_type.c_str());
    laser_model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  }

  odom_model_t odom_model_type;
  ros::Node::instance()->param("~odom_model_type", tmp_model_type, std::string("diff"));
  if(tmp_model_type == "diff")
    odom_model_type = ODOM_MODEL_DIFF;
  else if(tmp_model_type == "omni")
    odom_model_type = ODOM_MODEL_OMNI;
  else
  {
    ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model",
             tmp_model_type.c_str());
    odom_model_type = ODOM_MODEL_DIFF;
  }

  ros::Node::instance()->param("~update_min_d", d_thresh_, 0.2);
  ros::Node::instance()->param("~update_min_a", a_thresh_, M_PI/6.0);
  ros::Node::instance()->param("~odom_frame_id", odom_frame_id_, std::string("odom"));
  ros::Node::instance()->param("~base_frame_id", base_frame_id_, std::string("base_link"));
  ros::Node::instance()->param("~resample_interval", resample_interval_, 2);
  double tmp_tol;
  ros::Node::instance()->param("~transform_tolerance", tmp_tol, 0.1);
  ros::Node::instance()->param("~recovery_alpha_slow", alpha_slow, 0.001);
  ros::Node::instance()->param("~recovery_alpha_fast", alpha_fast, 0.1);


  transform_tolerance_.fromSec(tmp_tol);

  double init_pose[3];
  ros::Node::instance()->param("~initial_pose_x", init_pose[0], 0.0);
  ros::Node::instance()->param("~initial_pose_y", init_pose[1], 0.0);
  ros::Node::instance()->param("~initial_pose_a", init_pose[2], 0.0);
  double init_cov[3];
  ros::Node::instance()->param("~initial_cov_xx", init_cov[0], 0.5 * 0.5);
  ros::Node::instance()->param("~initial_cov_yy", init_cov[1], 0.5 * 0.5);
  ros::Node::instance()->param("~initial_cov_aa", init_cov[2], 
                               (M_PI/12.0) * (M_PI/12.0));

  cloud_pub_interval.fromSec(1.0);
  tfb_ = new tf::TransformBroadcaster();
  tf_ = new tf::TransformListener(*ros::Node::instance());

  map_ = requestMap();

  // Create the particle filter
  pf_ = pf_alloc(min_particles, max_particles,
                 alpha_slow, alpha_fast,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);
  pf_->pop_err = pf_err;
  pf_->pop_z = pf_z;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose[0];
  pf_init_pose_mean.v[1] = init_pose[1];
  pf_init_pose_mean.v[2] = init_pose[2];
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov[0];
  pf_init_pose_cov.m[1][1] = init_cov[1];
  pf_init_pose_cov.m[2][2] = init_cov[2];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  if(odom_model_type == ODOM_MODEL_OMNI)
    odom_->SetModelOmni(alpha1, alpha2, alpha3, alpha4, alpha5);
  else
    odom_->SetModelDiff(alpha1, alpha2, alpha3, alpha4);
  // Laser
  laser_ = new AMCLLaser(max_beams, map_);
  ROS_ASSERT(laser_);
  if(laser_model_type == LASER_MODEL_BEAM)
    laser_->SetModelBeam(z_hit, z_short, z_max, z_rand,
                         sigma_hit, lambda_short, 0.0);
  else
    laser_->SetModelLikelihoodField(z_hit, z_rand, sigma_hit,
                                    laser_likelihood_max_dist);

  ros::Node::instance()->advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",2);
  ros::Node::instance()->advertise<geometry_msgs::PoseArray>("particlecloud",2);
  ros::Node::instance()->advertiseService("global_localization",
                                          &AmclNode::globalLocalizationCallback,
                                          this);
  laser_scan_notifer =
          new tf::MessageNotifier<sensor_msgs::LaserScan>
          (tf_, ros::Node::instance(),
           boost::bind(&AmclNode::laserReceived,
                       this, _1),
           "scan", odom_frame_id_,
           100);
  ros::Node::instance()->subscribe("initialpose", initial_pose_, &AmclNode::initialPoseReceived,this,2);
}

map_t*
AmclNode::requestMap()
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

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
  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           resp.map.info.width,
           resp.map.info.height,
           resp.map.info.resolution);

  map->size_x = resp.map.info.width;
  map->size_y = resp.map.info.height;
  map->scale = resp.map.info.resolution;
  map->origin_x = resp.map.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = resp.map.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(resp.map.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if(resp.map.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }

  return map;
}

AmclNode::~AmclNode()
{
  map_free(map_);
  delete tfb_;
  delete tf_;
  pf_free(pf_);
  delete laser_;
  delete odom_;
  // TODO: delete everything allocated in constructor
}

bool
AmclNode::getOdomPose(tf::Stamped<btTransform>& odom_pose,
                      double& x, double& y, double& yaw,
                      const ros::Time& t, const std::string& f)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (btTransform(btQuaternion(0,0,0),
                                           btVector3(0,0,0)), t, f);
  try
  {
    this->tf_->transformPose(odom_frame_id_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.getOrigin().x();
  y = odom_pose.getOrigin().y();
  double pitch,roll;
  odom_pose.getBasis().getEulerZYX(yaw, pitch, roll);

  return true;
}


pf_vector_t
AmclNode::uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
  max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
  min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
  max_y = (map->size_y * map->scale)/2.0 + map->origin_y;

  pf_vector_t p;

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

  return p;
}

bool
AmclNode::globalLocalizationCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
  pf_mutex_.lock();
  ROS_INFO("Initializing with uniform distribution");
  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                (void *)map_);
  pf_init_ = false;
  pf_mutex_.unlock();
  return true;
}

void
AmclNode::laserReceived(const tf::MessageNotifier<sensor_msgs::LaserScan>::MessagePtr& laser_scan)
{
  // Do we have the base->base_laser Tx yet?
  if(!have_laser_pose)
  {
    tf::Stamped<tf::Pose> ident (btTransform(btQuaternion(0,0,0),
                                             btVector3(0,0,0)),
                                 ros::Time(), laser_scan->header.frame_id);
    tf::Stamped<btTransform> laser_pose;
    try
    {
      this->tf_->transformPose("base_footprint", ident, laser_pose);
    }
    catch(tf::TransformException e)
    {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                laser_scan->header.frame_id.c_str(),
                "base_footprint");
      return;
    }

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.getOrigin().x();
    laser_pose_v.v[1] = laser_pose.getOrigin().y();
    double p,r;
    laser_pose.getBasis().getEulerZYX(laser_pose_v.v[2],p,r);
    laser_->SetLaserPose(laser_pose_v);
    ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0],
              laser_pose_v.v[1],
              laser_pose_v.v[2]);
    have_laser_pose = true;
  }

  // Where was the robot when this scan was taken?
  tf::Stamped<btTransform> odom_pose;
  pf_vector_t pose;
  if(!getOdomPose(odom_pose, pose.v[0], pose.v[1], pose.v[2],
                  laser_scan->header.stamp, "base_footprint"))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }

  pf_mutex_.lock();

  bool update = false;
  pf_vector_t delta = pf_vector_zero();

  if(pf_init_)
  {
    // Compute change in pose
    //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

    // See if we should update the filter
    update = fabs(delta.v[0]) > d_thresh_ ||
             fabs(delta.v[1]) > d_thresh_ ||
             fabs(delta.v[2]) > a_thresh_;
  }

  bool force_publication = false;
  if(!pf_init_)
  {
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is now initialized
    pf_init_ = true;

    // Should update sensor data
    update = true;
    force_publication = true;

    resample_count_ = 0;
  }
  // If the robot has moved, update the filter
  else if(pf_init_ && update)
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
  if(update)
  {
    AMCLLaserData ldata;
    ldata.sensor = laser_;
    ldata.range_count = laser_scan->ranges.size();

    // Apply min/max thresholds, if the user supplied them
    if(laser_max_range_ > 0.0)
      ldata.range_max = std::min(laser_scan->range_max, (float)laser_max_range_);
    else
      ldata.range_max = laser_scan->range_max;
    double range_min;
    if(laser_min_range_ > 0.0)
      range_min = std::max(laser_scan->range_min, (float)laser_min_range_);
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
      ldata.ranges[i][1] = laser_scan->angle_min +
              (i * laser_scan->angle_increment);
    }

    laser_->UpdateSensor(pf_, (AMCLSensorData*)&ldata);
    pf_odom_pose_ = pose;

    // Resample the particles
    if(!(++resample_count_ % resample_interval_))
    {
      pf_update_resample(pf_);
      resampled = true;
    }

    pf_sample_set_t* set = pf_->sets + pf_->current_set;
    ROS_DEBUG("Num samples: %d\n", set->sample_count);

    // Publish the resulting cloud
    // TODO: set maximum rate for publishing
    geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";
    cloud_msg.set_poses_size(set->sample_count);
    for(int i=0;i<set->sample_count;i++)
    {
      tf::poseTFToMsg(tf::Pose(btQuaternion(set->samples[i].pose.v[2], 0, 0),
                               btVector3(set->samples[i].pose.v[0],
                                         set->samples[i].pose.v[1], 0)),
                      cloud_msg.poses[i]);

    }
    ros::Node::instance()->publish("particlecloud", cloud_msg);
  }

  if(resampled || force_publication)
  {
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for(int hyp_count = 0;
        hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
      {
        ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
        break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if(hyps[hyp_count].weight > max_weight)
      {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
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
      // Fill in the header
      p.header.frame_id = "map";
      p.header.stamp = laser_scan->header.stamp;
      // Copy in the pose
      p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      tf::quaternionTFToMsg(tf::Quaternion(hyps[max_weight_hyp].pf_pose_mean.v[2], 0.0, 0.0),
                            p.pose.pose.orientation);
      // Copy in the covariance, converting from 3-D to 6-D
      pf_sample_set_t* set = pf_->sets + pf_->current_set;
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
      //p.covariance[6*3+3] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
      p.pose.covariance[6*3+3] = set->cov.m[2][2];

      /*
         printf("cov:\n");
         for(int i=0; i<6; i++)
         {
         for(int j=0; j<6; j++)
         printf("%6.3f ", p.covariance[6*i+j]);
         puts("");
         }
       */

      ros::Node::instance()->publish("amcl_pose", p);
      last_published_pose = p;

      ROS_DEBUG("New pose: %6.3f %6.3f %6.3f",
               hyps[max_weight_hyp].pf_pose_mean.v[0],
               hyps[max_weight_hyp].pf_pose_mean.v[1],
               hyps[max_weight_hyp].pf_pose_mean.v[2]);

      // subtracting base to odom from map to base and send map to odom instead
      tf::Stamped<tf::Pose> odom_to_map;
      try
      {
        tf::Transform tmp_tf(tf::Quaternion(hyps[max_weight_hyp].pf_pose_mean.v[2],
                                            0, 0),
                             tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
                                         hyps[max_weight_hyp].pf_pose_mean.v[1],
                                         0.0));
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                              laser_scan->header.stamp,
                                              "base_footprint");
        this->tf_->transformPose(odom_frame_id_,
                                 tmp_tf_stamped,
                                 odom_to_map);
      }
      catch(tf::TransformException)
      {
        ROS_DEBUG("Failed to subtract base to odom transform");
        pf_mutex_.unlock();
        return;
      }

      latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                 tf::Point(odom_to_map.getOrigin()));
      latest_tf_valid_ = true;

      // We want to send a transform that is good up until a
      // tolerance time so that odom can be used
      ros::Time transform_expiration = (laser_scan->header.stamp +
                                        transform_tolerance_);
      tf::Stamped<tf::Transform> tmp_tf_stamped(latest_tf_.inverse(),
                                                transform_expiration,
                                                odom_frame_id_, "map");
      this->tfb_->sendTransform(tmp_tf_stamped);
    }
    else
    {
      ROS_ERROR("No pose!");
    }
  }
  else if(latest_tf_valid_)
  {
    // Nothing changed, so we'll just republish the last transform, to keep
    // everybody happy.
    ros::Time transform_expiration = (laser_scan->header.stamp +
                                      transform_tolerance_);
    tf::Stamped<tf::Transform> tmp_tf_stamped(latest_tf_.inverse(),
                                              transform_expiration,
                                              odom_frame_id_, "map");
    this->tfb_->sendTransform(tmp_tf_stamped);


    // Is it time to save our last pose to the param server
    ros::Time now = ros::Time::now();
    if((save_pose_period.toSec() > 0.0) &&
       (now - save_pose_last_time) >= save_pose_period)
    {
      // We need to apply the last transform to the latest odom pose to get
      // the latest map pose to store.  We'll take the covariance from
      // last_published_pose.
      tf::Pose map_pose = latest_tf_.inverse() * odom_pose;
      double yaw,pitch,roll;
      map_pose.getBasis().getEulerZYX(yaw, pitch, roll);

      ros::Node::instance()->setParam("~initial_pose_x", map_pose.getOrigin().x());
      ros::Node::instance()->setParam("~initial_pose_y", map_pose.getOrigin().y());
      ros::Node::instance()->setParam("~initial_pose_a", yaw);
      ros::Node::instance()->setParam("~initial_cov_xx", 
                                      last_published_pose.pose.covariance[6*0+0]);
      ros::Node::instance()->setParam("~initial_cov_yy", 
                                      last_published_pose.pose.covariance[6*1+1]);
      ros::Node::instance()->setParam("~initial_cov_aa", 
                                      last_published_pose.pose.covariance[6*3+3]);
      save_pose_last_time = now;
    }
  }

  pf_mutex_.unlock();
}

double
AmclNode::getYaw(tf::Pose& t)
{
  double yaw, pitch, roll;
  btMatrix3x3 mat = t.getBasis();
  mat.getEulerZYX(yaw,pitch,roll);
  return yaw;
}

void
AmclNode::initialPoseReceived()
{
  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::Stamped<tf::Transform> tx_odom;
  try
  {
    tf_->lookupTransform(base_frame_id_, ros::Time::now(),
                         base_frame_id_, initial_pose_.header.stamp,
                         "map", tx_odom);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
    tx_odom.setIdentity();
  }

  tf::Pose pose_old, pose_new;
  tf::poseMsgToTF(initial_pose_.pose.pose, pose_old);
  pose_new = tx_odom.inverse() * pose_old;

  ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
           ros::Time::now().toSec(),
           pose_new.getOrigin().x(),
           pose_new.getOrigin().y(),
           getYaw(pose_new));
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
      pf_init_pose_cov.m[i][j] = initial_pose_.pose.covariance[6*i+j];
    }
  }
  pf_init_pose_cov.m[2][2] = initial_pose_.pose.covariance[6*3+3];

  pf_mutex_.lock();
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;
  pf_mutex_.unlock();
}
