#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <math.h>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// Signal handling
#include <signal.h>

#include "map.h"
#include "pf.h"
#include "amcl_odom.h"
#include "amcl_laser.h"

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "move_base_msgs/PoseWithCovarianceStampedArray.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"
#include "move_base_msgs/amcl_analytics.h"
#include "move_base_msgs/amcl_data.h"
#include "move_base_msgs/cluster.h"
#include "move_base_msgs/flatCov.h"
// For transform support
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include <tf2_ros/buffer.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer_interface.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include "amcl/AMCLConfig.h"

// Allows AMCL to run from bag file
#include <boost/foreach.hpp>

#include <srslib_timing/ScopedTimingSampleRecorder.hpp>
#include <srslib_timing/MasterTimingDataRecorder.hpp>

#include <actionlib/server/simple_action_server.h>

#include <move_base_msgs/SetInitialPoseAction.h>
#include <move_base_msgs/SetInitialPoseResult.h>
#include <srslib_framework/chuck/ChuckTopics.hpp>
#include <srslib_framework/AmclStatusOnNewWorkArea.h>

#define NEW_UNIFORM_SAMPLING 1
using namespace amcl;
typedef struct amcl_parameters
{
    bool use_map_topic_;
    bool first_map_only_;
    bool use_emulator_;
    //parameter for what odom to use
    std::string odom_frame_id_;

    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    ros::Duration gui_publish_period;
    ros::Duration save_pose_period;

  
    int max_beams_, min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
    //beam skip related params
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double laser_likelihood_max_dist_;
    odom_model_t odom_model_type_;  
    laser_model_t laser_model_type_;

    double pf_err_, pf_z_;
    
    double d_thresh_, a_thresh_;
    int resample_interval_;
    double laser_min_range_;
    double laser_max_range_;
    bool tf_broadcast_;
    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    // For slowing play-back when reading directly from a bag file
    ros::WallDuration bag_scan_period_;

    amcl_parameters():
    use_map_topic_(false),
    first_map_only_(false),
    use_emulator_(false),
    odom_frame_id_(std::string("odom")),
    base_frame_id_(std::string("base_link")),
    global_frame_id_(std::string("map")),
    gui_publish_period(ros::Duration(-1.0)),
    save_pose_period(ros::Duration(2.0)),
    max_beams_(30),
    min_particles_(100),
    max_particles_(5000),
    alpha1_(0.2),
    alpha2_(0.2),
    alpha3_(0.2),
    alpha4_(0.2),
    alpha5_(0.2),
    alpha_slow_(0.001),
    alpha_fast_(0.1),
    z_hit_(0.95),
    z_short_(0.1),
    z_max_(0.05),
    z_rand_(0.05),
    sigma_hit_(0.2),
    lambda_short_(0.1),
    do_beamskip_(false),
    beam_skip_distance_(0.5),
    beam_skip_threshold_(0.3),
    beam_skip_error_threshold_(0.9),
    laser_likelihood_max_dist_(0.2),
    odom_model_type_(ODOM_MODEL_DIFF),
    laser_model_type_(LASER_MODEL_LIKELIHOOD_FIELD),
    pf_err_(0.01), pf_z_(0.99),
    d_thresh_(0.2), a_thresh_(M_PI/6.0),
    resample_interval_(2),
    laser_min_range_(-1.0),
    laser_max_range_(-1.0),
    tf_broadcast_(true)
    {
      transform_tolerance_.fromSec(0.1);
      bag_scan_period_.fromSec(-1.0);
    }
} amcl_parameters;

typedef struct
{
  float percent_invalid_poses_;
  geometry_msgs::PoseArray particle_cloud_msg_;
  move_base_msgs::amcl_analytics analytics_data_;
  geometry_msgs::PoseWithCovarianceStamped pose_data_;
  move_base_msgs::amcl_data amcl_data_;
} amcl_analytics_data;

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;
  //num of Samples
  int numSamples;
  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
};

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
    virtual ~AmclNode();

    bool handleInitialPosesMessage(const move_base_msgs::PoseWithCovarianceStampedArray& msg);

    void handleLaserScanMessage(const sensor_msgs::LaserScanConstPtr& laser_scan);

    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);

    bool globalInitialization();

    void setInitialPose(double init_pose[3], double init_cov[3]);

    bool hasNewOdomToMapTf() { return hasNewOdomToMapTf_;};

    inline void setHasNewOdomToMapTf(bool hasNewOdomToMapTf) { hasNewOdomToMapTf_ = hasNewOdomToMapTf;};

    inline void setForceUpdateControl(bool force_update) { force_update_ = force_update;};

    inline map_t* getAmclMap() { return map_;};

    inline tf::StampedTransform getOdomToMapTf() {return odom_to_map_tf_;};

    inline tf::Stamped<tf::Pose> getLatestOdomPose() { return latest_odom_pose_;};

    inline tf::Transform getLatestTf() { return latest_tf_;};

    inline geometry_msgs::PoseWithCovarianceStamped getLastPublishedPose() { return last_published_pose;};

  private:
    int resample_count_;

    tf2_ros::Buffer tf_buffer_;

    //paramater to store latest odom pose
    tf::Stamped<tf::Pose> latest_odom_pose_;

    tf::StampedTransform odom_to_map_tf_;

    bool hasNewOdomToMapTf_;

    // Nomotion update control
	  bool force_update_;  // used to temporarily let amcl update samples even when no motion occurs...

    pf_vector_t pf_odom_pose_;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    tf::Transform latest_tf_;

    bool latest_tf_valid_;

    bool sent_first_transform_;

    double init_pose_[3];
    double init_cov_[3];

    map_t* map_;

    AMCLOdom* odom_;
    AMCLLaser* laser_;

    std::vector< AMCLLaser* > lasers_;
    std::vector< bool > lasers_update_;
    std::map< std::string, int > frame_to_laser_;

    std::vector<amcl_hyp_t> initial_poses_hyp_;

    // Particle filter
    pf_t *pf_;
    bool pf_init_;

    void updateHypothesis(const sensor_msgs::LaserScanConstPtr& laser_scan);

    bool applyInitialPose();

    void freeMapDependentMemory();

    map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg );

    double getYaw(tf::Pose& t);

            // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,
                double& x, double& y, double& yaw,
                const ros::Time& t, const std::string& f);

  protected:
    amcl_parameters amclParams_;

    amcl_analytics_data amclAnalyticsData_;

    // Pose-generating function used to uniformly distribute particles over
    // the map
    static pf_vector_t uniformPoseGenerator(void* arg);
#if NEW_UNIFORM_SAMPLING
    static std::vector<std::pair<int,int> > free_space_indices;
#endif
    void instantiateOdom();

    void instantiateLaser();

    void instantiateParticleFilter(pf_vector_t pf_init_pose_mean, pf_matrix_t pf_init_pose_cov);
    boost::recursive_mutex configuration_mutex_;
};