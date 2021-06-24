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
typedef struct 
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
    ros::Time save_pose_last_time;
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
    
    pf_vector_t pf_odom_pose_;
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

    //TODO: add constructor with default values
} AmclParameters;

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
angle_diff(double a, double b);

class AmclNode
{
  public:
    AmclNode();
    ~AmclNode();

    bool handleInitialPosesMessage(const move_base_msgs::PoseWithCovarianceStampedArray& msg);
 
  private:
    int resample_count_;

    tf2_ros::Buffer tf_buffer_;

//     int process();

  protected:
    AmclParameters amclParams_;

    bool applyInitialPose();
    // Pose-generating function used to uniformly distribute particles over
    // the map
    static pf_vector_t uniformPoseGenerator(void* arg);
#if NEW_UNIFORM_SAMPLING
    static std::vector<std::pair<int,int> > free_space_indices;
#endif
    void setInitialPose(double init_pose[3], double init_cov[3]);
    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    void freeMapDependentMemory();
    map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg );
    
    double getYaw(tf::Pose& t);
    
    geometry_msgs::PoseWithCovarianceStamped last_published_pose;
    
    map_t* map_;
    
//     // TODO: If we need below three variables??
//     char* mapdata;
//     int sx, sy;
//     double resolution;

    double init_pose_[3];
    double init_cov_[3];
       
    std::vector< AMCLLaser* > lasers_;
    std::vector< bool > lasers_update_;
    std::map< std::string, int > frame_to_laser_;

    // Particle filter
    pf_t *pf_;
    bool pf_init_;
    
    AMCLOdom* odom_;
    AMCLLaser* laser_;

    std::vector<amcl_hyp_t> initial_poses_hyp_;

    boost::recursive_mutex configuration_mutex_;
};