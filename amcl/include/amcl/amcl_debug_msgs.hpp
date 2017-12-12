#ifndef AMCL_DEBUG_HPP_
#define AMCL_DEBUG_HPP_

#ifdef BUILD_DEBUG
//
// #include <algorithm>
// #include <vector>
// #include <map>
#include <cmath>

// #include <boost/bind.hpp>
// #include <boost/thread/mutex.hpp>
//
// // Signal handling
// #include <signal.h>
//
#include "map/map.h"
#include "pf/pf.h"
// #include "sensors/amcl_odom.h"
// #include "sensors/amcl_laser.h"
//
// #include "ros/assert.h"

// roscpp
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"

// For debugging particles
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigenvalues>
#include <atomic>
#include <vector>

namespace amcl
{

class AmclDebug
{
public:
    AmclDebug(ros::NodeHandle* _nh, pf_t** _pf)
    : nh(_nh), pf(_pf)
    {
        marker_pub_ = nh->advertise<visualization_msgs::Marker>("clusters_coloured", 2, true);
        pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>("amcl_pose_debug", 2, true);
        heavy_particles_cloud_pub_ = nh->advertise<geometry_msgs::PoseArray>("heavy_particlescloud", 2, true);
        likelihood_field_pub_ = nh->advertise<nav_msgs::OccupancyGrid>("amcl_likelihood_field", 2, true);
        pose_scan_pub_ = nh->advertise<sensor_msgs::LaserScan>("amcl_pose_scan", 2, true);
        //tf_lis_ = tf::TransformListener();
        seq = 1;
    }
    void importParams(std::string _global_frame_id, std::string _laser_base_frame_id, ros::Duration _save_pose_period)
    {
        global_frame_id = _global_frame_id;
        laser_base_frame_id = _laser_base_frame_id;
        lifetime = _save_pose_period;
    }
    void publish_pose(const geometry_msgs::PoseWithCovarianceStamped& p) const;
    void publish_cov(const geometry_msgs::PoseWithCovariance& p, std_msgs::Header* h = NULL) const;
    void publish_cov(const visualization_msgs::Marker& cluster_msg, pf_matrix_t *cov_ptr, bool clear) const;
    void publish_scan_at_pose(sensor_msgs::LaserScan laser_scan, std::vector<double>measure_probs = std::vector<double>());

    void publish_cluster( size_t id, const double& weight, pf_vector_t *mean, pf_matrix_t *cov ) const;
    void publish_particles_compare(pf_sample_t* prev_samples, pf_sample_set_t* curr_set) const;

    void publish_clusters_cloud(pf_sample_set_t* set = NULL) const;

    void publish_likelihood_field(map_t* map);


    static void outputMap(map_t *map, std::string name = "map.bmp");
    static void outputLikelihoodField(map_t *map, std::string name = "map.bmp");
private:
    pf_sample_set_t* curr_set() const {return (*pf)->sets + (*pf)->current_set;}

    typedef Eigen::Matrix<double, 6, 6> Covariance3D;

    bool fillCovMsg(const Covariance3D& cov, visualization_msgs::Marker& msg) const;

    ros::NodeHandle* nh;
    pf_t **pf;

    ros::Publisher marker_pub_, pose_pub_, heavy_particles_cloud_pub_, likelihood_field_pub_, pose_scan_pub_ ;
    tf::TransformBroadcaster tf_br_;
    tf::TransformListener tf_lis_;
    std::string global_frame_id, laser_base_frame_id;
    ros::Duration lifetime;
    std::atomic<uint32_t> seq;
};






} // namespace


#endif // BUILD_DEBUG
#endif // AMCL_DEBUG_HPP_
