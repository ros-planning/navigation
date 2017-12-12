#ifndef POSE_EXTRAPOLATE_HPP_
#define POSE_EXTRAPOLATE_HPP_
#include <thread>
#include <vector>
#include <fstream>
#include <mutex>
#include <condition_variable>
#include <ostream>
#include <chrono>
#include <cmath>
#include <atomic>

// mrpt
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

// roscpp
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/tf.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

#ifdef BUILD_DEBUG
    //#define PRINT_DEBUG_MSGS_POSE_EXTRAPOLATOR
#endif

class PoseExtrapolator
{
public:
    PoseExtrapolator(std::string localization_src) : src_name(localization_src), new_est(false), the_thread()
    {
        ros::NodeHandle n;
        pose_pub_ = n.advertise<geometry_msgs::PoseStamped>(src_name+"_pose_extrapolated", 100, true);
        pose_cov_pub_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(src_name+"_pose_cov_extrapolated", 100, true);
        odom_pub_ = n.advertise<nav_msgs::Odometry>(src_name+"_odom", 100, true);
        initPoses();
    }
    void pause();
    void shutdown();
    void cleanup();
    ~PoseExtrapolator();
    void newOdom(nav_msgs::Odometry odom);
    void newEstimate(geometry_msgs::PoseWithCovarianceStamped pc);
    void newInit(geometry_msgs::PoseWithCovarianceStamped pc);

protected:
    ros::Publisher pose_pub_, odom_pub_, pose_cov_pub_;
    const std::string src_name;

    bool finished = false;
    bool paused = false;
    typedef mrpt::poses::CPose3D Pose3D;
    typedef mrpt::math::CMatrixDouble66 Cov3D;
    typedef mrpt::poses::CPose3DPDFGaussian Pose3DWithCov;
    void initPoses();

    tf::TransformListener tf_listener;
    Pose3DWithCov last_odom_pose, curr_odom_pose;
    Pose3DWithCov last_est_odom, last_est, curr_est;

    ros::Time last_est_stamp, newest_est_stamp;
    size_t est_seq = 0;

    void startCSVWriter();
    bool started = false;

    std::string csv_name_str;
    std::fstream pose_csv;
    std::mutex csv_lock;

    std::atomic<bool> new_est;
    std::thread the_thread;
    std::mutex queue_lock, est_lock, odom_lock;
    bool odom_init;
    std::condition_variable queue_processing;
    std::vector<nav_msgs::Odometry> odom_queue;
    bool pushBackOdom(const nav_msgs::Odometry& odom);
    void threadMain();
    void purgeQueue(int& wait_count);
    void retroCorrect();
    bool updateCurrPoseWithOdomMsg(const nav_msgs::Odometry& odom, Pose3DWithCov& last_odom, Pose3DWithCov& pose);

    std::mutex backup_queue_lock;
    std::condition_variable queue_recyling;
    std::vector<nav_msgs::Odometry> backup_odom_queue;
    void queueRecycling();
    void recycleQueue();

    geometry_msgs::PoseStamped publishPose(ros::Time timestamp, const Pose3D& p, size_t num) const;
    void publishPose(ros::Time timestamp, const Pose3DWithCov& pc, size_t num) const;

    void writeSinglePose(std::string src, ros::Time timestamp, const geometry_msgs::Pose& p, size_t num);
    void writeSinglePose(std::string src, ros::Time timestamp, const Pose3DWithCov& pc, size_t num);

    template<typename valT>
    void writeSinglePoseToCSV(std::fstream& file, std::string src, ros::Time timestamp, valT x, valT y, valT yaw, size_t num)
    {
        std::lock_guard<std::mutex> lock(csv_lock);
        file << src << ", " << num << ", " << timestamp.toNSec() << ", " << std::flush;
        file << x << ", " << y << ", " << yaw << std::endl;
#ifdef PRINT_DEBUG_MSGS_POSE_EXTRAPOLATOR
        if (src != "odom")
            printf("[PoseExtrapolator]\t[Write to CSV] %s # %d @ t = %ld | %lf, %lf, yaw %lf\n", src.c_str(), num, timestamp.toNSec(), x, y, yaw);
#endif
    }


};

#endif
