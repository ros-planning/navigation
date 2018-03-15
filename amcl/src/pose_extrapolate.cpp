#include "pose_extrapolate.hpp"

#ifdef BUILD_DEBUG
    #ifdef PRINT_DEBUG_MSGS_POSE_EXTRAPOLATOR
        std::mutex print_lock;
        #define DEBUG_INFO(...) do { std::lock_guard<std::mutex> lock(print_lock); printf("[PoseExtrapolator]------"); printf(__VA_ARGS__); } while (0)
    #else
        #define DEBUG_INFO(...) do { ROS_INFO("(DEBUG) "); ROS_INFO(__VA_ARGS__); } while (0)
    #endif
#else
    #define DEBUG_INFO(...) do {} while (0)
#endif

typedef mrpt::poses::CPose3D Pose3D;
typedef mrpt::math::CMatrixDouble66 Cov3D;
typedef mrpt::poses::CPose3DPDFGaussian Pose3DWithCov;

Pose3D makePose3D(double x, double y, double yaw)
{
    return Pose3D (x, y, 0.0, yaw, 0.0, 0.0);
}

Pose3D makePose3D(const geometry_msgs::Pose& p)
{
    Pose3D tf;
    Eigen::Quaterniond quat(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    tf.setRotationMatrix(quat.toRotationMatrix());
    tf.x() = p.position.x;
    tf.y() = p.position.y;
    tf.z() = p.position.z;
    return tf;
}


Pose3D makePose3D(const geometry_msgs::PoseWithCovariance& pc)
{
    return makePose3D(pc.pose);
}

Pose3D makePose3D(const geometry_msgs::PoseWithCovarianceStamped& pc)
{
    return makePose3D(pc.pose);
}

Pose3DWithCov makePose3DWithCov(const geometry_msgs::PoseWithCovariance& pc)
{
    double arr_cov[36];
    for (size_t idx = 0; idx < 36; ++idx) // 36: 6x6 covariance matrix
    {
        arr_cov[idx] = pc.covariance[idx];
    }
    Cov3D cov(arr_cov);
    return Pose3DWithCov(makePose3D(pc.pose), cov);
}

Pose3DWithCov makePose3DWithCov(const geometry_msgs::PoseWithCovarianceStamped& pc)
{
    return makePose3DWithCov(pc.pose);
}

Pose3D makePose3D(const nav_msgs::Odometry& omsg)
{
    return makePose3D(omsg.pose.pose);
}

Pose3DWithCov makePose3DWithCov(const nav_msgs::Odometry& odom)
{
    return makePose3DWithCov(odom.pose);
}

Pose3D makePose3D(const tf::Stamped<tf::Pose>& tfp)
{
    Pose3D tf;
    double roll, pitch, yaw;
    tfp.getBasis().getEulerYPR(yaw, pitch, roll);
    assert(pitch == 0.0 && roll == 0.0);
    return makePose3D(tfp.getOrigin().x(), tfp.getOrigin().y(), yaw);
}

template <typename objT>
void SortQueue(std::vector< objT >& vec )
{
    std::sort(
        vec.begin(),
        vec.end(),
        [](const objT& a, const objT& b)
        {
            return a.header.stamp < b.header.stamp;
        });
}

void PoseExtrapolator::initPoses()
{
    last_odom_pose = Pose3DWithCov(makePose3D(0,0,0));
    last_est_odom = last_odom_pose;
    curr_odom_pose = last_odom_pose;
    last_est_odom = last_odom_pose;
    last_est_stamp = ros::Time::now();
}

void PoseExtrapolator::pause()
{
    paused = true;
}

void PoseExtrapolator::cleanup()
{
    if (! started) return;

    queue_processing.notify_all();

    std::lock_guard<std::mutex> lock(queue_lock);
    DEBUG_INFO("[cleanup] Cleaning up...\n");

    if (odom_queue.size())
    {
        {
            std::lock_guard<std::mutex> lock(csv_lock);
            pose_csv << "---cleaning up---" << std::endl;
        }
        int count = odom_queue.size();
        DEBUG_INFO("[cleanup] Writing remaining %d poses using odometry...\n", count);
        {
            std::lock_guard<std::mutex> lock2(odom_lock);
            for (auto oqitr = odom_queue.begin(); oqitr != odom_queue.end(); ++oqitr)
            {
                updateCurrPoseWithOdomMsg(*oqitr, last_odom_pose, curr_odom_pose);
                writeSinglePose("odom", oqitr->header.stamp, curr_odom_pose, oqitr->header.seq);
            }
        }

        odom_queue.clear();
    }
}

void PoseExtrapolator::shutdown()
{
    finished = true;
    cleanup();
    if (started && the_thread.joinable())
    {
        DEBUG_INFO("[shutdown] Waiting to join thread in PoseExtrapolator to finished...\n");
        the_thread.join();
    }
    started = false;
    DEBUG_INFO("[shutdown] PoseExtrapolator has been shutdown\n");
}

PoseExtrapolator::~PoseExtrapolator()
{
    shutdown();
    DEBUG_INFO("[~] PoseExtrapolator is being destructed\n");
}

void PoseExtrapolator::newOdom(nav_msgs::Odometry odom)
{
    if (! odom_init)
    {
        std::lock_guard<std::mutex> lock(odom_lock);
        last_odom_pose = makePose3DWithCov(odom);
        odom_init = true;
    }
    {
        std::lock_guard<std::mutex> lock(est_lock);
        if (odom.header.stamp > newest_est_stamp && est_seq)
        {
            std::lock_guard<std::mutex> lock(odom_lock);
            updateCurrPoseWithOdomMsg(odom,  last_odom_pose, curr_odom_pose);
            writeSinglePose("odom", odom.header.stamp, curr_odom_pose, odom.header.seq);
        }
    }

    pushBackOdom(odom);
}

bool operator!=(const nav_msgs::Odometry& odom1, const nav_msgs::Odometry& odom2)
{
    if (odom1.pose.pose.position.x != odom2.pose.pose.position.x) return true;
    if (odom1.pose.pose.position.y != odom2.pose.pose.position.y) return true;
    if (odom1.pose.pose.position.z != odom2.pose.pose.position.z) return true;
    if (odom1.pose.pose.orientation.x != odom2.pose.pose.orientation.x) return true;
    if (odom1.pose.pose.orientation.y != odom2.pose.pose.orientation.y) return true;
    if (odom1.pose.pose.orientation.z != odom2.pose.pose.orientation.z) return true;
    if (odom1.pose.pose.orientation.w != odom2.pose.pose.orientation.w) return true;
    return false;
}

bool PoseExtrapolator::pushBackOdom(const nav_msgs::Odometry& odom)
{
    std::lock_guard<std::mutex> lock(queue_lock);
    {
        std::lock_guard<std::mutex> lock(est_lock);
        if (odom_queue.size() && odom_queue.back().header.stamp < newest_est_stamp)
        {
            recycleQueue();
        }
        if (odom.header.stamp < newest_est_stamp)
        {
            if (odom.header.stamp > last_est_stamp)
            {
                std::lock_guard<std::mutex> lock(odom_lock);
                updateCurrPoseWithOdomMsg(odom,  last_odom_pose, curr_odom_pose);
                writeSinglePose("odom_extra", odom.header.stamp, curr_odom_pose, odom.header.seq);
            }
            return false;
        }
    }

    if ( (! odom_queue.size()) || (odom != odom_queue.back()) )
    {
        odom_queue.push_back(odom);
        return true;
    }

    return false;

// #ifdef BUILD_DEBUG
//     else
//     {
//         DEBUG_INFO("Skipped zero movement odom # %ld @ t = %ld; == back? %c\n", odom.header.seq, odom.header.stamp.toNSec(), (odom != odom_queue.back() ? 'N':'Y'));
//     }
// #endif

}

void PoseExtrapolator::recycleQueue()
{
    std::lock_guard<std::mutex> lock(backup_queue_lock);
    odom_queue.swap(backup_odom_queue);
    queue_recyling.notify_all();
}

void PoseExtrapolator::newEstimate(geometry_msgs::PoseWithCovarianceStamped pc)
{
    if (! est_seq)
    {
        newInit(pc);
        startCSVWriter();
    }
    else writeSinglePose(src_name, pc.header.stamp, pc.pose.pose, est_seq);

    std::lock_guard<std::mutex> lock(est_lock);
    paused = false;
    new_est = true;
    ++est_seq;
    last_est_stamp = newest_est_stamp;
    newest_est_stamp = pc.header.stamp;
    curr_est = makePose3DWithCov(pc);
    queue_processing.notify_one();
}

void PoseExtrapolator::newInit(geometry_msgs::PoseWithCovarianceStamped pc)
{
    est_seq = 0;
    writeSinglePose("init", pc.header.stamp, pc.pose.pose, est_seq);

    {
        std::lock_guard<std::mutex> lock(est_lock);
        last_est = makePose3DWithCov(pc);
        {
            std::lock_guard<std::mutex> lock(odom_lock);
            curr_odom_pose = makePose3DWithCov(pc);
            curr_est = curr_odom_pose;
            odom_init = false;
        }
        last_est_stamp = newest_est_stamp;
        newest_est_stamp = pc.header.stamp;
    }

    std::lock_guard<std::mutex> lock(queue_lock);
    odom_queue.clear();
}

void PoseExtrapolator::startCSVWriter()
{
    {
        std::lock_guard<std::mutex> lock(csv_lock);
        if (started) return;
        started = true;
        std::ostringstream csv_name("");
        csv_name << src_name << "_extrapolated_pose_" << last_est_stamp.toNSec() << ".csv";
        csv_name_str = csv_name.str();
        std::cout << "Will write to " << csv_name_str << std::endl;
        pose_csv.open(csv_name_str, std::fstream::out | std::fstream::trunc);
        if (! pose_csv.is_open()) throw std::runtime_error("Unable to open a csv to write to!");
        pose_csv << "Est. By, Seq, Stamp, x, y, yaw" << std::endl;
    }

    the_thread = std::thread(&PoseExtrapolator::threadMain, this);
}

void PoseExtrapolator::threadMain()
{
    int wait_count = 0;

    std::thread recycling_thread(&PoseExtrapolator::queueRecycling, this);
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~ [Retro Thread] started" << std::endl;

    while (! finished)
    {
        purgeQueue(wait_count);
        queue_recyling.notify_all();
    }
    if (started) DEBUG_INFO("[Retro Thread] Poses written to %s\n", csv_name_str.c_str());
    queue_recyling.notify_all();
    recycling_thread.join();
    std::lock_guard<std::mutex> lock(csv_lock);
    DEBUG_INFO("Closing csv...\n");
    pose_csv.close();
    DEBUG_INFO("[Retro Thread] finished\n");
}

void PoseExtrapolator::purgeQueue(int& wait_count)
{
    std::unique_lock<std::mutex> lock(queue_lock);
    SortQueue(odom_queue);
    DEBUG_INFO("[Retro Thread] Waiting...%d\n", ++wait_count);
    if ( ! queue_processing.wait_for(lock, std::chrono::seconds(2),
        [this, &wait_count]
        {
            bool continue_waiting = ! odom_queue.size();// && (odom_queue.back().header.stamp >= newest_est_stamp);
            if (! odom_queue.size()) DEBUG_INFO("[Retro Thread] Spurious waiting skipped %d - no new odom\n", wait_count);

            std::lock_guard<std::mutex> lock(est_lock);
            if (! new_est.load() || est_seq <= 1) return false;
            if (paused) return false;
            paused = (newest_est_stamp == last_est_stamp);

            continue_waiting |= paused;
            //if (newest_est_stamp == last_est_stamp) DEBUG_INFO("[Retro Thread] Spurious waiting skipped %ld - no new estimate\n", wait_count);
            return ! continue_waiting;
        }
        )
    ) return;

    DEBUG_INFO("[Retro Thread] ~~~~~~~~~~~~~~~~~ Finished waiting %d -------- %s seq = %lu @ t = %ld %c last odom in queue seq# %d @ t = %ld with queue size = %lu\n",
        wait_count, src_name.c_str(), (est_seq-1), newest_est_stamp.toNSec(), (newest_est_stamp > odom_queue.back().header.stamp ?'>':'<'), odom_queue.back().header.seq, odom_queue.back().header.stamp.toNSec(), odom_queue.size());
    retroCorrect();
    wait_count = 0;
}

void PoseExtrapolator::queueRecycling()
{
    while (! finished)
    {
        std::unique_lock<std::mutex> lock(backup_queue_lock);
        queue_recyling.wait( lock, [this] { return backup_odom_queue.size() || finished; } );
        DEBUG_INFO("[Recycling Thread] Clearing vec of size %lu...\n", backup_odom_queue.size());
        backup_odom_queue.clear();
        DEBUG_INFO("[Recycling Thread] ... Clearing done.\n");
    }
    if (started) DEBUG_INFO("[Recycling Thread] finished\n");
}

void PoseExtrapolator::retroCorrect()
{
    bool has_new_est = new_est.load();
    if (! has_new_est || ! new_est.compare_exchange_strong(has_new_est, false)) return;
    std::vector<nav_msgs::Odometry>::const_iterator oqitr = odom_queue.begin();
    decltype(oqitr) start_itr, erase_to_itr;
    while ( oqitr != odom_queue.end() && (oqitr->header.stamp < last_est_stamp) && new_est.compare_exchange_strong(has_new_est, false))
    {
        ++oqitr;
    }

#ifdef BUILD_DEBUG
    int count;
    if (oqitr == odom_queue.end())
    {
        DEBUG_INFO("[Retro Thread] %lu odom stamp < last est stamp t = %ld\n", odom_queue.size(), last_est_stamp.toNSec());
    }
#endif

    {
        std::lock_guard<std::mutex> lock(est_lock);
        curr_odom_pose = curr_est;
    }

    if (oqitr == odom_queue.end())
    {
        recycleQueue();
        return;
    }

    start_itr = oqitr;

    if (oqitr > odom_queue.begin())
    {
#ifdef BUILD_DEBUG
        count = oqitr - odom_queue.begin();
        DEBUG_INFO("[Retro Thread] Skipped %d odoms that were older than %s pose up to t = %ld\n", count, src_name.c_str(), oqitr->header.stamp.toNSec());
#endif
        if (oqitr == odom_queue.end()) return;
        last_est_odom = makePose3DWithCov(*(oqitr-1));
        erase_to_itr = oqitr;
    }

    Pose3DWithCov corrected_pose;
    corrected_pose = last_est;
    for (; oqitr != odom_queue.end() && (oqitr->header.stamp < newest_est_stamp) && new_est.compare_exchange_strong(has_new_est, false);
        ++oqitr)
    {
        updateCurrPoseWithOdomMsg(*oqitr, last_est_odom, corrected_pose);
        //writeSinglePose(src_name+"_correcting...", oqitr->header.stamp, corrected_pose, oqitr->header.seq);
    }

    if (oqitr != start_itr) last_odom_pose = last_est_odom;

#ifdef BUILD_DEBUG
    count = start_itr - odom_queue.begin();
    if (count) DEBUG_INFO("[Retro Thread] Latest pose retrocorrected up to %ld with %d odom up to (seq %u) %ld\n", newest_est_stamp.toNSec(), count, (oqitr-1)->header.seq, (oqitr-1)->header.stamp.toNSec());
#endif

    // if (count) writeSinglePose(src_name+"_corrected", (oqitr-1)->header.stamp, last_est_odom_odom_odom, (oqitr-1)->header.seq);
    erase_to_itr = oqitr;
    assert(erase_to_itr <= odom_queue.end());
    last_est = curr_est;

    if (erase_to_itr == odom_queue.end())
    {
        recycleQueue();
        DEBUG_INFO("[Retro Thread] Clearing entire Q\n");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(odom_lock);
#ifdef BUILD_DEBUG
        count = odom_queue.end() - erase_to_itr;
        DEBUG_INFO("[Retro Thread] Latest pose to be updated by %s with %d odoms concatenated AFTER correction\n", src_name.c_str(), count);
        Pose3D curr3d = curr_odom_pose.getPoseMean();
        DEBUG_INFO("[Retro Thread] Concatenating start pose: %lf, %lf, yaw %lf\n", curr3d.x(), curr3d.y(), curr3d.yaw());
#endif
        // Get curr_odom_pose up-to-date wth odom
        for (; oqitr != odom_queue.end() && new_est.compare_exchange_strong(has_new_est, false); ++oqitr)
        {
            if (! updateCurrPoseWithOdomMsg(*oqitr, last_odom_pose, curr_odom_pose)) continue;
            writeSinglePose(src_name+"_extra", oqitr->header.stamp, curr_odom_pose, oqitr->header.seq);
        }
    }

#ifdef BUILD_DEBUG
    count = erase_to_itr - odom_queue.begin();
    DEBUG_INFO("[Retro Thread] Erasing first %d in odom Q\n", count);
#endif

    odom_queue.erase(odom_queue.begin(), erase_to_itr);
}

bool PoseExtrapolator::updateCurrPoseWithOdomMsg(const nav_msgs::Odometry& omsg, Pose3DWithCov& last_odom ,Pose3DWithCov& pose)
{
    Pose3DWithCov this_odom_pose_cov = makePose3DWithCov(omsg);
    Pose3DWithCov delta_pose_cov = this_odom_pose_cov - last_odom;

    Pose3D delta_pose = delta_pose_cov.getPoseMean();
    if (delta_pose.yaw() || delta_pose.x() || delta_pose.y())
    {
        last_odom = this_odom_pose_cov;
        pose = pose + delta_pose_cov;
        return true;
    }
    return false;

#ifdef BUILD_DEBUG
    // if (delta_pose.yaw() || delta_pose.x() || delta_pose.y())
    // {
    //     Pose3D this_odom_pose = this_odom_pose_cov.getPoseMean();
    //     DEBUG_INFO("\todom # %d @ t = %ld | %lf, %lf, yaw %lf\n", omsg.header.seq, omsg.header.stamp.toNSec(), delta_pose.x(), delta_pose.y(), delta_pose.yaw());
    //     DEBUG_INFO("[PoseExtrapolator] %s # %d @ t = %ld | %lf, %lf, yaw %lf\n", "odom", omsg.header.seq, omsg.header.stamp.toNSec(), this_odom_pose.x(), this_odom_pose.y(), this_odom_pose.yaw());
    // }
#endif
}

geometry_msgs::PoseStamped PoseExtrapolator::publishPose(ros::Time timestamp, const Pose3D& p, size_t num) const
{
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = timestamp;
    msg.header.seq = num;
    msg.pose.position.x = p.x();
    msg.pose.position.y = p.y();
    msg.pose.position.z = 0.0;
    msg.pose.orientation.z = sin(p.yaw()/2.0);
    msg.pose.orientation.w = cos(p.yaw()/2.0);
    pose_pub_.publish(msg);
    return msg;
}

void PoseExtrapolator::publishPose(ros::Time timestamp, const Pose3DWithCov& pc, size_t num) const
{
    geometry_msgs::PoseWithCovarianceStamped pc_msg;
    geometry_msgs::PoseStamped p_msg;
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "map";
    odom_msg.header.stamp = timestamp;
    odom_msg.header.seq = num;
    odom_msg.child_frame_id = "base_link"; //FIXME

    Pose3D p = pc.getPoseMean();
    p_msg = publishPose(timestamp, p, num); // publish as PoseStamped

    odom_msg.pose.pose = p_msg.pose;
    Cov3D cov = pc.getCovariance();
    for (size_t idx = 0; idx < 36; ++idx) // 36: 6x6 covariance matrix
    {
        odom_msg.pose.covariance[idx] = cov.data()[idx];
    }
    odom_pub_.publish(odom_msg); // also publish as Odometry

    pc_msg.header = p_msg.header;
    pc_msg.pose.pose = p_msg.pose;
    pc_msg.pose.covariance = odom_msg.pose.covariance;
    pose_cov_pub_.publish(pc_msg);
}

void PoseExtrapolator::writeSinglePose(std::string src, ros::Time timestamp, const Pose3DWithCov& pc, size_t num)
{
    Pose3D p = pc.getPoseMean();
    writeSinglePoseToCSV(pose_csv, src, timestamp, p.x(), p.y(), p.yaw(), num);
    publishPose(timestamp, pc, num);
}

void PoseExtrapolator::writeSinglePose(std::string src, ros::Time timestamp, const geometry_msgs::Pose& p, size_t num)
{
    Eigen::Quaterniond quat(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    Eigen::AngleAxisd angle_axis(quat);
    Eigen::Vector3d unit_z(0,0,1);
    double yaw = angle_axis.axis().dot(unit_z) * angle_axis.angle();
    writeSinglePoseToCSV(pose_csv, src, timestamp, p.position.x, p.position.y, yaw, num);

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = timestamp;
    msg.header.seq = num;
    msg.pose = p;
    pose_pub_.publish(msg);
}
