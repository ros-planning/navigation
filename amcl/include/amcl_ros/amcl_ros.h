#include <amcl.h>
using namespace amcl;
class AmclRosNode: public AmclNode
{
    public:
        AmclRosNode();
        ~AmclRosNode();
        /**
         * @brief Uses TF and LaserScan messages from bag file to drive AMCL instead
         */
        // void runFromBag(const std::string &in_bag_fn);
        void savePoseToServer();
        void deletePoseFromServer();

    private:
        void initializeParams();
        // Callbacks
        bool globalLocalizationCallback(std_srvs::Empty::Request& req,
                                        std_srvs::Empty::Response& res);
        bool nomotionUpdateCallback(std_srvs::Empty::Request& req,
                                        std_srvs::Empty::Response& res);
        bool setMapCallback(nav_msgs::SetMap::Request& req,
                            nav_msgs::SetMap::Response& res);

        void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
        void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
        void initialPosesReceived(const move_base_msgs::PoseWithCovarianceStampedArrayConstPtr& msg);
        bool handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);

        void updatePoseFromServer();
        bool checkPoseParamOnServer();

        void publishAmclReadySignal(bool signal);

        void publishStatusOnNewWorkArea(bool mapUpdated, bool initialPoseUpdated);

        //callback for action server
        void executeInitialPoseCB(const move_base_msgs::SetInitialPoseGoalConstPtr &goal);

        void reconfigureCB(amcl::AMCLConfig &config, uint32_t level);

        void checkLaserReceived(const ros::TimerEvent& event);

        void requestMap();

        // Helper to get odometric pose from transform system
         bool getOdomPose(tf::Stamped<tf::Pose>& pose,
                     double& x, double& y, double& yaw,
                     const ros::Time& t, const std::string& f);

        dynamic_reconfigure::Server<amcl::AMCLConfig> *dsrv_;

        ros::Timer check_laser_timer_;

        amcl::AMCLConfig default_config_;
        bool first_reconfigure_call_;

        // TODO: check if we need below two variables?
        ros::Duration cloud_pub_interval;
        // ros::Time last_cloud_pub_time;

        srs::MasterTimingDataRecorder tdr_;

        // Nomotion update control
	    bool force_update_;  // used to temporarily let amcl update samples even when no motion occurs...

        // set inital pose using action
        actionlib::SimpleActionServer<move_base_msgs::SetInitialPoseAction> set_inital_pose_action_;
        move_base_msgs::SetInitialPoseResult set_initial_pose_action_result_;

        message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
        tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
 
        ros::Time last_laser_received_ts_;
        ros::Duration laser_check_interval_;
        
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Publisher pose_pub_; //regular pose pub
        ros::Publisher analytics_pub_; //pub to send amcl_analytics msg
        ros::Publisher data_pub_; //pub to send amcl_data msg
        ros::Publisher particlecloud_pub_;
        ros::Publisher invalid_pose_percent_pub_;
        ros::Publisher ready_pub_;
        ros::Publisher status_on_new_work_area_pub_;
        ros::ServiceServer global_loc_srv_;
        ros::ServiceServer nomotion_update_srv_; //to let amcl update samples without requiring motion
        ros::ServiceServer set_map_srv_;
        ros::Subscriber initial_pose_sub_old_;
        ros::Subscriber map_sub_;

        ros::Publisher brainstem_driver_pose_pub_;
        ros::Subscriber initial_pose_sub_;
        ros::Subscriber initial_poses_sub_;
        
        tf::TransformBroadcaster* tfb_;

        // Use a child class to get access to tf2::Buffer class inside of tf_
        struct TransformListenerWrapper : public tf::TransformListener
        {
            inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
        };

        TransformListenerWrapper* tf_;

        bool sent_first_transform_;

        tf::Transform latest_tf_;
        bool latest_tf_valid_;
        //paramater to store latest odom pose
        tf::Stamped<tf::Pose> latest_odom_pose_;
        bool first_map_received_;
        boost::recursive_mutex configuration_mutex_;
};