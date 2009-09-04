/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include <robot_pose_ekf/odom_estimation_node.h>


using namespace MatrixWrapper;
using namespace std;
using namespace ros;
using namespace tf;


static const double EPS = 1e-5;
static const string publish_name = "odom_combined";


//#define __EKF_DEBUG_FILE__

namespace estimation
{
  // constructor
  OdomEstimationNode::OdomEstimationNode()
    : vel_desi_(2),
      vel_active_(false),
      odom_active_(false),
      imu_active_(false),
      vo_active_(false),
      odom_initializing_(false),
      imu_initializing_(false),
      vo_initializing_(false),
      odom_covariance_(6),
      imu_covariance_(3),
      vo_covariance_(6)
  {
    // paramters
    node_.getParam("~sensor_timeout", timeout_, 1.0);
    node_.getParam("~odom_used", odom_used_, true);
    node_.getParam("~imu_used",  imu_used_, true);
    node_.getParam("~vo_used",   vo_used_, true);
    double freq;
    node_.getParam("~freq", freq, 30.0);

    timer_ = node_.createTimer(ros::Duration(1.0/max(freq,1.0)), &OdomEstimationNode::spin, this);

    // advertise our estimation
    pose_pub_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>("~"+publish_name, 10);

    // initialize
    filter_stamp_ = Time::now();

    // fiexed transform between camera frame and vo frame
    vo_camera_ = Transform(Quaternion(M_PI/2.0, -M_PI/2,0), Vector3(0,0,0));

    // subscribe to messages
    cmd_vel_sub_ = node_.subscribe("cmd_vel", 10, &OdomEstimationNode::velCallback, this);

    // subscribe to odom messages
    if (odom_used_){
      ROS_DEBUG("Odom sensor can be used");
      odom_sub_ = node_.subscribe("odom", 10, &OdomEstimationNode::odomCallback, this);
    }
    else ROS_DEBUG("Odom sensor will NOT be used");

    // subscribe to imu messages
    if (imu_used_){
      ROS_DEBUG("Imu sensor can be used");
      imu_sub_ = node_.subscribe("imu_data", 10,  &OdomEstimationNode::imuCallback, this);
    }
    else ROS_DEBUG("Imu sensor will NOT be used");

    // subscribe to vo messages
    if (vo_used_){
      ROS_DEBUG("VO sensor can be used");
      vo_sub_ = node_.subscribe("vo", 10, &OdomEstimationNode::voCallback, this);
    }
    else ROS_DEBUG("VO sensor will NOT be used");


#ifdef __EKF_DEBUG_FILE__
    // open files for debugging
    odom_file_.open("/tmp/odom_file.txt");
    imu_file_.open("/tmp/imu_file.txt");
    vo_file_.open("/tmp/vo_file.txt");
    corr_file_.open("/tmp/corr_file.txt");
    time_file_.open("/tmp/time_file.txt");
    extra_file_.open("/tmp/extra_file.txt");
#endif
  };




  // destructor
  OdomEstimationNode::~OdomEstimationNode(){

#ifdef __EKF_DEBUG_FILE__
    // close files for debugging
    odom_file_.close();
    imu_file_.close();
    vo_file_.close();
    corr_file_.close();
    time_file_.close();
    extra_file_.close();
#endif
  };





  // callback function for odom data
  void OdomEstimationNode::odomCallback(const OdomConstPtr& odom)
  {
    ROS_DEBUG("Odom callback at time %f ", ros::Time::now().toSec());
    assert(odom_used_);

    // receive data 
    boost::mutex::scoped_lock lock(odom_mutex_);
    odom_stamp_ = odom->header.stamp;
    odom_time_  = Time::now();
    Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
    odom_meas_  = Transform(q, Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, 0));
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
        odom_covariance_(i+1, j+1) = odom->pose.covariance[6*i+j];

    my_filter_.addMeasurement(Stamped<Transform>(odom_meas_, odom_stamp_,"wheelodom", "base_footprint"), odom_covariance_);
    
    // activate odom
    if (!odom_active_) {
      if (!odom_initializing_){
	odom_initializing_ = true;
	odom_init_stamp_ = odom_stamp_;
	ROS_INFO("Initializing Odom sensor");      
      }
      if ( filter_stamp_ >= odom_init_stamp_){
	odom_active_ = true;
	odom_initializing_ = false;
	ROS_INFO("Odom sensor activated");      
      }
      else ROS_INFO("Waiting to activate Odom, because Odom measurements are still %f sec in the future.", 
		    (odom_init_stamp_ - filter_stamp_).toSec());
    }
    
#ifdef __EKF_DEBUG_FILE__
    // write to file
    double tmp, yaw;
    odom_meas_.getBasis().getEulerZYX(yaw, tmp, tmp);
    odom_file_ << odom_meas_.getOrigin().x() << " " << odom_meas_.getOrigin().y() << "  " << yaw << "  " << endl;
#endif
  };




  // callback function for imu data
  void OdomEstimationNode::imuCallback(const ImuConstPtr& imu)
  {
    assert(imu_used_);

    // receive data 
    boost::mutex::scoped_lock lock(imu_mutex_);
    imu_stamp_ = imu->header.stamp;
    imu_time_  = Time::now();
    btQuaternion orientation;
    quaternionMsgToTF(imu->orientation, orientation);
    imu_meas_ = btTransform(orientation, btVector3(0,0,0));
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
        imu_covariance_(i+1, j+1) = imu->orientation_covariance[3*i+j];

    // manually set covariance untile imu sends covariance
    if (imu_covariance_(1,1) == 0.0){
      SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
      measNoiseImu_Cov(1,1) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(2,2) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(3,3) = pow(0.00017,2);  // = 0.01 degrees / sec
      imu_covariance_ = measNoiseImu_Cov;
    }

    my_filter_.addMeasurement(Stamped<Transform>(imu_meas_, imu_stamp_, "imu", "base_footprint"), imu_covariance_);
    
    // activate imu
    if (!imu_active_) {
      if (!imu_initializing_){
	imu_initializing_ = true;
	imu_init_stamp_ = imu_stamp_;
	ROS_INFO("Initializing Imu sensor");      
      }
      if ( filter_stamp_ >= imu_init_stamp_){
	imu_active_ = true;
	imu_initializing_ = false;
	ROS_INFO("Imu sensor activated");      
      }
      else ROS_INFO("Waiting to activate IMU, because IMU measurements are still %f sec in the future.", 
		    (imu_init_stamp_ - filter_stamp_).toSec());
    }
    
#ifdef __EKF_DEBUG_FILE__
    // write to file
    double tmp, yaw;
    imu_meas_.getBasis().getEulerZYX(yaw, tmp, tmp); 
    imu_file_ << yaw << endl;
#endif
  };




  // callback function for VO data
  void OdomEstimationNode::voCallback(const VoConstPtr& vo)
  {
    assert(vo_used_);

    // get data
    boost::mutex::scoped_lock lock(vo_mutex_);
    vo_stamp_ = vo->header.stamp;
    vo_time_  = Time::now();
    poseMsgToTF(vo->pose.pose, vo_meas_);
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
        vo_covariance_(i+1, j+1) = vo->pose.covariance[6*i+j];
    my_filter_.addMeasurement(Stamped<Transform>(vo_meas_, vo_stamp_, "vo", "base_footprint"), vo_covariance_);
    
    // activate vo
    if (!vo_active_) {
      if (!vo_initializing_){
	vo_initializing_ = true;
	vo_init_stamp_ = vo_stamp_;
	ROS_INFO("Initializing Vo sensor");      
      }
      if (filter_stamp_ >= vo_init_stamp_){
	vo_active_ = true;
	vo_initializing_ = false;
	ROS_INFO("Vo sensor activated");      
      }
      else ROS_INFO("Waiting to activate VO, because VO measurements are still %f sec in the future.", 
		    (vo_init_stamp_ - filter_stamp_).toSec());
    }
    
#ifdef __EKF_DEBUG_FILE__
    // write to file
    double Rx, Ry, Rz;
    vo_meas_.getBasis().getEulerZYX(Rz, Ry, Rx);
    vo_file_ << vo_meas_.getOrigin().x() << " " << vo_meas_.getOrigin().y() << " " << vo_meas_.getOrigin().z() << " "
	     << Rx << " " << Ry << " " << Rz << endl;
#endif
  };




  // callback function for vel data
  void OdomEstimationNode::velCallback(const VelConstPtr& vel)
  {
    // receive data
    boost::mutex::scoped_lock lock(vel_mutex_);
    vel_desi_(1) = vel->linear.x;   vel_desi_(2) = vel->angular.z;

    // active
    //if (!vel_active_) vel_active_ = true;
  };





  // filter loop
  void OdomEstimationNode::spin(const ros::TimerEvent& e)
  {
    ROS_DEBUG("Spin function at time %f", ros::Time::now().toSec());

    boost::mutex::scoped_lock odom_lock(odom_mutex_);
    boost::mutex::scoped_lock imu_lock(imu_mutex_);
    boost::mutex::scoped_lock vo_lock(vo_mutex_);
      
    // check for timing problems
    if ( (odom_initializing_ || odom_active_) && (imu_initializing_ || imu_active_) ){
      double diff = fabs( Duration(odom_stamp_ - imu_stamp_).toSec() );
      if (diff > 1.0) ROS_ERROR("Timestamps of odometry and imu are %f seconds apart.", diff);
    }
    
    // initial value for filter stamp; keep this stamp when no sensors are active
    filter_stamp_ = Time::now();
    
    // check which sensors are still active
    if ((odom_active_ || odom_initializing_) && 
        (Time::now() - odom_time_).toSec() > timeout_){
      odom_active_ = false; odom_initializing_ = false;
      ROS_INFO("Odom sensor not active any more");
    }
    if ((imu_active_ || imu_initializing_) && 
        (Time::now() - imu_time_).toSec() > timeout_){
      imu_active_ = false;  imu_initializing_ = false;
      ROS_INFO("Imu sensor not active any more");
    }
    if ((vo_active_ || vo_initializing_) && 
        (Time::now() - vo_time_).toSec() > timeout_){
      vo_active_ = false;  vo_initializing_ = false;
      ROS_INFO("VO sensor not active any more");
    }
    
    // only update filter when one of the sensors is active
    if (odom_active_ || imu_active_ || vo_active_){
      
      // update filter at time where all sensor measurements are available
      if (odom_active_)  filter_stamp_ = min(filter_stamp_, odom_stamp_);
      if (imu_active_)   filter_stamp_ = min(filter_stamp_, imu_stamp_);
      if (vo_active_)    filter_stamp_ = min(filter_stamp_, vo_stamp_);
      
      // update filter
      if ( my_filter_.isInitialized() )  {
        bool diagnostics = true;
        if (my_filter_.update(odom_active_, imu_active_, vo_active_,  filter_stamp_, diagnostics)){
          
          // output most recent estimate and relative covariance
          my_filter_.getEstimate(output_);
          pose_pub_.publish(output_);
          
          // broadcast most recent estimate to TransformArray
          Stamped<Transform> tmp;
          my_filter_.getEstimate(ros::Time(), tmp);
          if(!vo_active_)
            tmp.getOrigin().setZ(0.0);
          odom_broadcaster_.sendTransform(Stamped<Transform>(tmp, tmp.stamp_, "base_footprint", publish_name));
          
#ifdef __EKF_DEBUG_FILE__
          // write to file
          ColumnVector estimate; 
          my_filter_.getEstimate(estimate);
          for (unsigned int i=1; i<=6; i++)
            corr_file_ << estimate(i) << " ";
          corr_file_ << endl;
#endif
        }
        if (!diagnostics)
          ROS_WARN("Robot pose ekf diagnostics discovered a potential problem");
      }
      // initialize filer with odometry frame
      if ( odom_active_ && !my_filter_.isInitialized()){
        my_filter_.initialize(odom_meas_, odom_stamp_);
        ROS_INFO("Fiter initialized");
      }
      // initialize filter with visual odometry
      //if ( vo_active_ && !my_filter_.isInitialized()){
      //  my_filter_.initialize(vo_meas_, vo_stamp_);
      //  ROS_INFO("Fiter initialized");
      //}
      
    }
  };


}; // namespace






// ----------
// -- MAIN --
// ----------
using namespace estimation;
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "robot_pose_ekf");

  // create filter class
  OdomEstimationNode my_filter_node;

  ros::spin();
  
  return 0;
}
