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

#ifndef __ODOM_ESTIMATION__
#define __ODOM_ESTIMATION__

// bayesian filtering
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussianodo.h"

// TF
#include <tf/tf.h>

// msgs
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// log files
#include <fstream>

namespace estimation
{

class OdomEstimation
{
public:
  /// constructor
  OdomEstimation();

  /// destructor
  virtual ~OdomEstimation();

  /** update the extended Kalman filter
   * \param odom_active specifies if the odometry sensor is active or not
   * \param imu_active specifies if the imu sensor is active or not
   * \param gps_active specifies if the gps sensor is active or not
   * \param vo_active specifies if the vo sensor is active or not
   * \param filter_time update the ekf up to this time
   * \param diagnostics_res returns false if the diagnostics found that the sensor measurements are inconsistent
   * returns true on successfull update
   */
  bool update(bool odom_active, bool imu_active, bool gps_active, bool vo_active, const ros::Time& filter_time, bool& diagnostics_res);

  /** initialize the extended Kalman filter
   * \param prior the prior robot pose
   * \param time the initial time of the ekf
   */
  void initialize(const tf::Transform& prior, const ros::Time& time);

  /** check if the filter is initialized
   * returns true if the ekf has been initialized already
   */
  bool isInitialized() {return filter_initialized_;};

  /** get the filter posterior
   * \param estimate the filter posterior as a columnvector
   */
  void getEstimate(MatrixWrapper::ColumnVector& estimate);

  /** get the filter posterior
   * \param time the time of the filter posterior
   * \param estimate the filter posterior as a tf transform
   */
  void getEstimate(ros::Time time, tf::Transform& estimate);

  /** get the filter posterior
   * \param time the time of the filter posterior
   * \param estimate the filter posterior as a stamped tf transform
   */
  void getEstimate(ros::Time time, tf::StampedTransform& estimate);

  /** get the filter posterior
   * \param estimate the filter posterior as a pose with covariance
   */
  void getEstimate(geometry_msgs::PoseWithCovarianceStamped& estimate);

  /** Add a sensor measurement to the measurement buffer
   * \param meas the measurement to add
   */
  void addMeasurement(const tf::StampedTransform& meas);

  /** Add a sensor measurement to the measurement buffer
   * \param meas the measurement to add
   * \param covar the 6x6 covariance matrix of this measurement, as defined in the PoseWithCovariance message
   */
  void addMeasurement(const tf::StampedTransform& meas, const MatrixWrapper::SymmetricMatrix& covar);

  /** set the output frame used by tf
   * \param output_frame the desired output frame published on /tf (e.g., odom_combined)
   */
  void setOutputFrame(const std::string& output_frame);

  /** set the base_footprint frame of the robot used by tf
   * \param base_frame the desired base frame from which to transform when publishing the combined odometry frame (e.g., base_footprint)
   */
  void setBaseFootprintFrame(const std::string& base_frame);

private:
  /// correct for angle overflow
  void angleOverflowCorrect(double& a, double ref);

  // decompose Transform into x,y,z,Rx,Ry,Rz
  void decomposeTransform(const tf::StampedTransform& trans,
			  double& x, double& y, double&z, double&Rx, double& Ry, double& Rz);
  void decomposeTransform(const tf::Transform& trans,
			  double& x, double& y, double&z, double&Rx, double& Ry, double& Rz);


  // pdf / model / filter
  BFL::AnalyticSystemModelGaussianUncertainty*            sys_model_;
  BFL::NonLinearAnalyticConditionalGaussianOdo*           sys_pdf_;
  BFL::LinearAnalyticConditionalGaussian*                 odom_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* odom_meas_model_;
  BFL::LinearAnalyticConditionalGaussian*                 imu_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* imu_meas_model_;
  BFL::LinearAnalyticConditionalGaussian*                 vo_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* vo_meas_model_;
  BFL::LinearAnalyticConditionalGaussian*                 gps_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* gps_meas_model_;
  BFL::Gaussian*                                          prior_;
  BFL::ExtendedKalmanFilter*                              filter_;
  MatrixWrapper::SymmetricMatrix                          odom_covariance_, imu_covariance_, vo_covariance_, gps_covariance_;

  // vars
  MatrixWrapper::ColumnVector vel_desi_, filter_estimate_old_vec_;
  tf::Transform filter_estimate_old_;
  tf::StampedTransform odom_meas_, odom_meas_old_, imu_meas_, imu_meas_old_, vo_meas_, vo_meas_old_, gps_meas_, gps_meas_old_;
  ros::Time filter_time_old_;
  bool filter_initialized_, odom_initialized_, imu_initialized_, vo_initialized_, gps_initialized_;

  // diagnostics
  double diagnostics_odom_rot_rel_, diagnostics_imu_rot_rel_;

  // tf transformer
  tf::Transformer transformer_;

  std::string output_frame_;
  std::string base_footprint_frame_;

}; // class

}; // namespace

#endif
