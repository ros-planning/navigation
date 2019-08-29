///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL pose routines
// Author: Michael Hutchinson
// Date: 19 Aug 2019
//
///////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <assert.h>

#include <sys/types.h> // required by Darwin
#include <math.h>
#include "ros/ros.h"
#include "mel_amcl/sensors/mel_amcl_pose.h"

using namespace mel_amcl;


////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLPose::AMCLPose() : AMCLSensor()
{
  this->time = 0.0;
}


////////////////////////////////////////////////////////////////////////////////
// Apply the position sensor model
bool AMCLPose::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{

  pf_update_sensor(pf, (pf_sensor_model_fn_t) GaussianModel, data);

  return true;
}


double AMCLPose::GaussianModel(AMCLPoseData *data, pf_sample_set_t* set)
{  
  
  int j;
  double p;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  double angle_error;
  total_weight = 0.0;

  double additional_pose_covariance = 0.4;
  double additional_yaw_covariance = 0.2;


  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {


    sample = set->samples + j;
    pose = sample->pose;

    // use gps positions and covariances separately or combine to distance from pose?
    // are the gps data covariance in the correct frame - check navsat output
    // need to get gps data inc covariances into this function

    angle_error = fmod(pose.v[2] - data->pose.v[2] + M_PI, 2*M_PI);
    if (angle_error < 0)
        angle_error += 2*M_PI;
    angle_error -= M_PI;


    double x_p =  exp(-0.5 * (pow((pose.v[0] - data->pose.v[0]), 2) / (data->pose_covariance.v[0]+additional_pose_covariance))) / sqrt(2 * M_PI * (data->pose_covariance.v[0]+additional_pose_covariance));
    double y_p =  exp(-0.5 * (pow((pose.v[1] - data->pose.v[1]), 2) / (data->pose_covariance.v[1]+additional_pose_covariance))) / sqrt(2 * M_PI * (data->pose_covariance.v[1]+additional_pose_covariance));
    double yaw_p =  exp(-0.5 * (pow(angle_error, 2) / (data->pose_covariance.v[2]+additional_yaw_covariance))) / sqrt(2 * M_PI * (data->pose_covariance.v[2]+additional_yaw_covariance));
    //double num = exp(-0.5 * (pow((pose.v[0] - data->pose.v[0]), 2) / (data->pose_covariance.v[0]+additional_pose_covariance) + pow((pose.v[1] - data->pose.v[1]), 2) / (data->pose_covariance.v[1]+additional_pose_covariance) + pow(angle_error, 2) / (data->pose_covariance.v[2]+additional_yaw_covariance) ));
    //double denom = sqrt(2 * M_PI * (data->pose_covariance.v[0]+additional_pose_covariance) * (data->pose_covariance.v[1]+additional_pose_covariance) * (data->pose_covariance.v[2]+additional_yaw_covariance));
    //p = num/denom;
    p = x_p*y_p*yaw_p;
    //assert(p <= 1.0);
    //assert(p >= 0.0);
    

    sample->weight *= p;
    
    total_weight += sample->weight;
  }

  //ROS_INFO("Angle_error %f.", angle_error);

  return(total_weight);

}
