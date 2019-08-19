/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey et al.
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
///////////////////////////////////////////////////////////////////////////
//
// Desc: Odometry sensor model for AMCL
// Author: Andrew Howard
// Date: 17 Aug 2003
// CVS: $Id: amcl_odom.h 4135 2007-08-23 19:58:48Z gerkey $
//
///////////////////////////////////////////////////////////////////////////

#ifndef AMCL_POSE_H
#define AMCL_POSE_H

#include "mel_amcl_sensor.h"
#include "../pf/pf_pdf.h"

namespace mel_amcl
{


// Map position sensor data
class AMCLPoseData : public AMCLSensorData
{
  // Map frame pose
  public: 
    AMCLPoseData () {};
    pf_vector_t pose;
    pf_vector_t pose_covariance;

};


// Pose sensor model
class AMCLPose : public AMCLSensor
{
  // Default constructor
  public: AMCLPose();

  // Update the filter based on the sensor model.  Returns true if the filter
  // has been updated.
  public: virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data);

  // Determine the probability for the given pose
  private: static double GaussianModel(AMCLPoseData *data, 
                                   pf_sample_set_t* set);


  // Current data timestamp
  private: double time;
  
};


}

#endif
