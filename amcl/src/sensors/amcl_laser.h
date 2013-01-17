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
// Desc: LASER sensor model for AMCL
// Author: Andrew Howard
// Date: 17 Aug 2003
// CVS: $Id: amcl_laser.h 6443 2008-05-15 19:46:11Z gerkey $
//
///////////////////////////////////////////////////////////////////////////

#ifndef AMCL_LASER_H
#define AMCL_LASER_H

#include "amcl_sensor.h"
#include "../map/map.h"

namespace amcl
{

typedef enum
{
  LASER_MODEL_BEAM,
  LASER_MODEL_LIKELIHOOD_FIELD
} laser_model_t;

// Laser sensor data
class AMCLLaserData : public AMCLSensorData
{
  public:
    AMCLLaserData () {ranges=NULL;};
    virtual ~AMCLLaserData() {delete [] ranges;};
  // Laser range data (range, bearing tuples)
  public: int range_count;
  public: double range_max;
  public: double (*ranges)[2];
};


// Laseretric sensor model
class AMCLLaser : public AMCLSensor
{
  // Default constructor
  public: AMCLLaser(size_t max_beams, map_t* map);

  public: void SetModelBeam(double z_hit,
                            double z_short,
                            double z_max,
                            double z_rand,
                            double sigma_hit,
                            double labda_short,
                            double chi_outlier);

  public: void SetModelLikelihoodField(double z_hit,
                                       double z_rand,
                                       double sigma_hit,
                                       double max_occ_dist);
  
  // Update the filter based on the sensor model.  Returns true if the
  // filter has been updated.
  public: virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data);

  // Set the laser's pose after construction
  public: void SetLaserPose(pf_vector_t& laser_pose) 
          {this->laser_pose = laser_pose;}

  // Determine the probability for the given pose
  private: static double BeamModel(AMCLLaserData *data, 
                                   pf_sample_set_t* set);
  // Determine the probability for the given pose
  private: static double LikelihoodFieldModel(AMCLLaserData *data, 
                                              pf_sample_set_t* set);

  private: laser_model_t model_type;

  // Current data timestamp
  private: double time;

  // The laser map
  private: map_t *map;

  // Laser offset relative to robot
  private: pf_vector_t laser_pose;
  
  // Max beams to consider
  private: int max_beams;

  // Laser model params
  //
  // Mixture params for the components of the model; must sum to 1
  private: double z_hit;
  private: double z_short;
  private: double z_max;
  private: double z_rand;
  //
  // Stddev of Gaussian model for laser hits.
  private: double sigma_hit;
  // Decay rate of exponential model for short readings.
  private: double lambda_short;
  // Threshold for outlier rejection (unused)
  private: double chi_outlier;
};


}

#endif
