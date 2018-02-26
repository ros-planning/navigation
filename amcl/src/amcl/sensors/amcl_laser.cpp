/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
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
// Desc: AMCL laser routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_laser.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <cstdio>
#include <ros/console.h>

#include "amcl/sensors/amcl_laser.h"

#define DELTA_INTERVAL 10

using namespace amcl;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLLaser::AMCLLaser(size_t max_beams, map_t* map) : AMCLSensor(), 
						     max_samples(0), max_obs(0), 
						     temp_obs(NULL)
{
  this->time = 0.0;

  this->max_beams = max_beams;
  this->map = map;

  return;
}

AMCLLaser::~AMCLLaser()
{
  if(temp_obs){
	for(int k=0; k < max_samples; k++){
	  delete [] temp_obs[k];
	}
	delete []temp_obs; 
  }
}

void 
AMCLLaser::SetModelBeam(double z_hit,
                        double z_short,
                        double z_max,
                        double z_rand,
                        double sigma_hit,
                        double lambda_short,
                        double chi_outlier)
{
  this->model_type = LASER_MODEL_BEAM;
  this->z_hit = z_hit;
  this->z_short = z_short;
  this->z_max = z_max;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->lambda_short = lambda_short;
  this->chi_outlier = chi_outlier;
}

void 
AMCLLaser::SetModelLikelihoodField(double z_hit,
                                   double z_rand,
                                   double sigma_hit,
                                   double max_occ_dist)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;

  map_update_cspace(this->map, max_occ_dist);
}

void 
AMCLLaser::SetModelLikelihoodFieldProb(double z_hit,
				       double z_rand,
				       double sigma_hit,
				       double max_occ_dist,
				       bool do_beamskip,
				       double beam_skip_distance,
				       double beam_skip_threshold, 
				       double beam_skip_error_threshold)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->do_beamskip = do_beamskip;
  this->beam_skip_distance = beam_skip_distance;
  this->beam_skip_threshold = beam_skip_threshold;
  this->beam_skip_error_threshold = beam_skip_error_threshold;
  map_update_cspace(this->map, max_occ_dist);
}

void 
AMCLLaser::SetModelCustomField(double z_hit,
                        double z_short,
                        double z_max,
                        double z_rand,
                        double sigma_hit,
                        double lambda_short,
                        double chi_outlier)
{
  this->model_type = LASER_MODEL_CUSTOM_FIELD;
  this->z_hit = z_hit;
  this->z_short = z_short;
  this->z_max = z_max;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->lambda_short = lambda_short;
  this->chi_outlier = chi_outlier;
}
  
void 
AMCLLaser::SetModelDelta(double z_hit,
                        double z_short,
                        double z_max,
                        double z_rand,
                        double sigma_hit,
                        double lambda_short,
                        double chi_outlier)
{
  this->model_type = LASER_MODEL_DELTA;
  this->z_hit = z_hit;
  this->z_short = z_short;
  this->z_max = z_max;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->lambda_short = lambda_short;
  this->chi_outlier = chi_outlier;
}

////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool AMCLLaser::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  if (this->max_beams < 2)
    return false;

  // Apply the laser sensor model
  if(this->model_type == LASER_MODEL_BEAM)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) BeamModel, data);
  else if(this->model_type == LASER_MODEL_LIKELIHOOD_FIELD)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) LikelihoodFieldModel, data);  
  else if(this->model_type == LASER_MODEL_LIKELIHOOD_FIELD_PROB)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) LikelihoodFieldModelProb, data);  
  else if(this->model_type == LASER_MODEL_CUSTOM_FIELD)
    pf_update_sensor(pf, (pf_sensor_model_fn_t) CustomFieldModel, data);
  else
    pf_update_sensor(pf, (pf_sensor_model_fn_t) DeltaModel, data);
  return true;
}


////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose
double AMCLLaser::BeamModel(AMCLLaserData *data, pf_sample_set_t* set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;

  self = (AMCLLaser*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams - 1);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // Compute the range according to the map
      map_range = map_calc_range(self->map, pose.v[0], pose.v[1],
                                 pose.v[2] + obs_bearing, data->range_max);
      pz = 0.0;
      if(isnan(obs_range))
        obs_range = data->range_max;
      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));
      // fprintf(stderr, "z, obs_range, map_range = %f, %f, %f \n", z, obs_range, map_range);
      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if(z < 0)
        pz += self->z_short * self->lambda_short * exp(-self->lambda_short*obs_range);

      // Part 3: Failure to detect obstacle, reported as max-range
      if(obs_range == data->range_max)
        pz += self->z_max * 1.0;

      // Part 4: Random measurements
      if(obs_range < data->range_max)
        pz += self->z_rand * 1.0/data->range_max;

      // TODO: outlier rejection for short readings
      // fprintf(stderr, "pz = %f \n", pz);
      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}

double AMCLLaser::LikelihoodFieldModel(AMCLLaserData *data, pf_sample_set_t* set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (AMCLLaser*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    // Pre-compute a couple of things
    double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
    double z_rand_mult = 1.0/data->range_max;

    step = (data->range_count - 1) / (self->max_beams - 1);

    // Step size must be at least 1
    if(step < 1)
      step = 1;

    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if(obs_range >= data->range_max)
        continue;

      // Check for NaN
      if(obs_range != obs_range)
        continue;

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if(!MAP_VALID(self->map, mi, mj))
        z = self->map->max_occ_dist;
      else
        z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  return(total_weight);
}

double AMCLLaser::LikelihoodFieldModelProb(AMCLLaserData *data, pf_sample_set_t* set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double log_p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (AMCLLaser*) data->sensor;

  total_weight = 0.0;

  step = ceil((data->range_count) / static_cast<double>(self->max_beams)); 
  
  // Step size must be at least 1
  if(step < 1)
    step = 1;

  // Pre-compute a couple of things
  double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
  double z_rand_mult = 1.0/data->range_max;

  double max_dist_prob = exp(-(self->map->max_occ_dist * self->map->max_occ_dist) / z_hit_denom);

  //Beam skipping - ignores beams for which a majoirty of particles do not agree with the map
  //prevents correct particles from getting down weighted because of unexpected obstacles 
  //such as humans 

  bool do_beamskip = self->do_beamskip;
  double beam_skip_distance = self->beam_skip_distance;
  double beam_skip_threshold = self->beam_skip_threshold;
  
  //we only do beam skipping if the filter has converged 
  if(do_beamskip && !set->converged){
    do_beamskip = false;
  }

  //we need a count the no of particles for which the beam agreed with the map 
  int *obs_count = new int[self->max_beams]();

  //we also need a mask of which observations to integrate (to decide which beams to integrate to all particles) 
  bool *obs_mask = new bool[self->max_beams]();
  
  int beam_ind = 0;
  
  //realloc indicates if we need to reallocate the temp data structure needed to do beamskipping 
  bool realloc = false; 

  if(do_beamskip){
    if(self->max_obs < self->max_beams){
      realloc = true;
    }

    if(self->max_samples < set->sample_count){
      realloc = true;
    }

    if(realloc){
      self->reallocTempData(set->sample_count, self->max_beams);     
      fprintf(stderr, "Reallocing temp weights %d - %d\n", self->max_samples, self->max_obs);
    }
  }

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    log_p = 0;
    
    beam_ind = 0;
    
    for (i = 0; i < data->range_count; i += step, beam_ind++)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if(obs_range >= data->range_max){
        continue;
      }

      // Check for NaN
      if(obs_range != obs_range){
        continue;
      }

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      
      if(!MAP_VALID(self->map, mi, mj)){
	pz += self->z_hit * max_dist_prob;
      }
      else{
	z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
	if(z < beam_skip_distance){
	  obs_count[beam_ind] += 1;
	}
	pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      }
       
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      assert(pz <= 1.0); 
      assert(pz >= 0.0);

      // TODO: outlier rejection for short readings
            
      if(!do_beamskip){
	log_p += log(pz);
      }
      else{
	self->temp_obs[j][beam_ind] = pz; 
      }
    }
    if(!do_beamskip){
      sample->weight *= exp(log_p);
      total_weight += sample->weight;
    }
  }
  
  if(do_beamskip){
    int skipped_beam_count = 0; 
    for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++){
      if((obs_count[beam_ind] / static_cast<double>(set->sample_count)) > beam_skip_threshold){
	obs_mask[beam_ind] = true;
      }
      else{
	obs_mask[beam_ind] = false;
	skipped_beam_count++; 
      }
    }

    //we check if there is at least a critical number of beams that agreed with the map 
    //otherwise it probably indicates that the filter converged to a wrong solution
    //if that's the case we integrate all the beams and hope the filter might converge to 
    //the right solution
    bool error = false; 

    if(skipped_beam_count >= (beam_ind * self->beam_skip_error_threshold)){
      fprintf(stderr, "Over %f%% of the observations were not in the map - pf may have converged to wrong pose - integrating all observations\n", (100 * self->beam_skip_error_threshold));
      error = true; 
    }

    for (j = 0; j < set->sample_count; j++)
      {
	sample = set->samples + j;
	pose = sample->pose;

	log_p = 0;

	for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++){
	  if(error || obs_mask[beam_ind]){
	    log_p += log(self->temp_obs[j][beam_ind]);
	  }
	}
	
	sample->weight *= exp(log_p);
	
	total_weight += sample->weight;
      }      
  }

  delete [] obs_count; 
  delete [] obs_mask;
  return(total_weight);
}

double AMCLLaser::CustomFieldModel(AMCLLaserData *data, pf_sample_set_t* set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  double total_weight; 
  pf_sample_t *sample;
  pf_vector_t pose;
  double *obs_array;
  double *map_array;
  double obs_range_mean=0, obs_range_var=0, obs_range_max=-1;
  double map_range_mean=0, map_range_var=0, map_range_max=-1;
  double mean_filter=0;//assigned initial value
  int counter = 0;
  // freopen( "output.txt", "w", stdout );

  // ROS_DEBUG("CUSTOM FIELD");
  // fprintf(stdout, "CUSTOM FIELD");
  fprintf(stderr, "TESTING\n");
  fprintf(stderr, "%f, %f\n", obs_range, map_range);

  self = (AMCLLaser*) data->sensor;
  obs_array = new double[data->range_count];
  map_array = new double[data->range_count];

  total_weight = 0.0;
  step = (data->range_count - 1) / (self->max_beams - 1);  

  for (int idx=0; idx < data->range_count; idx += step){
    obs_array[idx] = data->ranges[idx][0];
    if(isnan(data->ranges[idx][0]))
      continue;
    else
      counter++;
    obs_range_mean += obs_array[idx]; 
    if (obs_range_max < obs_array[idx])
      obs_range_max = obs_array[idx];
  }
  obs_range_mean = obs_range_mean / counter;
  if(isnan(obs_range_mean))
    return total_weight; 

  counter = 0;
  for (int idx=0; idx < data->range_count; idx += step){
    if(isnan(obs_array[idx]))
      continue;
    else
      counter++;
    // fprintf(stderr, "%f, %f, %d \n", obs_array[idx], obs_range_var, counter);
    obs_range_var += (obs_array[idx] - obs_range_mean) * (obs_array[idx] - obs_range_mean);
  }
  obs_range_var /= counter;
  // fprintf(stderr, "mean, var, max = %f, %f, %f \n", obs_range_mean, obs_range_var, obs_range_max);
  if(isnan(obs_range_var)){
    fprintf(stderr, "SKIPPED\n");
    return total_weight;
  }
  // counter=0;
  // for (int idx=0; idx < data->range_count; idx += step){
  //   if(isnan(obs_array[idx]))
  //     continue;
  //   else
  //     counter++;
  //   fprintf(stderr, "obs_array[%d], %f\n", idx, obs_array[idx]);
  // }
    // obs_range_var = 0;

  // fprintf(stderr, "mean, var, max = %f, %f, %f \n", obs_range_mean, obs_range_var, obs_range_max);

      // Mean Filtering for Observation Array
      for (int idx = 0; idx < data->range_count; idx += step){
        if (idx == 0){
          for (int j = 0; j < 10*step; j += step)
            mean_filter += obs_array[idx+j];
           mean_filter /= 10;
           obs_array[idx] = mean_filter;
        }
        else if (idx == (data->range_count/step)*step - 9*step){
          mean_filter *= 10;
          mean_filter -= obs_array[idx - step];
          mean_filter += obs_array[idx + 9*step];
          mean_filter /= 10;
          obs_array[idx] = mean_filter;
          // change it in the future!!!!
          for (int j = 0; j < 10*step; j += step){
            mean_filter -= obs_array[j - step];
            mean_filter /= 10 - j;
            obs_array[j] = mean_filter;
          }
            
        }
        else{
          mean_filter *= 10;
          mean_filter -= obs_array[idx - step];
          mean_filter += obs_array[idx + 9*step];
          mean_filter /= 10;
          obs_array[idx] = mean_filter;
        }
  
      }
  // for (int idx=0; idx < data->range_count; idx += step){
  //   fprintf(stderr, "obs_array[%d], %f\n", idx, obs_array[idx]);
  // }
  // fprintf(stderr, "MEAN FILTERED\n");
  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);
    pose.v[0]=0; pose.v[1]=0;pose.v[2]=0;
    
    p = 1.0;
    
    step = (data->range_count - 1) / (self->max_beams - 1);
    counter = 0;


    for (int idx = 0; idx < data->range_count; idx += step){
      obs_bearing = data->ranges[idx][1];

      map_array[idx] = map_calc_range(self->map, pose.v[0], pose.v[1],
        pose.v[2] + obs_bearing, data->range_max);
    
      if(map_range_max < map_array[idx] && map_array[idx] != data->range_max)
        map_range_max = map_array[idx];
    }

    for (int idx = 0; idx < data->range_count; idx += step){
      if(map_array[idx]  == data->range_max)
        map_array[idx] = map_range_max;
    }
    // for (int idx=0; idx < data->range_count; idx += step){
    //   fprintf(stderr, "map_array[%d], %f\n", idx, map_array[idx]);
    // }
    // fprintf(stderr, "MAP ARRAY ASSIGNMENTS\n");
    // Mean Filtering for Map Array
    for (int idx = 0; idx < data->range_count; idx += step){
      if (idx == 0){
        for (int j = 0; j < 10*step; j += step)
          mean_filter += map_array[idx+j];
         mean_filter /= 10;
         map_array[idx] = mean_filter;
      }
      else if (idx == (data->range_count/step)*step - 9*step){
        mean_filter *= 10;
        mean_filter -= map_array[idx - step];
        mean_filter += map_array[idx + 9*step];
        mean_filter /= 10;
        map_array[idx] = mean_filter;
        // change it in the future!!!! ask dr duff
        for (int j = 0; j < 10*step; j += step){
          mean_filter -= map_array[j - step];
          mean_filter /= 10 - j;
          map_array[j] = mean_filter;
        }
          
      }
      else{
        mean_filter *= 10;
        mean_filter -= map_array[idx - step];
        mean_filter += map_array[idx + 9*step];
        mean_filter /= 10;
        map_array[idx] = mean_filter;
      }

    } 

    // Mean Calculation
    for (int idx = 0; idx < data->range_count; idx += step){
      map_range_mean += map_array[idx];
      counter++;
    }
    
    map_range_mean /= counter;

    // Variance Calculation
    for (int idx = 0; idx < data->range_count; idx += step){
      map_range_var += (map_array[idx] - map_range_mean)*(map_array[idx] - map_range_mean);
      // fprintf(stderr, "map_range_mean, map_array = %f, %f \n", map_range_mean, map_array[idx]);
    }

    map_range_var /= (counter - 1);//why minus one?
    fprintf(stderr, "RAW\n");
    fprintf(stderr, "map, obs ,%f,%f,%f\n", pose.v[0], pose.v[1],
    pose.v[2] );
    for (int idx=0; idx < data->range_count; idx += step){
      if(isnan(obs_array[idx]))
        continue;
      fprintf(stderr, "%f, %f\n", map_array[idx], obs_array[idx]);
    }
    fprintf(stderr, "NORMALIZED\n");
    fprintf(stderr, "map, obs\n");
    for (i = 0; i < data->range_count; i += step)
    {
      // obs_range = data->ranges[i][0];
      // obs_bearing = data->ranges[i][1];
      // fprintf(stderr, "%f, %f\n", obs_array[i], map_array[i]);
      
      // TODO: Kinect -> obs_range_max, NN -> skip
      if(isnan(obs_array[i]))
        continue;
        // obs_range = obs_range_max;
      
      obs_range = (obs_array[i]- obs_range_mean) / sqrt(obs_range_var);
      // ROS_INFO("%f", obs_range);
      
      // In a kinect mode if it is NaN set it to 0
      // if(isnan(obs_range))
      //   obs_range=0;
      // Compute the range according to the map
      map_range = map_array[i];

      map_range = (map_range - map_range_mean)/ sqrt(map_range_var);
      // if (!map_range_var)
      //   fprintf(stderr, "map_range_mean, map_range_var, index = %f, %f, %d \n", map_range_mean, map_range_var, i);
      if (isnan(map_range))
        map_range = obs_range_max;
      pz = 0.0;
      
      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));
      // fprintf(stderr, "z, obs_range, map_range = %f, %f, %f \n", z, obs_range, map_range);

      // Part 2: Random measurements
      if(obs_range < obs_range_max)
        pz += self->z_rand * 1.0/obs_range_max;
      
      // fprintf(stderr, "%f, %f\n", map_range, obs_range);
      // TODO: outlier rejection for short readings
      assert(pz <= 1.0);
      assert(pz >= 0.0);
      p += pz*pz*pz*pz*pz*pz*pz;
      // if (i % 132 == 0)
      //   fprintf(stderr, "obs_range, map_range, z, pz = %f, %f, %f, %f \n", obs_range, map_range, z, pz);

      
    }

    sample->weight *= p;
    total_weight += sample->weight;
  }

  delete [] obs_array;
  delete [] map_array;
  return(total_weight);
}

double AMCLLaser::DeltaModel(AMCLLaserData *data, pf_sample_set_t* set)
{
  AMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  double total_weight;
  pf_sample_t *sample;
  pf_vector_t pose;
  double delta, prev_obs_range, prev_map_range;
  int obs_delta, map_delta;
  double *obs_array;
  double *map_array;
  double obs_range_mean=0, obs_range_var=0, obs_range_max=-1;
  double map_range_mean=0, map_range_var=0, map_range_max=-1;
  double mean_filter;
  int counter = 0;

  self = (AMCLLaser*) data->sensor;
  obs_array = new double[data->range_count];
  map_array = new double[data->range_count];

  total_weight = 0.0;
  step = (data->range_count - 1) / (self->max_beams - 1);  

  for (int idx=0; idx < data->range_count; idx += step){
    obs_range = data->ranges[idx][0];
    if(isnan(obs_range))
      continue;
    // obs_range_mean += obs_range; 
    if (obs_range_max < obs_range)
      obs_range_max = obs_range;
  }
  // obs_range_mean = obs_range_mean / data->range_count;

  // for (int idx=0; idx < data->range_count; idx += step){
  //   obs_range = data->ranges[idx][0];
  //   if(isnan(obs_range))
  //     continue;
  //   obs_range_var += (obs_range - obs_range_mean) * (obs_range - obs_range_mean);
  // }
  // obs_range_var /= data->range_count;

  // fprintf(stdout, "mean, var, max = %f, %f, %f \n", obs_range_mean, obs_range_var, obs_range_max);

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;
    
    step = (data->range_count - 1) / (self->max_beams - 1);
    counter = 0;

    for (int idx = 0; idx < data->range_count; idx += step){
      obs_bearing = data->ranges[idx][1];

      map_array[idx] = map_calc_range(self->map, pose.v[0], pose.v[1],
        pose.v[2] + obs_bearing, data->range_max);
    
      if(map_range_max < map_array[idx] && map_array[idx] != data->range_max)
        map_range_max = map_array[idx];
    }

    for (int idx = 0; idx < data->range_count; idx += step){
      if(map_array[idx]  == data->range_max)
        map_array[idx] = map_range_max;
    }
    // Mean Filtering
    for (int idx = 0; idx < data->range_count; idx += step){
      if (idx == 0){
        for (int j = 0; j < 10*step; j += step)
          mean_filter += map_array[idx+j];
         mean_filter /= 10;
         map_array[idx] = mean_filter;
      }
      else if (idx == (data->range_count/step)*step - 9*step){
        mean_filter *= 10;
        mean_filter -= map_array[idx - step];
        mean_filter += map_array[idx + 9*step];
        mean_filter /= 10;
        map_array[idx] = mean_filter;
        // change it in the future!!!!
        for (int j = 0; j < 10*step; j += step){
          mean_filter -= map_array[j - step];
          mean_filter /= 10 - j;
          map_array[j] = mean_filter;
        }
          
      }
      else{
        mean_filter *= 10;
        mean_filter -= map_array[idx - step];
        mean_filter += map_array[idx + 9*step];
        mean_filter /= 10;
        map_array[idx] = mean_filter;
      }

    }

    counter = 0;
    for (i = 1; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      prev_obs_range = data->ranges[i-1][0];
      
      // if(isnan(obs_range))
      //   obs_range = obs_range_max;
      if(isnan(obs_range))
        continue;
      
      // Compute the range according to the map
      map_range = map_array[i];
      prev_map_range = map_array[i-1];

      pz = 0.0;
      
      // Calculate Delta Score
      // 0 := same value
      // 1 := ascending
      // 2 := descending
      // 3 := max value
      if (obs_range == obs_range_max)
        obs_delta = 3;
      else if (obs_range == prev_obs_range)
        obs_delta = 0;
      else if (obs_range > prev_obs_range)
        obs_delta = 1;
      else if (obs_range < prev_obs_range)
        obs_delta = 2;

      if (map_range == data->range_max)
        map_delta = 3;
      else if (map_range == prev_map_range)
        map_delta = 0;
      else if (map_range > prev_map_range)
        map_delta = 1;
      else if (map_range < prev_map_range)
        map_delta = 2;
      
      if (obs_delta == map_delta)
        delta++;
      // Part 1: good, but noisy, hit
      // z = obs_range - map_range;
      // z = delta / data->range_count;
      pz = z;
      // pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));
      // fprintf(stdout, "z, obs_range, map_range = %f, %f, %f \n", z, obs_range, map_range);

      // Part 2: Random measurements
      if(obs_range < obs_range_max)
        pz += self->z_rand * 1.0/obs_range_max;

      // TODO: outlier rejection for short readings
      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }
    if (!counter)
      pz = 0;
    else
      pz =  delta / counter;
      p += pz*pz*pz*pz*pz;
    sample->weight *= p;
    total_weight += sample->weight;
  }

  delete [] obs_array;
  delete [] map_array;
  return(total_weight);
}

void AMCLLaser::reallocTempData(int new_max_samples, int new_max_obs){
  if(temp_obs){
    for(int k=0; k < max_samples; k++){
      delete [] temp_obs[k];
    }
    delete []temp_obs; 
  }
  max_obs = new_max_obs; 
  max_samples = fmax(max_samples, new_max_samples); 

  temp_obs = new double*[max_samples]();
  for(int k=0; k < max_samples; k++){
    temp_obs[k] = new double[max_obs]();
  }
}
