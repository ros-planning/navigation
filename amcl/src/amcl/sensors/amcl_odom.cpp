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
// Desc: AMCL odometry routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_odom.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <algorithm>

#include <sys/types.h> // required by Darwin
#include <math.h>

#include "amcl_odom.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////

AMCLOdom::AMCLOdom(double _a_thresh, double _d_thresh)
: AMCLSensor(), a_thresh(abs(_a_thresh)), d_thresh(abs(_d_thresh/50.0))
{
    this->time = 0.0;
}

void
AMCLOdom::SetModelDiff(double alpha1,
    double alpha2,
    double alpha3,
    double alpha4)
{
    this->model_type = ODOM_MODEL_DIFF;
    this->alpha1 = alpha1;
    this->alpha2 = alpha2;
    this->alpha3 = alpha3;
    this->alpha4 = alpha4;
}

void
AMCLOdom::SetModelOmni(double alpha1,
    double alpha2,
    double alpha3,
    double alpha4,
    double alpha5)
{
    this->model_type = ODOM_MODEL_OMNI;
    this->alpha1 = alpha1;
    this->alpha2 = alpha2;
    this->alpha3 = alpha3;
    this->alpha4 = alpha4;
    this->alpha5 = alpha5;
}

void
AMCLOdom::SetModel( odom_model_t type,
    double alpha1,
    double alpha2,
    double alpha3,
    double alpha4,
    double alpha5 )
{
    this->model_type = type;
    this->alpha1 = alpha1;
    this->alpha2 = alpha2;
    this->alpha3 = alpha3;
    this->alpha4 = alpha4;
    this->alpha5 = alpha5;
}

////////////////////////////////////////////////////////////////////////////////
// Apply the action model
bool AMCLOdom::UpdateAction(pf_t *pf, AMCLSensorData *data)
{
    AMCLOdomData *ndata;
    ndata = (AMCLOdomData*) data;

    // Compute the new sample poses
    pf_sample_set_t *set;

    set = pf->sets + pf->current_set;
    pf_vector_t old_pose = pf_vector_sub(ndata->pose, ndata->delta);

    double delta_trans = sqrt(pow(ndata->delta.v[0], 2) + pow(ndata->delta.v[1], 2));
#ifdef BUILD_DEBUG
    printf("====================\n");
    printf("Odom pose @ t-1: %lf, %lf, yaw %lf\n", old_pose.v[0], old_pose.v[1], old_pose.v[2]);
    printf("Odom pose @ t: %lf, %lf, yaw %lf\n", ndata->pose.v[0], ndata->pose.v[1], ndata->pose.v[2]);
    printf("\tDelta: %lf, %lf, yaw %lf\n", ndata->delta.v[0], ndata->delta.v[1], ndata->delta.v[2]);
    printf("====================\n");
#endif
    if (! (delta_trans || ndata->delta.v[2])) return false;

    switch( this->model_type )
    {
    case ODOM_MODEL_OMNI:
        {
            double delta_rot, delta_bearing;
            double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

            delta_rot = ndata->delta.v[2];

            // Precompute a couple of things
            double trans_hat_stddev = (alpha3 * (delta_trans*delta_trans) +
            alpha1 * (delta_rot*delta_rot));
            trans_hat_stddev = sqrt(trans_hat_stddev);
            double rot_hat_stddev = (alpha4 * (delta_rot*delta_rot) +
            alpha2 * (delta_trans*delta_trans));
            rot_hat_stddev = sqrt(rot_hat_stddev);
            double strafe_hat_stddev = (alpha1 * (delta_rot*delta_rot) +
            alpha5 * (delta_trans*delta_trans));
            strafe_hat_stddev = sqrt(strafe_hat_stddev);

            for (int i = 0; i < set->sample_count; i++)
            {
                pf_sample_t* sample = set->samples + i;

                delta_bearing = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
                old_pose.v[2]) + sample->pose.v[2];

                double cs_bearing = cos(delta_bearing);
                double sn_bearing = sin(delta_bearing);

                // Sample pose differences
                delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
                delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
                delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);
                // Apply sampled update to particle pose
                sample->pose.v[0] += (delta_trans_hat * cs_bearing +
                delta_strafe_hat * sn_bearing);
                sample->pose.v[1] += (delta_trans_hat * sn_bearing -
                delta_strafe_hat * cs_bearing);
                sample->pose.v[2] = normalize(sample->pose.v[2] + delta_rot_hat);
            }
        }
        break;
    case ODOM_MODEL_DIFF:
        {
            // Implement sample_motion_odometry (Prob Rob p 136)
            double delta_rot1, delta_rot2;
            double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
            double trans_hat_stddev, rot1_hat_stddev, rot2_hat_stddev;
            double delta_rot1_noise, delta_rot2_noise;

            delta_rot1 = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
            old_pose.v[2]);
            delta_rot2 = angle_diff(ndata->delta.v[2], delta_rot1);
            bool in_place_rotation = abs(delta_trans) < d_thresh && ndata->delta.v[2] > a_thresh;
            #ifdef BUILD_DEBUG
            if (in_place_rotation)
            {
                printf("//////////////////////////\n/// in_place_rotation /// delta_trans = %lf, delta_theta = %lf\n/////////////////////////\n", delta_trans, ndata->delta.v[2]);
            }
            #endif

            // We want to treat backward and forward motion symmetrically for the
            // noise model to be applied below.  The standard model seems to assume
            // forward motion.
            delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)), fabs(angle_diff(delta_rot1,M_PI)));
            delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)), fabs(angle_diff(delta_rot2,M_PI)));

            trans_hat_stddev = alpha3 * pow(delta_trans, 2)
                + alpha4 * (pow(delta_rot1_noise, 2) + pow(delta_rot2_noise, 2));
            trans_hat_stddev = sqrt(trans_hat_stddev);
            rot1_hat_stddev = alpha1 * pow(delta_rot1_noise, 2) + alpha2 * pow(delta_trans, 2);
            rot1_hat_stddev = sqrt(rot1_hat_stddev);
            rot2_hat_stddev = alpha1 * pow(delta_rot2_noise, 2) + alpha2 * pow(delta_trans, 2);
            rot2_hat_stddev = sqrt(rot2_hat_stddev);

            if (in_place_rotation)
            {
                if ( trans_hat_stddev > delta_trans )
                {
                    trans_hat_stddev = delta_trans;
                }
                delta_trans = 0; // so sample rotation as if no velocity lost to trans
            }

            for (int i = 0; i < set->sample_count; i++)
            {
                pf_sample_t* sample = set->samples + i;

                // Sample pose differences
                delta_rot1_hat = angle_diff(delta_rot1, pf_ran_gaussian(rot1_hat_stddev));
                delta_trans_hat = delta_trans - pf_ran_gaussian(trans_hat_stddev);
                delta_rot2_hat = angle_diff(delta_rot2, pf_ran_gaussian(rot2_hat_stddev));

                // Apply sampled update to particle pose
                sample->pose.v[0] += delta_trans_hat * cos(sample->pose.v[2] + delta_rot1_hat);
                sample->pose.v[1] += delta_trans_hat * sin(sample->pose.v[2] + delta_rot1_hat);
                sample->pose.v[2] = normalize(sample->pose.v[2] + delta_rot1_hat + delta_rot2_hat);
            }
        }
        break;
    case ODOM_MODEL_OMNI_CORRECTED:
        {
            double delta_rot, delta_bearing;
            double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

            delta_rot = ndata->delta.v[2];

            // Precompute a couple of things
            double trans_hat_stddev = (alpha3 * (delta_trans*delta_trans) +
            alpha1 * (delta_rot*delta_rot));
            trans_hat_stddev = sqrt(trans_hat_stddev);
            double rot_hat_stddev = (alpha4 * (delta_rot*delta_rot) +
            alpha2 * (delta_trans*delta_trans));
            rot_hat_stddev = sqrt(rot_hat_stddev);
            double strafe_hat_stddev = (alpha1 * (delta_rot*delta_rot) +
            alpha5 * (delta_trans*delta_trans));
            strafe_hat_stddev = sqrt(strafe_hat_stddev);

            for (int i = 0; i < set->sample_count; i++)
            {
                pf_sample_t* sample = set->samples + i;

                delta_bearing = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
                old_pose.v[2]) + sample->pose.v[2];
                double cs_bearing = cos(delta_bearing);
                double sn_bearing = sin(delta_bearing);

                // Sample pose differences
                delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
                delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
                delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);
                // Apply sampled update to particle pose
                sample->pose.v[0] += (delta_trans_hat * cs_bearing +
                delta_strafe_hat * sn_bearing);
                sample->pose.v[1] += (delta_trans_hat * sn_bearing -
                delta_strafe_hat * cs_bearing);
                sample->pose.v[2] = normalize(sample->pose.v[2] + delta_rot_hat);//+= delta_rot_hat ;
            }
        }
        break;
    case ODOM_MODEL_DIFF_CORRECTED:
        {
            // Implement sample_motion_odometry (Prob Rob p 136)
            double delta_rot1, delta_rot2;
            double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
            double trans_hat_stddev, rot1_hat_stddev, rot2_hat_stddev;
            double delta_rot1_noise, delta_rot2_noise;

            // Avoid computing a bearing from two poses that are extremely near each
            // other (happens on in-place rotation).
            if(delta_trans < 0.01)
            delta_rot1 = 0.0;
            else
            delta_rot1 = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),
            old_pose.v[2]);
            delta_rot2 = angle_diff(ndata->delta.v[2], delta_rot1);

            // We want to treat backward and forward motion symmetrically for the
            // noise model to be applied below.  The standard model seems to assume
            // forward motion.
            delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)),
            fabs(angle_diff(delta_rot1,M_PI)));
            delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)),
            fabs(angle_diff(delta_rot2,M_PI)));

            trans_hat_stddev = alpha3 * pow(delta_trans, 2)
            + alpha4 * (pow(delta_rot1_noise, 2) + pow(delta_rot2_noise, 2));
            trans_hat_stddev = sqrt(trans_hat_stddev);
            rot1_hat_stddev = alpha1 * pow(delta_rot1_noise, 2) + alpha2 * pow(delta_trans, 2);
            rot1_hat_stddev = sqrt(rot1_hat_stddev);
            rot2_hat_stddev = alpha1 * pow(delta_rot2_noise, 2) + alpha2 * pow(delta_trans, 2);
            rot2_hat_stddev = sqrt(rot2_hat_stddev);

            for (int i = 0; i < set->sample_count; i++)
            {
                pf_sample_t* sample = set->samples + i;

                // Sample pose differences
                delta_rot1_hat = angle_diff(delta_rot1, pf_ran_gaussian(rot1_hat_stddev));
                delta_trans_hat = delta_trans - pf_ran_gaussian(trans_hat_stddev);
                delta_rot2_hat = angle_diff(delta_rot2, pf_ran_gaussian(rot2_hat_stddev));

                // Apply sampled update to particle pose
                sample->pose.v[0] += delta_trans_hat *
                cos(sample->pose.v[2] + delta_rot1_hat);
                sample->pose.v[1] += delta_trans_hat *
                sin(sample->pose.v[2] + delta_rot1_hat);
                sample->pose.v[2] = normalize(sample->pose.v[2] + delta_rot1_hat + delta_rot2_hat);//+= delta_rot1_hat + delta_rot2_hat;
            }
        }
        break;
    }

    return true;
}
