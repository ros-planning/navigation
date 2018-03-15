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
/**************************************************************************
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf.c 6345 2008-04-17 01:36:39Z gerkey $
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "pf.h"
#include "pf_pdf.h"
#include "pf_kdtree.h"

double normalize(double z)
{
    return atan2(sin(z),cos(z));
}

double angle_diff(double a, double b)
{
    a = normalize(a);
    b = normalize(b);
    if (a < 0) a += 2*M_PI;
    if (b < 0) b += 2*M_PI;

    return normalize(a - b);
}

// Re-compute the cluster statistics for a sample set
void pf_calc_cluster_stats(pf_t *pf, pf_sample_set_t *set);

// Create a new filter
pf_t *pf_alloc(int min_samples, int max_samples,
    double alpha_slow, double alpha_fast,
    pf_init_model_fn_t random_pose_fn, void *random_pose_data,
    double resolution)
{
    int i, j;
    pf_t *pf;
    pf_sample_set_t *set;
    pf_sample_t *sample;

    srand48(time(NULL));

    pf = calloc(1, sizeof(pf_t));

    pf->random_pose_fn = random_pose_fn;
    pf->random_pose_data = random_pose_data;

    pf->min_samples = min_samples;
    pf->max_samples = max_samples;

    // Control parameters for the population size calculation.  [err] is
    // the max error between the true distribution and the estimated
    // distribution.  [z] is the upper standard normal quantile for (1 -
    // p), where p is the probability that the error on the estimated
    // distrubition will be less than [err].
    pf->pop_err = 0.01;
    pf->pop_z = 3;
    pf->dist_threshold = resolution*10.0;  // HACK is 10x res enough?

    pf->current_set = 0;
    for (j = 0; j < 2; j++)
    {
        set = pf->sets + j;

        set->sample_count = max_samples;
        set->samples = calloc(max_samples, sizeof(pf_sample_t));

        for (i = 0; i < set->sample_count; i++)
        {
            sample = set->samples + i;
            sample->pose.v[0] = 0.0;
            sample->pose.v[1] = 0.0;
            sample->pose.v[2] = 0.0;
            sample->weight = 1.0 / max_samples;
        }

        // HACK: is 3 times max_samples enough?
        set->kdtree = pf_kdtree_alloc(3 * max_samples, pf->dist_threshold/2.0, (5.0 * M_PI / 180.0)); // TODO suitable angular_resolution

        set->cluster_count = 0;
        set->cluster_max_count = max_samples;
        set->clusters = calloc(set->cluster_max_count, sizeof(pf_cluster_t));

        set->mean = pf_vector_zero();
        set->cov = pf_matrix_zero();
    }

    pf->w_slow = 0.0;
    pf->w_fast = 0.0;

    pf->alpha_slow = alpha_slow;
    pf->alpha_fast = alpha_fast;

    //set converged to 0
    pf_init_converged(pf);

    return pf;
}

// Free an existing filter
void pf_free(pf_t *pf)
{
    int i;

    for (i = 0; i < 2; i++)
    {
        free(pf->sets[i].clusters);
        pf_kdtree_free(pf->sets[i].kdtree);
        free(pf->sets[i].samples);
    }
    free(pf);

    return;
}

// Initialize the filter using a guassian
void pf_init(pf_t *pf, pf_vector_t mean, pf_matrix_t cov)
{
    int i;
    pf_sample_set_t *set;
    pf_sample_t *sample;
    pf_pdf_gaussian_t *pdf;

    set = pf->sets + pf->current_set;

    // Create the kd tree for adaptive sampling
    pf_kdtree_clear(set->kdtree);

    set->sample_count = pf->max_samples;

    assert(fabs(cov.m[0][0]) < 1e3 && fabs(cov.m[1][1]) < 1e3);
    pf->dist_threshold = cov.m[0][0] > cov.m[1][1] ? sqrt(cov.m[1][1]) : sqrt(cov.m[0][0]);
    printf("\n[pf_init] dist_threshold updated to %lf\n", pf->dist_threshold);
    pdf = pf_pdf_gaussian_alloc(mean, cov);

#ifdef BUILD_DEBUG
    printf("\n[pf_init] init mean = %lf, %lf, yaw %lf\n", mean.v[0], mean.v[1], mean.v[2]);

    for (int i = 0; i < 3; i++)
    {
        printf("\n[pf_init] pdf->cd.v[%d] = %lf\n", i, pdf->cd.v[i]);
    }
    for (int i = 0; i < 3; i++)
    {
        printf("\n[pf_init] pdf->x[%d] = %lf\n", i, pdf->x.v[i]);
    }
    printf("\n[pf_init] pdf->cr.m = \n");
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++) {
            printf("%lf\t", pdf->cr.m[i][j]);
        }
        printf("\n");
    }
#endif

    // Compute the new sample poses
    double standard_w = 1.0 / set->sample_count;
    for (i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;
        sample->weight = standard_w;
        sample->score = 1.0;
        sample->pose = pf_pdf_gaussian_sample(pdf);
    }

    pf->w_slow = pf->w_fast = 0.0;

    pf_pdf_gaussian_free(pdf);

    // Re-compute cluster statistics
    pf_calc_cluster_stats(pf, set);

#ifdef BUILD_DEBUG
    printf("\n[pf_init] Initialized mean = %lf, %lf, yaw %lf\n", set->mean.v[0], set->mean.v[1], set->mean.v[2]);
    printf("\t cov = \n");
    for (int r = 0; r < 3; ++r) {
        printf("\t\t");
        for (int c = 0; c < 3; ++c) {
            printf("%lf\t", set->cov.m[r][c]);
        }
        printf("\n");
    }
#endif

    //set converged to 0
    pf_init_converged(pf);

    return;
}


// Initialize the filter using some model
void pf_init_model(pf_t *pf, pf_init_model_fn_t init_fn, void *init_data)
{
    int i;
    pf_sample_set_t *set;
    pf_sample_t *sample;

    set = pf->sets + pf->current_set;

    // Create the kd tree for adaptive sampling
    pf_kdtree_clear(set->kdtree);

    set->sample_count = pf->max_samples;

    // Compute the new sample poses
    for (i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;
        sample->weight = 1.0 / set->sample_count;
        sample->pose = (*init_fn) (init_data);

        // Add sample to histogram
        pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
    }

    pf->w_slow = pf->w_fast = 0.0;

    // Re-compute cluster statistics
    pf_calc_cluster_stats(pf, set);

    //set converged to 0
    pf_init_converged(pf);

    return;
}

void pf_init_converged(pf_t *pf){
    pf_sample_set_t *set;
    set = pf->sets + pf->current_set;
    set->converged = 0;
    pf->converged = 0;
}

int pf_update_converged(pf_t *pf)
{
    int i;
    pf_sample_set_t *set;
    pf_sample_t *sample;
    double total;

    set = pf->sets + pf->current_set;
    double mean_x = 0, mean_y = 0, mean_weight = 0.0;

    for (i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;
        mean_x += sample->pose.v[0];
        mean_y += sample->pose.v[1];
        mean_weight += sample->weight;
    }
    mean_x /= (double)set->sample_count;
    mean_y /= (double)set->sample_count;
    mean_weight /= (double)set->sample_count;

    for (i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;
        if ( sample->weight <= mean_weight ) continue;
        if (fabs(sample->pose.v[0] - mean_x) > pf->dist_threshold
        || fabs(sample->pose.v[1] - mean_y) > pf->dist_threshold)
        {
            set->converged = 0;
            pf->converged = 0;
#ifdef BUILD_DEBUG
            double sample_wx = sample->weight / mean_weight;
            if (sample_wx > 1.01) printf("****** Not converged due to particles out of dist thresh! Particle # %d: %lf, %lf, yaw %lf, weighs %lf = %lf x avg \n",
                i, sample->pose.v[0], sample->pose.v[1], sample->pose.v[2], sample->weight, sample_wx);
#else
            return 0;
#endif
        }
    }

    if ( set->lost || set->score < exp(-0.5) ) set->converged = 0;
    else set->converged = 1;
    pf->converged = set->converged;
    return pf->converged;
}

// Update the filter with some new action
void pf_update_action(pf_t *pf, pf_action_model_fn_t action_fn, void *action_data)
{
    pf_sample_set_t *set;

    set = pf->sets + pf->current_set;

    (*action_fn) (action_data, set);

    return;
}


#include <float.h>
// Update the filter with some new sensor observation
void pf_update_sensor(pf_t *pf, pf_sensor_model_fn_t sensor_fn, void *sensor_data)
{
    int i;
    pf_sample_set_t *set;
    pf_sample_t *sample;
    pf_sample_t *heaviest_sample;

    set = pf->sets + pf->current_set;

    // Compute the sample weights
    double total = (*sensor_fn) (sensor_data, set);
    double effective_n_denom = 0;

    if (total > 0.0)
    {
        set->lost = 0;
        // Normalize weights
        double w_avg=0.0;
        heaviest_sample = set->samples;
        for (i = 0; i < set->sample_count; i++)
        {
            sample = set->samples + i;
            w_avg += sample->weight;
            sample->weight /= total;
            if (sample->weight > heaviest_sample->weight) heaviest_sample = sample;
            if (sample->weight < 0.0) printf("!!! Sample # %d weight < 0! %lf\n", i, sample->weight);
            effective_n_denom += pow(sample->weight, 2.0);
        }

        // Update running averages of likelihood of samples (Prob Rob p258)
        w_avg /= set->sample_count;

        pf->effective_sample_count = ceil(1.0/effective_n_denom);
#ifdef BUILD_DEBUG
        int curr_n = set->sample_count;
        int effective_n = (int)pf->effective_sample_count;
        float effective_perct = (float)effective_n/(float)curr_n*100.0;
        printf("~~~~~~ (%.3f %%) Effective number of particles = %d out of %d \n", effective_perct, effective_n, curr_n);
        int heaviest_id = heaviest_sample - set->samples;
        if (heaviest_sample->weight >= 0.99)
        {
            printf("^ heaviest particle # %d (%f, %f yaw %f) DOMINATES with score = %lf, weight = %.10e\n",
                heaviest_id,
                heaviest_sample->pose.v[0], heaviest_sample->pose.v[1], heaviest_sample->pose.v[2],
                heaviest_sample->score, heaviest_sample->weight);
        }
        else
        {
            int heavy_n = round(heaviest_sample->weight/(w_avg/total));
            printf("^ heaviest particle # %d (%f, %f yaw %f)\n\t scores %lf, weigh %d x avg (%f vs avg weight = %f)\n",
                heaviest_id,
                heaviest_sample->pose.v[0], heaviest_sample->pose.v[1], heaviest_sample->pose.v[2],
                heaviest_sample->score, heavy_n, heaviest_sample->weight, w_avg/total);
        }

#endif

        if (! pf->converged)
        {
            if (pf->w_slow == 0.0) pf->w_slow = w_avg;
            if (pf->w_fast == 0.0) pf->w_fast = w_avg;
        }

        pf->w_slow += pf->alpha_slow * (w_avg - pf->w_slow);
        pf->w_fast += pf->alpha_fast * (w_avg - pf->w_fast);
#ifdef BUILD_DEBUG
        double w_diff = 1.0 - pf->w_fast / pf->w_slow;
        if (! pf->converged) printf("[Resampling] w_avg: %e slow: %e fast: %e --> inject %% = %lf\n",w_avg, pf->w_slow, pf->w_fast, w_diff);
#endif
    }
    else
    {
        printf("[WARNING] !!!!!!!! Sum of weights = %f !!!!!!!!\n", total);
        // Handle zero total
        set->lost = 1; // this will make covariance infinity when calculating cluster stats
        double init_weight = 1.0 / set->sample_count;
        for (i = 0; i < set->sample_count; i++)
        {
            sample = set->samples + i;
            sample->weight = init_weight;
        }
    }

    return;
}

// Compute the CEP statistics (mean and variance).
void pf_get_cep_stats(pf_t *pf, pf_vector_t *mean, double *var)
{
    int i;
    double mn, mx, my, mrr;
    pf_sample_set_t *set;
    pf_sample_t *sample;

    set = pf->sets + pf->current_set;

    mn = 0.0;
    mx = 0.0;
    my = 0.0;
    mrr = 0.0;

    for (i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;

        mn += sample->weight;
        mx += sample->weight * sample->pose.v[0];
        my += sample->weight * sample->pose.v[1];
        mrr += sample->weight * sample->pose.v[0] * sample->pose.v[0];
        mrr += sample->weight * sample->pose.v[1] * sample->pose.v[1];
    }

    mean->v[0] = mx / mn;
    mean->v[1] = my / mn;
    mean->v[2] = 0.0;

    *var = mrr / mn - (mx * mx / (mn * mn) + my * my / (mn * mn));

    return;
}


// Get the statistics for a particular cluster.
int pf_get_cluster_stats(pf_t *pf, int clabel, double *weight,
    pf_vector_t *mean, pf_matrix_t *cov)
{
    pf_sample_set_t *set;
    pf_cluster_t *cluster;

    set = pf->sets + pf->current_set;
    if (clabel >= set->cluster_count)
    return 0;
    cluster = set->clusters + clabel;

    *weight = cluster->weight;
    *mean = cluster->mean;
    *cov = cluster->cov;

    return 1;
}

// Re-compute the cluster statistics for a sample set
void pf_calc_cluster_stats(pf_t *pf, pf_sample_set_t *set)
{
    int i, j, k, clusterdx;
    pf_sample_t *sample;
    pf_cluster_t *cluster;

    // Workspace
    double m[4], c[3][3];
    size_t count;
    double weight;

    // Re-initialize new kd-tree since particles have been updated with robot motion and measurements
    pf_kdtree_clear(set->kdtree);
    for (i = 0; i < set->sample_count; ++i)
    {
        sample = set->samples + i;
        pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
    }

    // Cluster the samples
    pf_kdtree_cluster(set->kdtree);

    // Initialize cluster stats
    set->cluster_count = 0;

    for (i = 0; i < set->cluster_max_count; i++)
    {
        cluster = set->clusters + i;
        cluster->count = 0;
        cluster->weight = 0;
        cluster->mean = pf_vector_zero();
        cluster->cov = pf_matrix_zero();
        cluster->score = 0;

        for (j = 0; j < 4; j++)
        cluster->m[j] = 0.0;
        for (j = 0; j < 3; j++)
        for (k = 0; k < 3; k++)
        cluster->c[j][k] = 0.0;
    }

    // Initialize overall filter stats
    count = 0;
    weight = 0.0;
    set->mean = pf_vector_zero();
    set->cov = pf_matrix_zero();
    set->score = 0.0;
    for (j = 0; j < 4; j++)
    m[j] = 0.0;
    for (j = 0; j < 3; j++)
    for (k = 0; k < 3; k++)
    c[j][k] = 0.0;

    // Accumulate overall and cluster means respectively
    for (i = 0; i < set->sample_count; ++i)
    {
        sample = set->samples + i;
        if (! sample->weight) continue;

        set->score += sample->score * sample->weight;

        // Get the cluster label for this sample
        clusterdx = pf_kdtree_get_cluster(set->kdtree, sample->pose);
        assert(clusterdx >= 0);
        if (clusterdx >= set->cluster_max_count) continue;
        if (clusterdx + 1 > set->cluster_count) set->cluster_count = clusterdx + 1;

        cluster = set->clusters + clusterdx;

        cluster->count += 1;
        cluster->weight += sample->weight;
        cluster->score += sample->score * sample->weight;

        count += 1;
        weight += sample->weight;

        cluster->m[0] += sample->weight * sample->pose.v[0];
        cluster->m[1] += sample->weight * sample->pose.v[1];
        cluster->m[2] += sample->weight * cos(sample->pose.v[2]);
        cluster->m[3] += sample->weight * sin(sample->pose.v[2]);
    }

    // Initialize weighted sample covariance denomenator, and copy over clusters' weighted sums so far into set's weighted sum
    double cov_denom[set->cluster_count];
    for (i = 0; i < set->cluster_count; ++i)
    {
        cluster = set->clusters + i;
        for (j = 0; j < 4; ++j)
        {
            m[j] += cluster->m[j];
        }
        cov_denom[i] = 0;
    }
    double set_cov_denom = 0;

    // Compute weighted sample covariance denomenator
    for (i = 0; i < set->sample_count; ++i)
    {
        sample = set->samples + i;
        if (! sample->weight) continue;

        clusterdx = pf_kdtree_get_cluster(set->kdtree, sample->pose);
        assert(clusterdx >= 0);
        if (clusterdx >= set->cluster_max_count)
        continue;
        if (clusterdx + 1 > set->cluster_count)
        set->cluster_count = clusterdx + 1;

        cluster = set->clusters + clusterdx;
        cov_denom[clusterdx] += pow(sample->weight, 2.0);
    }

    // Compuate clusters mean
    for (i = 0; i < set->cluster_count; i++)
    {
        cluster = set->clusters + i;

        if (! cluster->weight) continue;
        set_cov_denom += cov_denom[i];

        cov_denom[i] = 1.0 / (cluster->weight - (cov_denom[i]/cluster->weight));

        for (j = 0; j < 2; ++j)
        {
            cluster->m[j] /= cluster->weight; // FIXME? Or not
        }

        cluster->mean.v[0] = cluster->m[0];
        cluster->mean.v[1] = cluster->m[1];
        cluster->mean.v[2] = atan2(cluster->m[3], cluster->m[2]);
        cluster->score /= cluster->weight;
    }

    // NOTE FIXED calculate cluster covariance AFTER mean

    // Accumulate coefficients to calculate cluster mean
    for (i = 0; i < set->sample_count; i++)
    {
        sample = set->samples + i;

        // Get the cluster label for this sample
        clusterdx = pf_kdtree_get_cluster(set->kdtree, sample->pose);
        assert(clusterdx >= 0);
        if (clusterdx >= set->cluster_max_count)
        continue;
        if (clusterdx + 1 > set->cluster_count)
        set->cluster_count = clusterdx + 1;

        cluster = set->clusters + clusterdx;

        for (j = 0; j < 2; j++)
        for (k = 0; k < 2; k++)
        {
            double var = sample->weight * (sample->pose.v[j] - cluster->mean.v[j])*(sample->pose.v[k] - cluster->mean.v[k]);
            cluster->cov.m[j][k] += var;
        }
        double var_angle = sample->weight * pow(angle_diff(sample->pose.v[2], cluster->mean.v[2]), 2);
        cluster->cov.m[2][2] += var_angle;
    }

    // Calculate cluster covariance, and accumulate coefficients to calculate overall covariance
#ifdef BUILD_DEBUG
    printf("~~~~~~~~~~ %d clusters ~~~~~~~~~~\n", set->cluster_count);
#endif
    for (i = 0; i < set->cluster_count; ++i)
    {
        cluster = set->clusters + i;

        if (! cluster->weight) continue;
        if ( cluster->score == 0.0 ) cov_denom[i] = INFINITY;
        else cov_denom[i] /= cluster->score;

        for (j = 0; j < 3; ++j )
        for (k = 0; k < 3; ++k )
        {
            if (cluster->cov.m[j][k] == 0) continue;
            c[j][k] += cluster->cov.m[j][k];
            cluster->cov.m[j][k] *= cov_denom[i];
        }
    }

    // Compute overall filter stats
    set->mean.v[0] = m[0] / weight;
    set->mean.v[1] = m[1] / weight;
    set->mean.v[2] = atan2(m[3], m[2]);
    set->score /= weight;

    if (set_cov_denom/weight != weight) set_cov_denom = 1.0 / (weight - (set_cov_denom/weight));

#ifdef BUILD_DEBUG
    else printf("set_cov_denom/weight = %lf ----> 1.0 / (weight - (set_cov_denom/weight) = inf!", set_cov_denom/weight);
    printf("set : cov_denom = %lf\n", set_cov_denom);
#endif

    if ( set->lost || set->score == 0.0 ) set_cov_denom = INFINITY;
    else set_cov_denom /= set->score;

    for (j = 0; j < 3; ++j)
    for (k = 0; k < 3; ++k)
    set->cov.m[j][k] = c[j][k]*set_cov_denom;

    pf_update_converged(pf);

    return;
}
