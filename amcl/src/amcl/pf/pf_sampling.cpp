#include "pf.h"
#include "pf_pdf.h"
#include "pf_kdtree.h"

#include <random>
#include <cmath>
#include <vector>
#include <assert.h>

// Compute the required number of samples, given that there are k bins
// with samples in them.
static int pf_resample_limit(pf_t *pf, int k);

// // Re-compute the cluster statistics for a sample set
extern "C" void pf_calc_cluster_stats(pf_t *pf, pf_sample_set_t *set);

#ifdef BUILD_DEBUG
int proposed_up;
#endif

// Compute the required number of samples, given that there are k bins
// with samples in them.  This is taken directly from Fox et al.
int pf_resample_limit(pf_t *pf, int k)
{
    int n = 0;
    pf_sample_set_t *curr_set = pf->sets + pf->current_set;
    int curr_n = curr_set->sample_count;

    if (k <= 1)
    {
        double score = curr_set->score;
        int limit;
        if (score < exp(-0.5) || ! pf->converged)
        {
            limit = pf_resample_limit(pf, k+1);
            limit = ceil((double)limit/score);
        }
        else limit = pf->min_samples;
        n = ceil(((float) curr_n + (float) limit)/2.0);
    }

    if (! n)
    {
        double a, b, c, x, y, k_1;

        k_1 = (double)(k - 1);
        a = 1;
        b = 2.0 / (9.0 * k_1);
        c = sqrt(b) * pf->pop_z;
        x = a - b + c;
        y = k_1/(2.0*pf->pop_err);

        n = (int) ceil(y * pow(x,3.0));
    }

#ifdef BUILD_DEBUG
    if (n >= (float)pf->max_samples * 1.5)// || n <= (float)pf->min_samples/2.0)
    {
        proposed_up = n;
    }
#endif

    if (n < pf->effective_sample_count) n = pf->effective_sample_count; // DEBUG improper downsampling

    if (n < pf->min_samples)
    return pf->min_samples;
    if (n > pf->max_samples)
    return pf->max_samples;

    return n;
}

/*
* Resample the particles with probabilities proportional to particle weights,
* using KLD sampling, according to Table 1: KLD-Sampling Algorithm
* in "KLD-Sampling: Adaptive Particle Filters" by Dieter Fox
*/
void pf_update_resample(pf_t *pf)
{
    double total;
    pf_sample_set_t *set_a, *set_b;
    pf_sample_t *sample_a, *sample_b;

    double w_diff = 0.0;

    set_a = pf->sets + pf->current_set; // particle set at time = t
    set_b = pf->sets + (pf->current_set + 1) % 2; // new particle set for time = t+1 (will be updated with motion update when control comes in)

    // Create the kd tree for adaptive sampling
    pf_kdtree_clear(set_b->kdtree);

    // Draw samples from set a to create set b.
    total = 0;
    set_b->sample_count = 0;

    if (pf->w_slow > 0.0) w_diff = 1.0 - pf->w_fast / pf->w_slow;
    bool inject_random = (w_diff > 0.0);
#ifdef BUILD_DEBUG
    if (inject_random) printf("[Resampling] Injecting random particles with probability of %lf\n", w_diff);
#endif

    // initialize discrete distribution with sample weights at time = t
    std::vector<double> weights;
    try
    {
        assert(set_a->sample_count > 0);
        weights.resize(set_a->sample_count);
    }
    catch (...)
    {
        throw ("Failed to construct vector of %d weights during resampling\n", set_a->sample_count);
        return;
    }

    for (size_t i = 0; i < set_a->sample_count; ++i)
    {
        sample_a = set_a->samples + i;
        assert(! std::isnan(sample_a->weight));
        weights[i] = sample_a->weight;
    }
    std::discrete_distribution<> weighted_particles(weights.begin(), weights.end());
    std::random_device rd;
    std::mt19937 gen(rd());

#ifdef BUILD_DEBUG
    proposed_up = 0;
#endif

    int resample_limit = pf->max_samples;
    do
    {
        sample_b = set_b->samples + set_b->sample_count++;
        if (! inject_random || drand48() >= w_diff)
        {
            int sample_idx = weighted_particles(gen); // sample an index from the discrete_distribution given by weights in particle set t
            assert(sample_idx >=0 && sample_idx < set_a->sample_count);
            sample_a = set_a->samples + sample_idx;
            sample_b->pose = sample_a->pose;
        }
        else
        {
            sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);
        }

        sample_b->weight = 1.0;
        total += sample_b->weight;

        int prev_leaf_count = set_b->kdtree->leaf_count;
        // Add sample to histogram
        pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

        if (set_b->kdtree->leaf_count != prev_leaf_count) // same leaft_count could NOT have lead to different resample_limit
        {
            resample_limit = pf_resample_limit(pf, set_b->kdtree->leaf_count);
        }

        // See if we have enough samples yet
    } while(set_b->sample_count < resample_limit);

#ifdef BUILD_DEBUG
    if (set_a->sample_count != set_b->sample_count)
    {
        printf("********************************************* resampled %d -> %d as leaf count %d -> %d\n", set_a->sample_count, set_b->sample_count, set_a->kdtree->leaf_count, set_b->kdtree->leaf_count);
    }
    else if (proposed_up)
    {
        printf("Proposed to UPSAMPLE to %d particles\n", proposed_up);
    }
#endif

    // Reset averages, to avoid spiraling off into complete randomness.
    if(w_diff > 0.0)
    pf->w_slow = pf->w_fast = 0.0;

    // Normalize weights
    for (size_t i = 0; i < set_b->sample_count; ++i)
    {
        sample_b = set_b->samples + i;
        sample_b->weight /= total;
    }

    // Use the newly created sample set
    pf->current_set = (pf->current_set + 1) % 2;

    return;
}
