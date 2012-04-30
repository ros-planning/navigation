/*********************************************************************
 *
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * Author: TKruse
 *********************************************************************/

#ifndef SIMPLE_SCORED_SAMPLING_PLANNER_H_
#define SIMPLE_SCORED_SAMPLING_PLANNER_H_

#include <vector>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/trajectory_search.h>

namespace base_local_planner {

/**
 * @class SimpleScoredSamplingPlanner
 * @brief Generates a local plan using the given generator and cost functions.
 * Assumes less cost are best, and negative costs indicate infinite costs
 *
 * This is supposed to be a simple and robust implementation of
 * the TrajectorySearch interface. More efficient search may well be
 * possible using search heuristics, parallel search, etc.
 */
class SimpleScoredSamplingPlanner : public base_local_planner::TrajectorySearch {
public:

  ~SimpleScoredSamplingPlanner() {}

  SimpleScoredSamplingPlanner() {}

  /**
   * Takes a list of generators and critics. Critics return costs > 0, or negative costs for invalid trajectories.
   * Generators other than the first are fallback generators,  meaning they only get to generate if the previous
   * generator did not find a valid trajectory.
   * Will use every generator until it stops returning trajectories or count reaches max_samples.
   * Then resets count and tries for the next in the list.
   * passing max_samples = -1 (default): Each Sampling planner will continue to call
   * generator until generator runs out of samples (or forever if that never happens)
   */
  SimpleScoredSamplingPlanner(std::vector<TrajectorySampleGenerator*> gen_list, std::vector<TrajectoryCostFunction*>& critics, int max_samples = -1) {
    max_samples_ = max_samples;
    gen_list_ = gen_list;
    critics_ = critics;
  }

  /**
   * runs all scoring functions over the trajectory creating a weigthed sum
   * of positive costs, or returns the first negative costs encountered if any.
   */
  double scoreTrajectory(Trajectory& traj) {
    double traj_cost = 0;
    for(std::vector<TrajectoryCostFunction*>::iterator score_function = critics_.begin(); score_function != critics_.end(); ++score_function) {
      TrajectoryCostFunction* score_function_p = *score_function;
      if (score_function_p->getScale() == 0) {
        continue;
      }
      double cost = score_function_p->scoreTrajectory(traj);
      if (cost < 0) {
        traj_cost = cost;
        break;
      }
      if (cost != 0) {
        cost *= score_function_p->getScale();
      }
      traj_cost += cost;
    }


    return traj_cost;
  }

  /**
   * Calls generator until generator has no more samples or max_samples is reached.
   * For each generated traj, calls critics in turn. If any critic returns negative
   * value, that value is assumed as costs, else the costs are the sum of all critics
   * result. Returns true and sets the traj parameter to the first trajectory with
   * minimal non-negative costs if sampling yields trajectories with non-negative costs,
   * else returns false.
   *
   * @param traj The container to write the result to
   * @param all_explored pass NULL or a container to collect all trajectories for debugging (has a penalty)
   */
  bool findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored) {
    Trajectory loop_traj;
    Trajectory best_traj;
    double loop_traj_cost, best_traj_cost = -1;
    bool gen_success;
    int count, count_valid;
    for (std::vector<TrajectoryCostFunction*>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic) {
      TrajectoryCostFunction* loop_critic_p = *loop_critic;
      if (loop_critic_p->prepare() == false) {
        ROS_WARN("A scoring function failed to prepare");
        return false;
      }
    }

    for (std::vector<TrajectorySampleGenerator*>::iterator loop_gen = gen_list_.begin(); loop_gen != gen_list_.end(); ++loop_gen) {
      count = 0;
      count_valid = 0;
      TrajectorySampleGenerator* gen_ = *loop_gen;
      while (gen_->hasMoreTrajectories()) {
        gen_success = gen_->nextTrajectory(loop_traj);
        if (gen_success == false) {
          // TODO use this for debugging
          continue;
        }
        loop_traj_cost = scoreTrajectory(loop_traj);
        if (all_explored != NULL) {
          loop_traj.cost_ = loop_traj_cost;
          all_explored->push_back(loop_traj);
        }

        if (loop_traj_cost >= 0) {
          count_valid++;
          if (best_traj_cost < 0 || loop_traj_cost < best_traj_cost) {
            best_traj_cost = loop_traj_cost;
            best_traj = loop_traj;
          }
        }
        count++;
        if (max_samples_ > 0 && count >= max_samples_) {
          break;
        }        
      }
      if (best_traj_cost >= 0) {
        traj.xv_ = best_traj.xv_;
        traj.yv_ = best_traj.yv_;
        traj.thetav_ = best_traj.thetav_;
        traj.cost_ = best_traj_cost;
        traj.resetPoints();
        double px, py, pth;
        for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
          best_traj.getPoint(i, px, py, pth);
          traj.addPoint(px, py, pth);
        }
      }
      //ROS_DEBUG("Evaluated %d trajectories, found %d valid", count, count_valid);
      if (best_traj_cost >= 0) {
        // do not try fallback generators
        break;
      }
    }
    return best_traj_cost >= 0;
  }


private:
  std::vector<TrajectorySampleGenerator*> gen_list_;
  std::vector<TrajectoryCostFunction*> critics_;

  int max_samples_;
};




} // namespace

#endif /* SIMPLE_SCORED_SAMPLING_PLANNER_H_ */
