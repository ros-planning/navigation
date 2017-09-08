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
#include <srslib_timing/ScopedTimingSampleRecorder.hpp>

#include <base_local_planner/simple_scored_sampling_planner.h>
#include <base_local_planner/ScoredSamplingPlannerCosts.h>

#include <ros/console.h>

namespace base_local_planner {

  SimpleScoredSamplingPlanner::SimpleScoredSamplingPlanner(std::vector<TrajectorySampleGenerator*> gen_list, std::vector<TrajectoryCostFunction*>& critics, int max_samples)
  : tdr_("SSSP") {
    max_samples_ = max_samples;
    gen_list_ = gen_list;
    critics_ = critics;
    ros::NodeHandle n("/");
    cost_publisher_ = n.advertise<ScoredSamplingPlannerCosts>("local_planner_costs", 100);

  }

  double SimpleScoredSamplingPlanner::scoreTrajectory(Trajectory& traj, double best_traj_cost,
    CriticCosts* msg) {
    srs::StopWatch stopWatch;
    critic_costs_.assign(critics_.size(), 0);
    if (msg != nullptr)
    {
      msg->costs.assign(critics_.size(), -20);
      msg->v = traj.xv_;
      msg->w = traj.thetav_;
    }

    double traj_cost = traj.cost_;
    int gen_id = 0;
    int k = -1;
    for(std::vector<TrajectoryCostFunction*>::iterator score_function = critics_.begin(); score_function != critics_.end(); ++score_function) {
      stopWatch.reset();
      k++;
      TrajectoryCostFunction* score_function_p = *score_function;
      if (score_function_p->getScale() == 0) {
        if (msg != nullptr)
        {
          msg->costs[k] = 0.0;
        }
        if (critic_timing_.size() > k)
        {
          critic_timing_[k] += stopWatch.elapsedMilliseconds();
        }
        continue;
      }
      double cost = score_function_p->scoreTrajectory(traj);
      if (cost < 0) {
        ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf discarded by cost function  %d with cost: %f", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost);
        traj_cost = cost;

        if (critic_timing_.size() > k)
        {
          critic_timing_[k] += stopWatch.elapsedMilliseconds();
        }
        if (critic_costs_.size() > k)
        {
          critic_costs_[k] = cost;
        }
        if (msg != nullptr)
        {
          msg->costs[k] = cost;
        }
        break;
      }
      if (cost != 0) {
        cost *= score_function_p->getScale();
      }

      if (msg != nullptr)
      {
        msg->costs[k] = cost;
      }

      critic_costs_[k] = cost;
      traj_cost += cost;
      if (best_traj_cost > 0) {
        // since we keep adding positives, once we are worse than the best, we will stay worse
        if (traj_cost > best_traj_cost) {
          if (critic_timing_.size() > k)
          {
            critic_timing_[k] = stopWatch.elapsedMilliseconds();
          }
          break;
        }
      }
      if (critic_timing_.size() > k)
      {
        critic_timing_[k] += stopWatch.elapsedMilliseconds();
      }
      gen_id ++;
    }

    if (msg != nullptr)
    {
      msg->total_cost = traj_cost;
    }

    return traj_cost;
  }

  bool SimpleScoredSamplingPlanner::findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored) {
    Trajectory loop_traj;
    Trajectory best_traj;
    double loop_traj_cost, best_traj_cost = -1;
    bool gen_success;
    int count, count_valid;

    ScoredSamplingPlannerCosts cost_msg;
    cost_msg.header.stamp = ros::Time::now();
    cost_msg.num_critics = critics_.size();

    srs::ScopedTimingSampleRecorder stsr(tdr_.getRecorder("-preparingCritics"));
    for (std::vector<TrajectoryCostFunction*>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic) {
      TrajectoryCostFunction* loop_critic_p = *loop_critic;
      if (loop_critic_p->prepare() == false) {
        ROS_DEBUG("A scoring function failed to prepare");
        return false;
      }
    }
    stsr.stopSample();

    srs::ScopedTimingSampleRecorder stsr2(tdr_.getRecorder("-scoring trajectories"));
    critic_timing_.assign(critics_.size(), 0);
    srs::StopWatch stopWatch;
    for (std::vector<TrajectorySampleGenerator*>::iterator loop_gen = gen_list_.begin(); loop_gen != gen_list_.end(); ++loop_gen) {
      cost_msg.start_v = (*loop_gen)->getStartLinearVelocity();
      cost_msg.start_w = (*loop_gen)->getStartAngularVelocity();

      count = 0;
      count_valid = 0;
      TrajectorySampleGenerator* gen_ = *loop_gen;
      while (gen_->hasMoreTrajectories()) {
        gen_success = gen_->nextTrajectory(loop_traj);
        if (gen_success == false) {
          // TODO use this for debugging
          continue;
        }
        CriticCosts critic_costs_msg;
        loop_traj_cost = scoreTrajectory(loop_traj, best_traj_cost, &critic_costs_msg);
        cost_msg.costs.push_back(critic_costs_msg);

        if (all_explored != NULL) {
          loop_traj.cost_ = loop_traj_cost;
          all_explored->push_back(loop_traj);
        }

        if (loop_traj_cost >= 0) {
          count_valid++;
          if (best_traj_cost < 0 || loop_traj_cost < best_traj_cost) {
            std::stringstream ss;
            ss << "New best: " << loop_traj_cost << " with critics: ";
            for (auto cost : critic_costs_)
            {
              ss << cost << ", ";
            }
            ss << " [v,w] - [" << loop_traj.xv_ << ", " << loop_traj.thetav_ << "]";
            ROS_DEBUG("%s", ss.str().c_str());

            cost_msg.selected_idx = count;
            best_traj_cost = loop_traj_cost;
            best_traj = loop_traj;
          } else if (loop_traj_cost == best_traj_cost) {
            std::stringstream ss;
            ss << "Tied best: " << loop_traj_cost << " with critics: ";
            for (auto cost : critic_costs_)
            {
              ss << cost << ", ";
            }
            ss << " [v,w] - [" << loop_traj.xv_ << ", " << loop_traj.thetav_ << "]";
            ROS_DEBUG("%s", ss.str().c_str());          }
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
        traj.time_delta_ = best_traj.time_delta_;
        traj.resetPoints();
        double px, py, pth;
        for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
          best_traj.getPoint(i, px, py, pth);
          traj.addPoint(px, py, pth);
        }
      }
      ROS_DEBUG("Evaluated %d trajectories, found %d valid", count, count_valid);
    }

    cost_msg.selected_v = traj.xv_;
    cost_msg.selected_w = traj.thetav_;

    cost_publisher_.publish(cost_msg);

    std::stringstream ss;
    ss << "Critical timing: total:  " << stopWatch.elapsedMilliseconds() << std::endl;
    for (size_t k = 0; k < critic_timing_.size(); ++k)
    {
      ss << " Critic " << k << ": " << critic_timing_[k] << std::endl;
    }
    ROS_DEBUG("%s", ss.str().c_str());

    return best_traj_cost >= 0;
  }


}// namespace
