/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <dwa_local_planner/dwa_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/map_grid_cost_point.h>
#include <cmath>

#include <visualization_msgs/Marker.h>

//for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>

#define PI 3.14159

#include <pcl_conversions/pcl_conversions.h>

namespace dwa_local_planner {

    void DWAPlanner::checkValid(const Eigen::Vector3f& vel, const base_local_planner::LocalPlannerLimits& limits, base_local_planner::Trajectory& traj)
    {
        if (traj.cost_ >= 0)
            return;

        double v0 = fabs(vel[0]);
        double v1 = fabs(vel[1]);

        if (vel[0] > 0) traj.xv_ = std::max(0.0, vel[0] - (v0/(v0+v1)) * limits.acc_limit_trans * sim_period_);
        if (vel[0] < 0) traj.xv_ = std::min(0.0, vel[0] + (v0/(v0+v1)) * limits.acc_limit_trans * sim_period_);

        if (vel[1] > 0) traj.yv_ = std::max(0.0, vel[1] - (v1/(v0+v1)) * limits.acc_limit_trans * sim_period_);
        if (vel[1] < 0) traj.yv_ = std::min(0.0, vel[1] + (v1/(v0+v1)) * limits.acc_limit_trans * sim_period_);

        if (vel[2] > 0) traj.thetav_ = std::max(0.0, vel[2] - limits.acc_lim_theta * sim_period_);
        if (vel[2] < 0) traj.thetav_ = std::min(0.0, vel[2] + limits.acc_lim_theta * sim_period_);

        ROS_WARN_STREAM("DWA PLANNER DISCARDED ALL TRAJECTORIES, DEACCELERATING WITH MAX; COST: " << traj.cost_ << "\n"
                        << "     vx: " << vel[0] << " --> " << traj.xv_ << "\n"
                        << "     vy: " << vel[1] << " --> " << traj.yv_ << "\n"
                        << "     vth: " << vel[2] << " --> " <<  traj.thetav_ );
    }

    void DWAPlanner::reconfigure(DWAPlannerConfig &config)
    {
        boost::mutex::scoped_lock l(configuration_mutex_);

        //! Configure the trajectory generator
        vsamples_[0] = config.vx_samples;
        vsamples_[1] = config.vy_samples;
        vsamples_[2] = config.vth_samples;
        sim_period_ = config.sim_period;
        generator_.setParameters(config.sim_time, config.sim_granularity, config.angular_sim_granularity, config.use_dwa, config.sim_period);

        ROS_INFO_STREAM("Trajectory Generator configured:\n"
                        << "    - Samples [x,y,th] : [" << (int)vsamples_[0] << "," << (int)vsamples_[1] << "," << (int)vsamples_[2] << "]\n"
                        << "    - Simulation time : " << config.sim_time << " [seconds]\n"
                        << "    - Simulation period : " << config.sim_period << " [seconds]\n"
                        << "    - Use DWA : " << config.use_dwa << " [-]\n"
                        << "    - Granularity : " << config.sim_granularity << " [m]\n"
                        << "    - Angular granularity : " << config.angular_sim_granularity << " [rad]\n");

        //! Configure the switches
        switch_yaw_error_ = config.switch_yaw_error;
        switch_goal_distance_ = config.switch_goal_distance;
        switch_plan_distance_ = config.switch_plan_distance;

        ROS_INFO_STREAM("Switches configured:\n"
                        << "    - Yaw error : " << config.switch_yaw_error << " [rad]\n"
                        << "    - Goal distance : " << config.switch_goal_distance << " [m]\n"
                        << "    - Plan distance : " << config.switch_plan_distance << " [m]\n");

        //! Set parameters for occupancy velocity costfunction
        occ_vel_costs_.setParams(config.max_trans_vel);
    }

    DWAPlanner::DWAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util) :
        planner_util_(planner_util),
        occ_vel_costs_(planner_util->getCostmap()),
        plan_costs_(planner_util->getCostmap()),
        goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
        vis_(planner_util->getCostmap(), goal_costs_, plan_costs_, planner_util->getGlobalFrame())
    {
        // Costfunctions
        std::vector<base_local_planner::TrajectoryCostFunction*> critics;
        critics.push_back(&occ_vel_costs_);
        critics.push_back(&plan_costs_);
        critics.push_back(&goal_costs_);
        critics.push_back(&alignment_costs_);
//        critics.push_back(&cmd_vel_costs_);

        // trajectory generator
        std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
        generator_list.push_back(&generator_);
        scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);
    }

    LocalPlannerState DWAPlanner::determineState(double yaw_error, double plan_distance, double goal_distance)
    {
        static LocalPlannerState prev_state = None;
        LocalPlannerState state = Default;

        // todo: optional path_distance state

        // ToDo: can we make this more generic???
        if (goal_distance < switch_goal_distance_)
            state = Arrive;
        else if (fabs(yaw_error) > switch_yaw_error_ || ( prev_state == Align && fabs(yaw_error) > (switch_yaw_error_/2) ) )
            state = Align;
        else
            state = Default;

        // To print state
        if (prev_state != state)
        {
            ROS_INFO_STREAM("State = " << StateName[state]);
            prev_state = state;
        }

        return state;
    }

    void DWAPlanner::updatePlanAndLocalCosts(tf::Stamped<tf::Pose> robot_pose, const std::vector<geometry_msgs::PoseStamped>& local_plan, const std::vector<geometry_msgs::Point>& footprint_spec)
    {
        /// Determine the errors
        double yaw_error     = base_local_planner::getGoalOrientationAngleDifference(robot_pose, tf::getYaw(local_plan.front().pose.orientation));
        double plan_distance = base_local_planner::getGoalPositionDistance(robot_pose, local_plan.front().pose.position.x, local_plan.front().pose.position.y);
        double goal_distance = base_local_planner::getGoalPositionDistance(robot_pose, local_plan.back().pose.position.x, local_plan.back().pose.position.y);

        /// Determine state of the controller
        LocalPlannerState state = determineState(yaw_error, plan_distance, goal_distance);

        /// Update the cost functions depending on the state we are in
        switch (state)
        {
        case Default:
            goal_costs_.setScale(1.0);
            plan_costs_.setScale(0.0);
            alignment_costs_.setDesiredOrientation(tf::getYaw(local_plan.front().pose.orientation));

            break;

        case Arrive:
            goal_costs_.setScale(1.0);
            plan_costs_.setScale(0.0);
            alignment_costs_.setDesiredOrientation(tf::getYaw(local_plan.back().pose.orientation));

            break;

        case Align:
            goal_costs_.setScale(0.0);
            plan_costs_.setScale(1.0);
            alignment_costs_.setDesiredOrientation(tf::getYaw(local_plan.front().pose.orientation));

            break;
        }

        //! Optimization data (Set local plan)
        goal_costs_.setTargetPoses(local_plan);
        plan_costs_.setTargetPoses(local_plan);

        //! Update footprint if changed
        occ_vel_costs_.setFootprint(footprint_spec);
    }

    base_local_planner::Trajectory DWAPlanner::findBestPath(tf::Stamped<tf::Pose> robot_pose, tf::Stamped<tf::Pose> robot_vel, tf::Stamped<tf::Pose> goal_pose)
    {
        //make sure that our configuration doesn't change mid-run
        boost::mutex::scoped_lock l(configuration_mutex_);

        // Setup the variables for traj generation
        Eigen::Vector3f pos(robot_pose.getOrigin().getX(), robot_pose.getOrigin().getY(), tf::getYaw(robot_pose.getRotation()));
        Eigen::Vector3f vel(robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), tf::getYaw(robot_vel.getRotation()));
        Eigen::Vector3f goal(goal_pose.getOrigin().getX(), goal_pose.getOrigin().getY(), tf::getYaw(goal_pose.getRotation()));
        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

        // prepare cost functions and generators for this run
        generator_.initialise(pos, vel, goal, &limits, vsamples_, false);

        // find best trajectory by sampling and scoring the samples
        base_local_planner::Trajectory result_traj;
        std::vector<base_local_planner::Trajectory> all_explored;
        scored_sampling_planner_.findBestTrajectory(result_traj, &all_explored);

        //! Visualization
        vis_.publishDesiredOrientation(alignment_costs_.getDesiredOrientation(), robot_pose);
        vis_.publishCostGrid();
        vis_.publishTrajectoryCloud(all_explored);

        //! Check valid, otherwise deaccelerate with max
        checkValid(vel, limits, result_traj);

        return result_traj;
    }
}
