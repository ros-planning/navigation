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

//for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>

#define PI 3.14159

#include <pcl_conversions/pcl_conversions.h>

namespace dwa_local_planner {

    void DWAPlanner::publishTrajectoryCloud(const std::vector<base_local_planner::Trajectory>& trajectories)
    {
        pcl::PointCloud<base_local_planner::MapGridCostPoint> traj_cloud;
        traj_cloud.header.frame_id = planner_util_->getGlobalFrame();

        base_local_planner::MapGridCostPoint pt;
        traj_cloud.width = 0;
        traj_cloud.height = 0;
        std_msgs::Header header;
        pcl_conversions::fromPCL(traj_cloud.header, header);
        header.stamp = ros::Time::now();
        traj_cloud.header = pcl_conversions::toPCL(header);
        for(std::vector<base_local_planner::Trajectory>::const_iterator t=trajectories.begin(); t != trajectories.end(); ++t)
        {
            if(t->cost_<0)
                continue;
            // Fill out the plan
            for(unsigned int i = 0; i < t->getPointsSize(); ++i) {
                double p_x, p_y, p_th;
                t->getPoint(i, p_x, p_y, p_th);
                pt.x=p_x;
                pt.y=p_y;
                pt.z=0;
                pt.path_cost=p_th;
                pt.total_cost=t->cost_;
                traj_cloud.push_back(pt);
            }
        }
        traj_cloud_pub_.publish(traj_cloud);
    }

    void DWAPlanner::reconfigure(DWAPlannerConfig &config)
    {

        boost::mutex::scoped_lock l(configuration_mutex_);

        generator_.setParameters(config.sim_time, config.sim_granularity, config.angular_sim_granularity, config.use_dwa, sim_period_);

        double resolution = planner_util_->getCostmap()->getResolution();
        pdist_scale_ = config.path_distance_bias;
        // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
        path_costs_.setScale(resolution * pdist_scale_ * 0.5);
        alignment_costs_.setScale(resolution * pdist_scale_ * 0.5);

        gdist_scale_ = config.goal_distance_bias;
        goal_costs_.setScale(resolution * gdist_scale_ * 0.5);
        //    goal_front_costs_.setScale(resolution * gdist_scale_ * 0.5);

        occdist_scale_ = config.occdist_scale;
        obstacle_costs_.setScale(resolution * occdist_scale_);

        stop_time_buffer_ = config.stop_time_buffer;

        alignment_costs_.setScale(config.orientation_bias);
        cmd_vel_costs_.setCoefficients(config.pen_pos_x, config.pen_neg_x, config.pen_pos_y, config.pen_neg_y, config.pen_pos_theta, config.pen_neg_theta);
        cmd_vel_costs_.setScale(1.0);

        /// Lookahead distance is the maximum that can be obtained in one simulation, the 'ideal' situation
        lookahead_dist_ = config.max_trans_vel * config.sim_time;

        /// Switch distance
        switch_dist_ = config.switch_distance;

        // obstacle costs can vary due to scaling footprint feature
        obstacle_costs_.setParams(config.max_trans_vel, config.max_scaling_factor, config.scaling_speed);

        int vx_samp, vy_samp, vth_samp;
        vx_samp = config.vx_samples;
        vy_samp = config.vy_samples;
        vth_samp = config.vth_samples;

        if (vx_samp <= 0) {
            ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
            vx_samp = 1;
            config.vx_samples = vx_samp;
        }

        if (vy_samp <= 0) {
            ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
            vy_samp = 1;
            config.vy_samples = vy_samp;
        }

        if (vth_samp <= 0) {
            ROS_WARN("You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to sample one value... so we're going to set vth_samples to 1 instead");
            vth_samp = 1;
            config.vth_samples = vth_samp;
        }

        vsamples_[0] = vx_samp;
        vsamples_[1] = vy_samp;
        vsamples_[2] = vth_samp;
    }

    DWAPlanner::DWAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util) :
        planner_util_(planner_util),
        obstacle_costs_(planner_util->getCostmap()),
        path_costs_(planner_util->getCostmap()),
        goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true)
        //    ,
        //      goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
        //      alignment_costs_(planner_util->getCostmap())
    {
        ros::NodeHandle private_nh("~/" + name);

        //    goal_front_costs_.setStopOnFailure( false );
        //    alignment_costs_.setStopOnFailure( false );

        //Assuming this planner is being run within the navigation stack, we can
        //just do an upward search for the frequency at which its being run. This
        //also allows the frequency to be overwritten locally.
        std::string controller_frequency_param_name;
        if (!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
        {
            sim_period_ = 0.05;
        }
        else
        {
            std::cout << "Found controller frequency @ " << controller_frequency_param_name << std::endl;
            double controller_frequency = 0;
            private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
            std::cout << "frequency found: " << controller_frequency << std::endl;
            if(controller_frequency > 0)
            {
                sim_period_ = 1.0 / controller_frequency;
            }
            else
            {
                ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
                sim_period_ = 0.05;
            }
        }
        ROS_INFO("Sim period is set to %.2f", sim_period_);

        //    oscillation_costs_.resetOscillationFlags();

        bool sum_scores;
        private_nh.param("sum_scores", sum_scores, false);
        obstacle_costs_.setSumScores(sum_scores);

        private_nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);

        std::string frame_id;
        private_nh.param("global_frame_id", frame_id, std::string("odom"));

        traj_cloud_pub_.advertise(private_nh, "trajectory_cloud", 1);

        // set up all the cost functions that will be applied in order
        // (any function returning negative values will abort scoring, so the order can improve performance)
        std::vector<base_local_planner::TrajectoryCostFunction*> critics;
        //    critics.push_back(&oscillation_costs_); // discards oscillating motions (assisgns cost -1)
        critics.push_back(&obstacle_costs_); // discards trajectories that move into obstacles
        //    critics.push_back(&goal_front_costs_); // prefers trajectories that make the nose go towards (local) nose goal
        critics.push_back(&alignment_costs_); // prefers trajectories that keep the robot nose on nose path
        critics.push_back(&path_costs_); // prefers trajectories on global path
        critics.push_back(&goal_costs_); // prefers trajectories that go towards (local) goal, based on wave propagation

        // trajectory generators
        std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
        generator_list.push_back(&generator_);

        scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

        private_nh.param("cheat_factor", cheat_factor_, 1.0);
    }

    geometry_msgs::PoseStamped getCurrentPlanPose(const tf::Stamped<tf::Pose>& robot_pose, const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        assert(plan.size() > 1);

        /// Find current index of the plan
        double min_plan_sq_dist = 1e6;
        int current_waypoint_index = -1;

        for(unsigned int i = 0; i < plan.size(); ++i) {
            double sq_plan_dist = (robot_pose.getOrigin().getX() - plan[i].pose.position.x) * (robot_pose.getOrigin().getX() - plan[i].pose.position.x) + (robot_pose.getOrigin().getY() - plan[i].pose.position.y) * (robot_pose.getOrigin().getY() - plan[i].pose.position.y);

            if (sq_plan_dist < min_plan_sq_dist) {
                current_waypoint_index = i;
                min_plan_sq_dist = sq_plan_dist;
            }
        }

        int offset = 1;
        if (current_waypoint_index + 1 == plan.size())
            offset = -1;

        geometry_msgs::PoseStamped p = plan[current_waypoint_index];
        double yaw = atan2(plan[current_waypoint_index+offset].pose.position.y - plan[current_waypoint_index].pose.position.y,
                           plan[current_waypoint_index+offset].pose.position.x - plan[current_waypoint_index].pose.position.x);
        p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        return p;
    }

    LocalPlannerState DWAPlanner::determineState(double angle_error, double path_distance, double goal_distance)
    {
        static LocalPlannerState prev_state = Default;
        LocalPlannerState state = Default;

        // todo: optional path_distance state

        double max_yaw_error = 1.6; // todo: get rid of this one

        // ToDo: can we make this more generic???
        if (goal_distance < switch_dist_)
            state = Arrive;
        else if (fabs(angle_error) > max_yaw_error || ( prev_state == Align && fabs(angle_error) > (max_yaw_error/2) ) )
            state = Align;
        else
            state = Default;

        // To print state
        if (prev_state != state)
        {
            ROS_INFO("State = %i", state);
            prev_state = state;
        }

        return state;
    }

    void DWAPlanner::updatePlanAndLocalCosts(tf::Stamped<tf::Pose> robot_pose, const std::vector<geometry_msgs::PoseStamped>& new_plan, const std::vector<geometry_msgs::Point>& footprint_spec)
    {
        /// Set the new plan
        plan_ = new_plan;

        /// Determine where we are w.r.t. the plan
        geometry_msgs::PoseStamped current_plan_pose = getCurrentPlanPose(robot_pose, plan_);

        /// Determine the errors
        double angle_error = base_local_planner::getGoalOrientationAngleDifference(robot_pose, tf::getYaw(current_plan_pose.pose.orientation));
        double path_distance = base_local_planner::getGoalPositionDistance(robot_pose, current_plan_pose.pose.position.x, current_plan_pose.pose.position.y);
        double goal_distance = base_local_planner::getGoalPositionDistance(robot_pose, plan_.back().pose.position.x, plan_.back().pose.position.y);

        /// Determine state of the controller
        LocalPlannerState state = determineState(angle_error, path_distance, goal_distance);

        /// Update the cost functions depending on the state we are in
        path_costs_.setTargetPoses(plan_);
        goal_costs_.setTargetPoses(plan_);

        double theta_desired = 0.0;

        switch (state)
        {
        case Default:
            path_costs_.setScale(pdist_scale_);
            goal_costs_.setScale(gdist_scale_);
            cmd_vel_costs_.setScale(1.0);

            /// Set desired orientation to global plan
            theta_desired = tf::getYaw(current_plan_pose.pose.orientation);

            break;

        case Arrive:
            /// Don't penalize sideways or backwards motion, don't care about path costs
            path_costs_.setScale(0.0);
            goal_costs_.setScale(0.1*gdist_scale_);
            cmd_vel_costs_.setScale(0.0);

            /// Set desired orientation to final goal
            theta_desired = tf::getYaw(plan_.back().pose.orientation);

            break;

        case Align:
            path_costs_.setScale(0.0);
            goal_costs_.setScale(0.0);
            cmd_vel_costs_.setScale(0.0);

            /// Set desired orientation to current index of global plan
            theta_desired = tf::getYaw(current_plan_pose.pose.orientation);
            break;
        }

        alignment_costs_.setDesiredOrientation(theta_desired);
        obstacle_costs_.setFootprint(footprint_spec);
    }

    /*
    * given the current state of the robot, find a good trajectory
    */
    base_local_planner::Trajectory DWAPlanner::findBestPath(tf::Stamped<tf::Pose> robot_pose, tf::Stamped<tf::Pose> robot_vel, tf::Stamped<tf::Pose>& drive_velocities)
    {
        //make sure that our configuration doesn't change mid-run
        boost::mutex::scoped_lock l(configuration_mutex_);

        // Setup the variables for traj generation
        Eigen::Vector3f pos(robot_pose.getOrigin().getX(), robot_pose.getOrigin().getY(), tf::getYaw(robot_pose.getRotation()));
        Eigen::Vector3f vel(robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), tf::getYaw(robot_vel.getRotation()));
        geometry_msgs::PoseStamped goal_pose = plan_.back();
        Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
        base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

        // prepare cost functions and generators for this run
        generator_.initialise(pos, vel, goal, &limits, vsamples_);

        // find best trajectory by sampling and scoring the samples
        result_traj_.cost_ = -7;
        std::vector<base_local_planner::Trajectory> all_explored;
        scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);

        // Publish the trajectory grid
        publishTrajectoryCloud(all_explored);

        // Set the driving velocity, command zero if no valid trajectory is found
        if (result_traj_.cost_ < 0) {
            drive_velocities.setIdentity();
        } else {
            tf::Vector3 start(result_traj_.xv_, result_traj_.yv_, 0);
            drive_velocities.setOrigin(start);
            tf::Matrix3x3 matrix;
            matrix.setRotation(tf::createQuaternionFromYaw(result_traj_.thetav_));
            drive_velocities.setBasis(matrix);
        }

        return result_traj_;
    }
}
