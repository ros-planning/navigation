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
*   * Neither the name of the Willow Garage nor the names of its
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
#ifndef TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_ROS_H_
#define TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_ROS_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/point_grid.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/voxel_grid_model.h>
#include <base_local_planner/trajectory_planner.h>

#include <base_local_planner/planar_laser_scan.h>

#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <tf/message_notifier.h>
#include <tf/transform_listener.h>

#include <boost/thread.hpp>

#include <string>

#include <angles/angles.h>

#include <nav_core/base_local_planner.h>

namespace base_local_planner {
  /**
   * @class TrajectoryPlannerROS
   * @brief A ROS wrapper for the trajectory controller that queries the param server to construct a controller
   */
  class TrajectoryPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Default constructor for the ros wrapper
       */
      TrajectoryPlannerROS();

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      TrajectoryPlannerROS(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~TrajectoryPlannerROS();
      
      /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

    private:
      /**
       * @brief  Used for display purposes, allows the footprint of the robot to be drawn in visualization tools
       * @param x_i The x position of the robot
       * @param y_i The y position of the robot
       * @param theta_i The orientation of the robot
       * @return A vector of points in world coordinates that correspond to the verticies of the robot's footprint 
       */
      std::vector<geometry_msgs::Point> drawFootprint(double x_i, double y_i, double theta_i);

      /**
       * @brief  Check whether the robot is stopped or not
       * @return True if the robot is stopped, false otherwise
       */
      bool stopped();

      /**
       * @brief  Check if the goal orientation has been achieved
       * @param  global_pose The pose of the robot in the global frame
       * @param  goal_x The desired x value for the goal
       * @param  goal_y The desired y value for the goal
       * @return True if achieved, false otherwise
       */
      bool goalOrientationReached(const tf::Stamped<tf::Pose>& global_pose, double goal_th);

      /**
       * @brief  Check if the goal position has been achieved
       * @param  global_pose The pose of the robot in the global frame
       * @param  goal_th The desired th value for the goal
       * @return True if achieved, false otherwise
       */
      bool goalPositionReached(const tf::Stamped<tf::Pose>& global_pose, double goal_x, double goal_y);

      /**
       * @brief Once a goal position is reached... rotate to the goal orientation
       * @param  global_pose The pose of the robot in the global frame
       * @param  robot_vel The velocity of the robot
       * @param  goal_th The desired th value for the goal
       * @param  cmd_vel The velocity commands to be filled
       * @return  True if a valid trajectory was found, false otherwise
       */
      bool rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Compute the distance between two points
       * @param x1 The first x point 
       * @param y1 The first y point 
       * @param x2 The second x point 
       * @param y2 The second y point 
       */
      double distance(double x1, double y1, double x2, double y2);

      /**
       * @brief  Trim off parts of the global plan that are far enough behind the robot
       * @param global_pose The pose of the robot in the global frame
       * @param plan The plan to be pruned
       * @param global_plan The plan to be pruned inf the frame of the planner
       */
      void prunePlan(const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& global_plan);

      /**
       * @brief  Transforms the global plan of the robot from the planner frame to the local frame
       * @param transformed_plan Populated with the transformed plan
       */
      bool transformGlobalPlan(std::vector<geometry_msgs::PoseStamped>& transformed_plan);

      /**
       * @brief  Publishes the footprint of the robot for visualization purposes
       * @param global_pose The pose of the robot in the global frame
       */
      void publishFootprint(const tf::Stamped<tf::Pose>& global_pose);

      /**
       * @brief  Publish a plan for visualization purposes
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub, double r, double g, double b, double a);

      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

      WorldModel* world_model_; ///< @brief The world model that the controller will use
      TrajectoryPlanner* tc_; ///< @brief The trajectory controller
      costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use
      costmap_2d::Costmap2D costmap_; ///< @brief The costmap the controller will use
      tf::TransformListener* tf_; ///< @brief Used for transforming point clouds
      std::string global_frame_; ///< @brief The frame in which the controller will run
      double max_sensor_range_; ///< @brief Keep track of the effective maximum range of our sensors
      nav_msgs::Odometry base_odom_; ///< @brief Used to get the velocity of the robot
      std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot
      double rot_stopped_velocity_, trans_stopped_velocity_;
      double xy_goal_tolerance_, yaw_goal_tolerance_, min_in_place_vel_th_;
      double inscribed_radius_, circumscribed_radius_, inflation_radius_; 
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      bool prune_plan_;
      ros::Publisher footprint_pub_, g_plan_pub_, l_plan_pub_;
      ros::Subscriber odom_sub_;
      boost::recursive_mutex odom_lock_;
      bool initialized_;
  };

};

#endif
