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
#ifndef NAVFN_NAVFN_ROS_H_
#define NAVFN_NAVFN_ROS_H_

#include <ros/ros.h>
#include <navfn/navfn.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_core/base_global_planner.h>

namespace navfn {
  class NavfnROS : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief  Default constructor for the NavFnROS object
       */
      NavfnROS();

      /**
       * @brief  Constructor for the NavFnROS object
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
       */
      NavfnROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Initialization function for the NavFnROS object
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Compute the full navigation function for the costmap given a point in the world to start from
       * @param world_point The point to use for seeding the navigation function 
       * @return True if the navigation function was computed successfully, false otherwise
       */
      bool computePotential(const geometry_msgs::Point& world_point);

      /**
       * @brief Get the potential, or naviagation cost, at a given point in the world (Note: You should call computePotential first)
       * @param world_point The point to get the potential for 
       * @return The navigation function's value at that point in the world
       */
      double getPointPotential(const geometry_msgs::Point& world_point);

      /**
       * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
       * @param world_point The point to get the potential for 
       * @return True if the navigation function is valid at that point in the world, false otherwise
       */
      bool validPointPotential(const geometry_msgs::Point& world_point);

      /**
       * @brief  Publish a path for visualization purposes
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);

      ~NavfnROS(){}

    protected:

      /**
       * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
       */
      virtual void getCostmap(costmap_2d::Costmap2D& costmap); 
      costmap_2d::Costmap2DROS* costmap_ros_;
      boost::shared_ptr<NavFn> planner_;
      double inscribed_radius_, circumscribed_radius_, inflation_radius_;
      ros::Publisher plan_pub_;
      bool initialized_;


    private:
      void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);
      costmap_2d::Costmap2D costmap_;
      std::string global_frame_;
  };
};

#endif
