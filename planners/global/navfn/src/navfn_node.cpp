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
* Author: Bhaskara Marthi
*********************************************************************/
#include <navfn/navfn_ros.h>
#include <navfn/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>

namespace cm=costmap_2d;
namespace rm=geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;

namespace navfn {

class NavfnWithCostmap : public NavfnROS
{
public:
  NavfnWithCostmap(string name, Costmap2DROS* cmap);
  bool makePlanService(MakeNavPlan::Request& req, MakeNavPlan::Response& resp);

private:
  void poseCallback(const rm::PoseStamped::ConstPtr& goal);
  Costmap2DROS* cmap_;
  ros::ServiceServer make_plan_service_;
  ros::Subscriber pose_sub_;
};


bool NavfnWithCostmap::makePlanService(MakeNavPlan::Request& req, MakeNavPlan::Response& resp)
{
  vector<PoseStamped> path;

  req.start.header.frame_id = "map";
  req.goal.header.frame_id = "map";
  bool success = makePlan(req.start, req.goal, path);
  resp.plan_found = success;
  if (success) {
    resp.path = path;
  }

  return true;
}

void NavfnWithCostmap::poseCallback(const rm::PoseStamped::ConstPtr& goal) {
  geometry_msgs::PoseStamped global_pose;
  cmap_->getRobotPose(global_pose);
  vector<PoseStamped> path;
  makePlan(global_pose, *goal, path);
}


NavfnWithCostmap::NavfnWithCostmap(string name, Costmap2DROS* cmap) : 
  NavfnROS(name, cmap)
{
  ros::NodeHandle private_nh("~");
  cmap_ = cmap;
  make_plan_service_ = private_nh.advertiseService("make_plan", &NavfnWithCostmap::makePlanService, this);
  pose_sub_ = private_nh.subscribe<rm::PoseStamped>("goal", 1, &NavfnWithCostmap::poseCallback, this);
}

} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "global_planner");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  costmap_2d::Costmap2DROS lcr("costmap", buffer);

  navfn::NavfnWithCostmap navfn("navfn_planner", &lcr);

  ros::spin();
  return 0;
}
















