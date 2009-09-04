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
#include <navfn/SetCostmap.h>
#include <navfn/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>

namespace cm=costmap_2d;
namespace rm=geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;

typedef boost::shared_ptr<Costmap2D> CmapPtr;

namespace navfn {

class NavfnWithLocalCostmap : public NavfnROS
{
public:
  NavfnWithLocalCostmap(string name, Costmap2DROS* cmap);
  bool setCostmap(SetCostmap::Request& req, SetCostmap::Response& resp);
  bool makePlanService(MakeNavPlan::Request& req, MakeNavPlan::Response& resp);

protected:
  virtual void getCostmap(Costmap2D& cmap);

private:
  ros::NodeHandle node_;
  CmapPtr cmap_;
  ros::ServiceServer set_costmap_service_;
  ros::ServiceServer make_plan_service_;
};


bool NavfnWithLocalCostmap::makePlanService(MakeNavPlan::Request& req, MakeNavPlan::Response& resp)
{
  vector<PoseStamped> path;

  req.start.header.frame_id = "/map";
  req.goal.header.frame_id = "/map";

  bool success = makePlan(req.start, req.goal, path);

  resp.plan_found = success;
  if (success) {
    resp.path = path;
  }

  return true;
}


NavfnWithLocalCostmap::NavfnWithLocalCostmap(string name, Costmap2DROS* cmap) : NavfnROS(name, cmap)
{
  inscribed_radius_ = 0.0;
  circumscribed_radius_ = 0.0;
  inflation_radius_ = 0.0;

  set_costmap_service_ = node_.advertiseService("~set_costmap", &NavfnWithLocalCostmap::setCostmap, this);
  make_plan_service_ = node_.advertiseService("~make_plan", &NavfnWithLocalCostmap::makePlanService, this);
}


bool NavfnWithLocalCostmap::setCostmap(SetCostmap::Request& req, SetCostmap::Response& resp)
{
  cmap_.reset(new Costmap2D(req.width, req.height, 1.0, 0.0, 0.0));
  unsigned ind=0;
  for (unsigned y=0; y<req.height; y++) 
    for (unsigned x=0; x<req.width; x++) 
      cmap_->setCost(x, y, req.costs[ind++]);


  for (unsigned y=0; y<req.height; y++) 
    for (unsigned x=0; x<req.width; x++) 
      ROS_DEBUG_NAMED ("node", "Cost of %u, %u is %u", x, y, cmap_->getCost(x,y));

  planner_.reset(new NavFn(cmap_->getSizeInCellsX(), cmap_->getSizeInCellsY()));
  ROS_DEBUG_STREAM_NAMED ("node", "Resetting planner object to have size " << cmap_->getSizeInCellsX() << ", " << cmap_->getSizeInCellsY());

  return true;
}

void NavfnWithLocalCostmap::getCostmap(Costmap2D& cmap)
{
  cmap = *cmap_;
}


} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "navfn_node");
  

  ros::NodeHandle n;
  tf::TransformListener tf;

  // Set params
  n.setParam("~dummy_costmap/global_frame", "/map");
  n.setParam("~dummy_costmap/robot_base_frame", "/map"); // Do this so it doesn't complain about unavailable transform 
  n.setParam("~dummy_costmap/publish_frequency", 0.0);
  n.setParam("~dummy_costmap/observation_sources", string(""));
  n.setParam("~dummy_costmap/static_map", false);

  
  Costmap2DROS dummy_costmap("dummy_costmap", tf);
  navfn::NavfnWithLocalCostmap navfn("navfn_planner", &dummy_costmap);

  ros::spin();
  return 0;
}
















