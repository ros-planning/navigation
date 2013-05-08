/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 * Author: Bhaskara Marthi
 *         David V. Lu!!
 *********************************************************************/
#include <global_planner/planner_core.h>
#include <navfn/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>

namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;

namespace global_planner {

class PlannerWithCostmap : public PlannerCore {
    public:
        PlannerWithCostmap(string name, Costmap2DROS* cmap);
        bool makePlanService(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& resp);

    private:
        void poseCallback(const rm::PoseStamped::ConstPtr& goal);
        Costmap2DROS* cmap_;
        ros::ServiceServer make_plan_service_;
        ros::Subscriber pose_sub_;
};

bool PlannerWithCostmap::makePlanService(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& resp) {
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

void PlannerWithCostmap::poseCallback(const rm::PoseStamped::ConstPtr& goal) {
    tf::Stamped<tf::Pose> global_pose;
    cmap_->getRobotPose(global_pose);
    vector<PoseStamped> path;
    rm::PoseStamped start;
    start.header.stamp = global_pose.stamp_;
    start.header.frame_id = global_pose.frame_id_;
    start.pose.position.x = global_pose.getOrigin().x();
    start.pose.position.y = global_pose.getOrigin().y();
    start.pose.position.z = global_pose.getOrigin().z();
    start.pose.orientation.x = global_pose.getRotation().x();
    start.pose.orientation.y = global_pose.getRotation().y();
    start.pose.orientation.z = global_pose.getRotation().z();
    start.pose.orientation.w = global_pose.getRotation().w();
    makePlan(start, *goal, path);
}

PlannerWithCostmap::PlannerWithCostmap(string name, Costmap2DROS* cmap) :
        PlannerCore(name, cmap) {
    ros::NodeHandle private_nh("~");
    cmap_ = cmap;
    make_plan_service_ = private_nh.advertiseService("make_plan", &PlannerWithCostmap::makePlanService, this);
    pose_sub_ = private_nh.subscribe<rm::PoseStamped>("goal", 1, &PlannerWithCostmap::poseCallback, this);
}

} // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_planner");

    tf::TransformListener tf(ros::Duration(10));

    costmap_2d::Costmap2DROS lcr("costmap", tf);

    global_planner::PlannerWithCostmap pppp("planner", &lcr);

    ros::spin();
    return 0;
}

