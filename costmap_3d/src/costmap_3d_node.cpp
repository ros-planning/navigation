/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Badger Technologies LLC
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
 * Author: C. Andy Martin
 *********************************************************************/
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Empty.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <costmap_3d/costmap_3d_ros.h>
#include <tf2_ros/transform_listener.h>

costmap_3d::Costmap3DROS* g_costmap_ptr;

bool stopService(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
  if (g_costmap_ptr)
  {
    g_costmap_ptr->stop();
  }
  return true;
}

bool startService(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
  if (g_costmap_ptr)
  {
    g_costmap_ptr->start();
  }
  return true;
}

bool resetService(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
  if (g_costmap_ptr)
  {
    g_costmap_ptr->resetLayers();
  }
  return true;
}

bool eraseBBoxService(octomap_msgs::BoundingBoxQueryRequest& req, octomap_msgs::BoundingBoxQueryResponse& resp)
{
  if (g_costmap_ptr)
  {
    g_costmap_ptr->resetBoundingBox(req.min, req.max);
  }
  return true;
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "costmap_3d_node");
  ros::NodeHandle pnh("~");
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);
  costmap_3d::Costmap3DROS costmap("costmap", buffer);
  g_costmap_ptr = &costmap;

  auto reset_srv = pnh.advertiseService("reset", resetService);
  auto stop_srv = pnh.advertiseService("stop", stopService);
  auto start_srv = pnh.advertiseService("start", startService);
  auto erase_bbx_srv = pnh.advertiseService("erase_bbx", eraseBBoxService);

  ros::spin();

  return 0;
}
