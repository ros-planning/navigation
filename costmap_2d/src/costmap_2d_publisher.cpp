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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/cost_values.h>

namespace costmap_2d
{
Costmap2DPublisher::Costmap2DPublisher(ros::NodeHandle ros_node, Costmap2D* costmap, std::string topic_name) :
    node(&ros_node), costmap_(costmap), active_(false)
{

  costmap_pub_ = ros_node.advertise < nav_msgs::OccupancyGrid > (topic_name, 1, true);
  costmap_update_pub_ = ros_node.advertise < costmap_2d::OccupancyGridUpdate
      > (topic_name + std::string("_updates"), 1);
}

Costmap2DPublisher::~Costmap2DPublisher()
{
}

void Costmap2DPublisher::publishCostmap()
{
  boost::shared_lock < boost::shared_mutex > lock(*(costmap_->getLock()));
  double resolution = costmap_->getResolution();

  if (grid_.header.frame_id != costmap_->getGlobalFrameID() || grid_.info.resolution != resolution
      || grid_.info.width != costmap_->getSizeInCellsX())
  {

    // Publish Whole Grid
    grid_.header.frame_id = costmap_->getGlobalFrameID();
    grid_.header.stamp = ros::Time::now();
    grid_.info.resolution = resolution;

    grid_.info.width = costmap_->getSizeInCellsX();
    grid_.info.height = costmap_->getSizeInCellsY();

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid_.info.origin.position.x = wx - resolution / 2;
    grid_.info.origin.position.y = wy - resolution / 2;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;

    grid_.data.resize(grid_.info.width * grid_.info.height);

    unsigned char* data = costmap_->getCharMap();
    for (unsigned int i = 0; i < grid_.data.size(); i++)
    {
      grid_.data[i] = data[i];
    }
    costmap_pub_.publish(grid_);
  }
  else if (x0_ < xn_)
  {
    // Publish Just an Update
    costmap_2d::OccupancyGridUpdate update;
    update.header.stamp = ros::Time::now();
    update.header.frame_id = costmap_->getGlobalFrameID();
    update.x = x0_;
    update.y = y0_;
    update.width = xn_ - x0_;
    update.height = yn_ - y0_;
    update.data.resize(update.width * update.height);

    unsigned int i = 0;
    for (unsigned int y = y0_; y < yn_; y++)
    {
      for (unsigned int x = x0_; x < xn_; x++)
      {
        unsigned char cost = costmap_->getCost(x, y);
        update.data[i++] = cost;
      }
    }
    costmap_update_pub_.publish(update);
  }

  xn_ = yn_ = 0;
  x0_ = costmap_->getSizeInCellsX();
  y0_ = costmap_->getSizeInCellsY();
}

}
