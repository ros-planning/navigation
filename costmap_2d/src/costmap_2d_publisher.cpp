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
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/cost_values.h>

namespace costmap_2d {
  Costmap2DPublisher::Costmap2DPublisher(ros::NodeHandle ros_node, Costmap2D* costmap, std::string topic_name) 
    : node(&ros_node), costmap_(costmap), active_(false) {

    costmap_pub_ = ros_node.advertise<nav_msgs::OccupancyGrid>(topic_name, 1, true);
    costmap_update_pub_ = ros_node.advertise<map_msgs::OccupancyGridUpdate>(topic_name + std::string("_updates"), 1);
  }

  Costmap2DPublisher::~Costmap2DPublisher(){  }

  void Costmap2DPublisher::publishCostmap(unsigned int x0, unsigned int xn, unsigned int y0, unsigned int yn){
    boost::shared_lock< boost::shared_mutex > lock(*(costmap_->getLock()));
    double resolution = costmap_->getResolution();
    
    if(grid_.header.frame_id != costmap_->getGlobalFrameID() ||
        grid_.info.resolution != resolution ||
        grid_.info.width != costmap_->getSizeInCellsX()){
        
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
        for(unsigned int i = 0; i < grid_.data.size(); i++){
            grid_.data[i] = data[i];
        }
        costmap_pub_.publish(grid_);
    }else{
        // Publish Just an Update
        map_msgs::OccupancyGridUpdate update;
        update.header.stamp = ros::Time::now();
        update.header.frame_id = costmap_->getGlobalFrameID();
        update.x = x0;
        update.y = y0;
        update.width = xn - x0 + 1;
        update.height = yn - y0 + 1;
        update.data.resize(update.width * update.height);
        
        unsigned int i = 0;
        for(unsigned int y=y0; y<=yn; y++){
            for(unsigned int x=x0; x<=xn;x++){
                unsigned char cost = costmap_->getCost(x, y);
                update.data[i++] = cost;
            }
        }
        costmap_update_pub_.publish(update);
    }
  }

};
