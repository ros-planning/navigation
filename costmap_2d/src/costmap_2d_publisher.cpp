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
#include <geometry_msgs/Point.h>
#include <costmap_2d/cost_values.h>

namespace costmap_2d {
  Costmap2DPublisher::Costmap2DPublisher(ros::NodeHandle ros_node, Costmap2D* costmap, std::string global_frame, std::string topic_name) 
    : node(&ros_node), costmap_(costmap), global_frame_(global_frame), active_(false) {

    costmap_pub_ = ros_node.advertise<nav_msgs::GridCells>(topic_name, 1);
  }

  Costmap2DPublisher::~Costmap2DPublisher(){  }

  void Costmap2DPublisher::publishCostmap(){
    boost::shared_lock< boost::shared_mutex > lock(*(costmap_->getLock()));
    double resolution = costmap_->getResolution();
    
    //create a GridCells message 
    nav_msgs::GridCells cells;
    cells.header.frame_id = global_frame_;
    cells.header.stamp = ros::Time::now();

    //set the width and height appropriately
    cells.cell_width = resolution;
    cells.cell_height = resolution;
    
    for(unsigned int i = 0; i < costmap_->getSizeInCellsX(); i++){
      for(unsigned int j = 0; j < costmap_->getSizeInCellsY(); j++){

        unsigned char cost = costmap_->getCost(i, j);
        if(cost==NO_INFORMATION ||cost== 0)
            continue;
            
        geometry_msgs::Point p;
        costmap_->mapToWorld(i, j, p.x, p.y);
        p.z = cost / 256.0;
        cells.cells.push_back( p );
      }
    }
    costmap_pub_.publish(cells);
  }

};
