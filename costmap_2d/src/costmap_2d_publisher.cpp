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
  Costmap2DPublisher::Costmap2DPublisher(ros::NodeHandle ros_node, Costmap2D* costmap, double publish_frequency, std::string global_frame, std::string topic_name) 
    : node(&ros_node), costmap_(costmap), global_frame_(global_frame), visualizer_thread_(NULL), active_(false), visualizer_thread_shutdown_(false){

    costmap_pub_ = ros_node.advertise<nav_msgs::GridCells>(topic_name, 1);
    visualizer_thread_ = new boost::thread(boost::bind(&Costmap2DPublisher::mapPublishLoop, this, publish_frequency));
  }

  Costmap2DPublisher::~Costmap2DPublisher(){
    visualizer_thread_shutdown_ = true;
    if(visualizer_thread_ != NULL){
      visualizer_thread_->join();
      delete visualizer_thread_;
    }
  }

  void Costmap2DPublisher::mapPublishLoop(double frequency){
    //the user might not want to run the loop every cycle
    if(frequency == 0.0)
      return;

    active_ = true;
    ros::NodeHandle n;
    ros::Rate r(frequency);
    while(n.ok() && !visualizer_thread_shutdown_){
      //we are about to publish the latest data
      ROS_DEBUG("Publishing costmap");
      publishCostmap();

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / frequency))
        ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency, r.cycleTime().toSec());
    }
  }

  void Costmap2DPublisher::publishCostmap(){
    double resolution = costmap_->getResolution();
    
    //create a GridCells message 
    nav_msgs::GridCells cells;
    cells.header.frame_id = global_frame_;
    cells.header.stamp = ros::Time::now();

    //set the width and height appropriately
    cells.cell_width = resolution;
    cells.cell_height = resolution;
    
    lock_.lock();
    for(unsigned int i = 0; i < costmap_->getSizeInCellsX(); i++){
      for(unsigned int j = 0; j < costmap_->getSizeInCellsY(); j++){

        unsigned char cost = costmap_->getCost(i, j);
        if(cost==NO_INFORMATION)
            continue;
            
        geometry_msgs::Point p;
        costmap_->mapToWorld(i, j, p.x, p.y);
        p.z = cost / 256.0;
        cells.cells.push_back( p );
      }
    }
    costmap_pub_.publish(cells);
    lock_.unlock();
  }

};
