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

namespace costmap_2d {
  Costmap2DPublisher::Costmap2DPublisher(ros::NodeHandle& ros_node, double publish_frequency, std::string global_frame) 
    : ros_node_(ros_node), global_frame_(global_frame), 
    visualizer_thread_(NULL), active_(false), new_data_(false), resolution_(0.0){

    obs_pub_ = ros_node_.advertise<nav_msgs::GridCells>("obstacles", 1);
    inf_obs_pub_ = ros_node_.advertise<nav_msgs::GridCells>("inflated_obstacles", 1);

    visualizer_thread_ = new boost::thread(boost::bind(&Costmap2DPublisher::mapPublishLoop, this, publish_frequency));
  }

  Costmap2DPublisher::~Costmap2DPublisher(){
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
    ros::Rate r(frequency);
    while(ros_node_.ok()){
      //check if we have new data to publish
      if(new_data_){
        //we are about to publish the latest data
        new_data_ = false;
        publishCostmap();
      }

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / frequency))
        ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency, r.cycleTime().toSec());
    }
  }

  void Costmap2DPublisher::updateCostmapData(const Costmap2D& costmap){
    std::vector< std::pair<double, double> > raw_obstacles, inflated_obstacles;
    for(unsigned int i = 0; i < costmap.getSizeInCellsX(); i++){
      for(unsigned int j = 0; j < costmap.getSizeInCellsY(); j++){
        double wx, wy;
        costmap.mapToWorld(i, j, wx, wy);
        std::pair<double, double> p(wx, wy);

        //if(costmap.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
        if(costmap.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE || costmap.getCost(i, j) == costmap_2d::NO_INFORMATION)
          raw_obstacles.push_back(p);
        else if(costmap.getCost(i, j) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
          inflated_obstacles.push_back(p);
      }
    }
    lock_.lock();
    resolution_ = costmap.getResolution();
    raw_obstacles_ = raw_obstacles;
    inflated_obstacles_ = inflated_obstacles;
    lock_.unlock();
    
    //let the publisher know we have new data to publish
    new_data_ = true;
  }

  void Costmap2DPublisher::publishCostmap(){
    std::vector< std::pair<double, double> > raw_obstacles, inflated_obstacles;
    double resolution;

    lock_.lock();
    raw_obstacles = raw_obstacles_;
    inflated_obstacles = inflated_obstacles_;
    resolution = resolution_;
    lock_.unlock();

    unsigned int point_count = raw_obstacles.size();

    //create a GridCells message for the obstacles
    nav_msgs::GridCells obstacle_cells;
    obstacle_cells.header.frame_id = global_frame_;
    obstacle_cells.header.stamp = ros::Time::now();

    //set the width and height appropriately
    obstacle_cells.cell_width = resolution;
    obstacle_cells.cell_height = resolution;

    //set the size equal to the number of obstacles
    obstacle_cells.set_cells_size(point_count);

    for(unsigned int i=0;i<point_count;i++){
      obstacle_cells.cells[i].x = raw_obstacles[i].first;
      obstacle_cells.cells[i].y = raw_obstacles[i].second;
      obstacle_cells.cells[i].z = 0;
    }

    obs_pub_.publish(obstacle_cells);

    //set the size equal to the number of inflated obstacles
    point_count = inflated_obstacles.size();
    obstacle_cells.set_cells_size(point_count);

    for(unsigned int i=0;i<point_count;i++){
      obstacle_cells.cells[i].x = inflated_obstacles[i].first;
      obstacle_cells.cells[i].y = inflated_obstacles[i].second;
      obstacle_cells.cells[i].z = 0;
    }

    inf_obs_pub_.publish(obstacle_cells);
  }

};
