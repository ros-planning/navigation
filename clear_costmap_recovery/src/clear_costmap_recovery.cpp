/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*********************************************************************/
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <vector>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(clear_costmap_recovery, ClearCostmapRecovery, clear_costmap_recovery::ClearCostmapRecovery, nav_core::RecoveryBehavior)

using costmap_2d::NO_INFORMATION;

namespace clear_costmap_recovery {
ClearCostmapRecovery::ClearCostmapRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
  tf_(NULL), initialized_(false) {} 

void ClearCostmapRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);

    private_nh.param("reset_distance", reset_distance_, 3.0);
    private_nh.param("layer_search_string", layer_search_string_, std::string("obstacle"));

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void ClearCostmapRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Clearing costmap to unstuck robot.");
  clear(global_costmap_);
  clear(local_costmap_);
}

void ClearCostmapRecovery::clear(costmap_2d::Costmap2DROS* costmap){
  std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();

  tf::Stamped<tf::Pose> pose;

  if(!costmap->getRobotPose(pose)){
    ROS_ERROR("Cannot clear map because pose cannot be retrieved");
    return;
  }

  double x = pose.getOrigin().x();
  double y = pose.getOrigin().y();

      for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
            boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
          if(plugin->getName().find(layer_search_string_)!=std::string::npos){
            boost::shared_ptr<costmap_2d::ObstacleLayer> costmap;
            costmap = boost::static_pointer_cast<costmap_2d::ObstacleLayer>(plugin);
            clearMap(costmap, x, y);
          }
      }
}


  void ClearCostmapRecovery::clearMap(boost::shared_ptr<costmap_2d::ObstacleLayer> costmap, double pose_x, double pose_y){
         boost::unique_lock< boost::shared_mutex > lock(*(costmap->getLock()));
         
         double start_point_x = pose_x - reset_distance_ / 2;
         double start_point_y = pose_y - reset_distance_ / 2;
         double end_point_x = start_point_x + reset_distance_;
         double end_point_y = start_point_y + reset_distance_;

         int start_x, start_y, end_x, end_y;
         costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
         costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

         unsigned char* grid = costmap->getCharMap();
         for(int x=0; x<(int)costmap->getSizeInCellsX(); x++){
	   bool xrange = x>start_x && x<end_x;
	                   
            for(int y=0; y<(int)costmap->getSizeInCellsY(); y++){
	      if(xrange && y>start_y && y<end_y)
	          continue;
                int index = costmap->getIndex(x,y);
                if(grid[index]!=NO_INFORMATION){
		  grid[index] = NO_INFORMATION;
		}
	    }
	 }
	 costmap->setResetBounds(0,0, costmap->getSizeInMetersX(), costmap->getSizeInMetersY());
	 return;
    }

};
