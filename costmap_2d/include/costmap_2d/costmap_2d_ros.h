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
*         David V. Lu!!
*********************************************************************/
#ifndef COSTMAP_COSTMAP_2D_ROS_H_
#define COSTMAP_COSTMAP_2D_ROS_H_

#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/plugin_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/Costmap2DConfig.h>
#include <dynamic_reconfigure/server.h>


namespace costmap_2d {
  
class Costmap2DROS{
    public: 
        Costmap2DROS(std::string name, tf::TransformListener& tf);
        ~Costmap2DROS();
        void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y);
        void start();
        void stop();
        void pause();
        void resume();
        bool isCurrent(){ return layered_costmap_->isCurrent(); }
        bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;
        Costmap2D* getCostmap() { return layered_costmap_->getCostmap(); }
        std::string getGlobalFrameID() { return global_frame_; }
        std::string getBaseFrameID() { return robot_base_frame_; }
        LayeredCostmap* getLayeredCostmap() { return layered_costmap_; } 

    protected:
        LayeredCostmap* layered_costmap_;
        std::string name_;
        tf::TransformListener& tf_;  /// < @brief Used for transforming point clouds
        std::string global_frame_;  /// < @brief The global frame for the costmap
        std::string robot_base_frame_;  /// < @brief The frame_id of the robot base
        double transform_tolerance_;

    private:
        void reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level);
        void movementCB(const ros::TimerEvent &event);
        void mapUpdateLoop(double frequency);
        bool map_update_thread_shutdown_;
        bool stop_updates_, initialized_, stopped_, robot_stopped_;
        boost::thread* map_update_thread_;  /// < @brief A thread for updating the map
        ros::Timer timer_;
        ros::Time last_publish_;
        ros::Duration publish_cycle;
        pluginlib::ClassLoader<CostmapPluginROS> plugin_loader_;
        tf::Stamped<tf::Pose> old_pose_;
        Costmap2DPublisher* publisher_;
        dynamic_reconfigure::Server<costmap_2d::Costmap2DConfig> *dsrv_;
        
        boost::recursive_mutex configuration_mutex_;
};  // class Costmap2DROS
};  // namespace costmap_2d

#endif
