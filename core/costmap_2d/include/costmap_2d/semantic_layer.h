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
#ifndef COSTMAP_2D_SEMANTIC_LAYER_H_
#define COSTMAP_2D_SEMANTIC_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>

#include <pedsim_msgs/SemanticDatum.h>
#include <pedsim_msgs/SemanticData.h>

#include <nav_msgs/OccupancyGrid.h>

#include <dynamic_reconfigure/server.h>
#include <costmap_2d/SemanticPluginConfig.h>

#include <costmap_2d/SemanticDump.h>

namespace costmap_2d
{

typedef std::vector<pedsim_msgs::SemanticDatum> SemanticObservation;

class SemanticBuffer{
public:
  SemanticBuffer(ros::NodeHandle _nh, std::string _topic){
    nh = _nh;
    stamp = ros::Time(0);
    topic = _topic;
  };

  void subscribe(){
    sub = nh.subscribe(
      topic,
      1,
      &SemanticBuffer::callback,
      this
    );
  };

  void unsubscribe(){
    if(sub)
      sub.shutdown();
  };

  ~SemanticBuffer(){
    unsubscribe();
  };

  std::string getTopic() { return topic; }
  ros::Time getStamp(){ return stamp; }
  SemanticObservation getData(){ return data; }
  std::string getType(){ return type; }

private:

  void callback(const pedsim_msgs::SemanticDataConstPtr& semanticData){
    stamp = semanticData->header.stamp;
    data = semanticData->points;
    type = semanticData->type;
  }

  ros::NodeHandle nh;
  std::string topic;
  
  ros::Time stamp;
  SemanticObservation data;
  std::string type;

  ros::Subscriber sub;
};

class SemanticLayer : public CostmapLayer
{
public:
  SemanticLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }

  virtual ~SemanticLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  boost::any dump(LayerType& type) override;

protected:
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

  /**
   * @brief  Get the observations used to mark space
   * @param observations A reference to a vector that will be populated with the observations
   * @return True if all the observation buffers are current, false otherwise
   */
  bool getObservations(std::vector<boost::shared_ptr<SemanticBuffer>>& buffers, std::vector<SemanticObservation>& observations) const;

  std::string global_frame_;  ///< @brief The global frame for the costmap

  std::vector<boost::shared_ptr<SemanticBuffer>> observation_buffers_;  ///< @brief Used to store observations from various sensors
  std::vector<boost::shared_ptr<SemanticBuffer>> marking_buffers_;  ///< @brief Used to store observation buffers used for marking obstacles
  std::vector<boost::shared_ptr<SemanticBuffer>> clearing_buffers_;  ///< @brief Used to store observation buffers used for clearing obstacles

  bool rolling_window_;
  dynamic_reconfigure::Server<costmap_2d::SemanticPluginConfig> *dsrv_;

  int combination_method_;

private:
  void reconfigureCB(costmap_2d::SemanticPluginConfig &config, uint32_t level);
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_SEMANTIC_LAYER_H_
