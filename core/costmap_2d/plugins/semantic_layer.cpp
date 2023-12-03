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
#include <costmap_2d/semantic_layer.h>
#include <costmap_2d/costmap_math.h>

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_2d::SemanticLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

void SemanticLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();

  default_value_ = FREE_SPACE;

  SemanticLayer::matchSize();
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();
  double transform_tolerance;

  std::string topics_string;
  // get the topics that we'll subscribe to from the parameter server
  nh.param("observation_sources", topics_string, std::string(""));
  ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source)
  {
    ros::NodeHandle source_node(nh, source);

    // get the parameters for the specific topic
    std::string topic;
    bool clearing, marking;

    source_node.param("topic", topic, source);
    source_node.param("clearing", clearing, false);
    source_node.param("marking", marking, false);

    ROS_DEBUG("Creating a semantic buffer for source %s, topic %s", source.c_str(), topic.c_str());

    // create an observation buffer
    observation_buffers_.push_back(
        boost::shared_ptr<SemanticBuffer>(new SemanticBuffer(source_node, topic))
    );

    // check if we'll add this buffer to our marking observation buffers
    if (marking)
      marking_buffers_.push_back(observation_buffers_.back());

    // check if we'll also add this buffer to our clearing observation buffers
    if (clearing)
      clearing_buffers_.push_back(observation_buffers_.back());

  }

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);
  activate();
}

void SemanticLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::SemanticPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::SemanticPluginConfig>::CallbackType cb =
      [this](auto& config, auto level){ reconfigureCB(config, level); };
  dsrv_->setCallback(cb);
}

SemanticLayer::~SemanticLayer()
{
    if (dsrv_)
        delete dsrv_;
}
void SemanticLayer::reconfigureCB(costmap_2d::SemanticPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  combination_method_ = config.combination_method;
}

void SemanticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<SemanticObservation> observations, clearing_observations;

  // get the marking observations
  current = current && getObservations(marking_buffers_, observations);

  // get the clearing observations
  current = current && getObservations(clearing_buffers_, clearing_observations);

  // update the global current status
  current_ = current;

//   for (unsigned int i = 0; i < clearing_observations.size(); ++i)
//   {
//     raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
//   }

  for (auto& observation : clearing_observations)
  {
    for (auto &datum: observation){

        unsigned int mx, my;
        if (!worldToMap(datum.location.x, datum.location.y, mx, my))
        {
            ROS_DEBUG("Computing map coords failed");
            continue;
        }

        unsigned int index = getIndex(mx, my);
        costmap_[index] = FREE_SPACE;
        touch(datum.location.x, datum.location.y, min_x, min_y, max_x, max_y);
    }
  }

  for (auto& observation : observations)
  {
    for (auto &datum: observation){

        unsigned int mx, my;
        if (!worldToMap(datum.location.x, datum.location.y, mx, my))
        {
            ROS_DEBUG("Computing map coords failed");
            continue;
        }

        unsigned int index = getIndex(mx, my);
        costmap_[index] = LETHAL_OBSTACLE;
        touch(datum.location.x, datum.location.y, min_x, min_y, max_x, max_y);
    }
  }

}


void SemanticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{

  switch (combination_method_)
  {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

bool SemanticLayer::getObservations(std::vector<boost::shared_ptr<SemanticBuffer>>& buffers, std::vector<SemanticObservation>& observations) const{
    bool current = true;
  // get the marking observations
  for (const auto& buffer: buffers)
  {
    observations.push_back(buffer->getData());
    current &= buffer->getStamp().isValid(); //TODO
  }

  return current;
}



void SemanticLayer::activate()
{
  // if we're stopped we need to re-subscribe to topics
  for (const auto& buffer : observation_buffers_)
    buffer->subscribe();
}
void SemanticLayer::deactivate()
{
  for (const auto& buffer : observation_buffers_)
    buffer->unsubscribe();
}

void SemanticLayer::reset()
{
  deactivate();
  resetMaps();
  current_ = true;
  activate();
}

boost::any SemanticLayer::dump(LayerType& type) {
  type = LayerType::SEMANTIC;
  
  auto dump = SemanticDump();

  for(const auto& observation : observation_buffers_){
    
    auto container = pedsim_msgs::SemanticData();
    
    container.points = observation->getData();
    container.type = observation->getType();
    container.header.stamp = observation->getStamp();

    dump.layers.push_back(container);
  }

  return dump;
}

}  // namespace costmap_2d
