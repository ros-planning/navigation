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
#ifndef COSTMAP_3D_COSTMAP_3D_PUBLISHER_H_
#define COSTMAP_3D_COSTMAP_3D_PUBLISHER_H_

#include <ros/ros.h>
#include <costmap_3d/layered_costmap_3d.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapUpdate.h>

namespace costmap_3d
{

/**
 * @class Costmap3DPublisher
 * @brief A tool to periodically publish visualization data from a LayeredCostmap3D
 */
class Costmap3DPublisher
{
public:
  Costmap3DPublisher(const ros::NodeHandle& nh, LayeredCostmap3D* layered_costmap_3d, std::string topic_name);

  ~Costmap3DPublisher();

private:
  void updateCompleteCallback(LayeredCostmap3D* layered_costmap_3d,
                              const Costmap3D& delta_map,
                              const Costmap3D& bounds_map);

  void connectCallback(const ros::SingleSubscriberPublisher& pub);
  octomap_msgs::OctomapPtr createMapMessage(const Costmap3D& map);
  octomap_msgs::OctomapUpdatePtr createMapUpdateMessage(const Costmap3D& value_map, const Costmap3D& bounds_map, bool first_map=false);

  std::string update_complete_id;
  ros::NodeHandle nh_;
  LayeredCostmap3D* layered_costmap_3d_;
  ros::Publisher costmap_pub_;
  ros::Publisher costmap_update_pub_;
  uint32_t update_seq_;
};

}  // namespace costmap_3d

#endif  // COSTMAP_3D_COSTMAP_3D_PUBLISHER_H_
