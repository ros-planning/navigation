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
#include <costmap_3d/costmap_3d_publisher.h>
#include <octomap_msgs/conversions.h>

namespace costmap_3d
{

Costmap3DPublisher::Costmap3DPublisher(const ros::NodeHandle& nh,
                                       LayeredCostmap3D* layered_costmap_3d,
                                       std::string topic_name)
    : nh_(nh), layered_costmap_3d_(layered_costmap_3d)
{
  costmap_pub_ = nh_.advertise<octomap_msgs::Octomap>(topic_name, 1);
  costmap_update_pub_ = nh_.advertise<octomap_msgs::OctomapUpdate>(topic_name + "_updates", 3,
                                                      std::bind(&Costmap3DPublisher::connectCallback, this,
                                                                std::placeholders::_1));

  update_complete_id = topic_name + "_publisher";
  layered_costmap_3d_->registerUpdateCompleteCallback(update_complete_id,
                                                      std::bind(&Costmap3DPublisher::updateCompleteCallback, this,
                                                                std::placeholders::_1,
                                                                std::placeholders::_2,
                                                                std::placeholders::_3));
}

Costmap3DPublisher::~Costmap3DPublisher()
{
  layered_costmap_3d_->unregisterUpdateCompleteCallback(update_complete_id);
}

void Costmap3DPublisher::connectCallback(const ros::SingleSubscriberPublisher& pub)
{
  Costmap3D universe(layered_costmap_3d_->getResolution());
  universe.setNodeValueAtDepth(Costmap3DIndex(), 0, LETHAL);
  octomap_msgs::OctomapUpdatePtr msg_ptr;

  // Be sure to lock the costmap while using it, and keep it locked until
  // we have published the state, otherwise a race can occur when the
  // map update complete callback publishes and the two messages could
  // end up out-of-order
  std::lock_guard<LayeredCostmap3D> costmap_lock(*layered_costmap_3d_);
  // Send an "update" message with universal bounds and the entire costmap.
  // This way, a new subscriber starts with the correct map state.
  msg_ptr = createMapUpdateMessage(*layered_costmap_3d_->getCostmap3D(), universe, true);

  pub.publish(msg_ptr);
}

void Costmap3DPublisher::updateCompleteCallback(LayeredCostmap3D* layered_costmap_3d,
                                                const Costmap3D& delta_map,
                                                const Costmap3D& bounds_map)
{
  // The layered costmap already holds the lock when calling the update
  // complete, no need to lock
  if (costmap_pub_.getNumSubscribers() > 0)
  {
    octomap_msgs::OctomapPtr msg_ptr(createMapMessage(*layered_costmap_3d_->getCostmap3D()));
    costmap_pub_.publish(msg_ptr);
  }
  if (costmap_update_pub_.getNumSubscribers() > 0)
  {
    octomap_msgs::OctomapUpdatePtr msg_ptr(createMapUpdateMessage(delta_map, bounds_map));
    costmap_update_pub_.publish(msg_ptr);
  }
}

octomap_msgs::OctomapPtr Costmap3DPublisher::createMapMessage(const Costmap3D& map)
{
  ros::Time stamp = ros::Time::now();
  std::string frame = layered_costmap_3d_->getGlobalFrameID();
  octomap_msgs::OctomapPtr msg_ptr(new octomap_msgs::Octomap);
  msg_ptr->header.frame_id = frame;
  msg_ptr->header.stamp = stamp;
  octomap_msgs::fullMapToMsg(map, *msg_ptr);
  return msg_ptr;
}

octomap_msgs::OctomapUpdatePtr Costmap3DPublisher::createMapUpdateMessage(const Costmap3D& value_map,
                                                                          const Costmap3D& bounds_map,
                                                                          bool first_map)
{
  ros::Time stamp = ros::Time::now();
  std::string frame = layered_costmap_3d_->getGlobalFrameID();
  octomap_msgs::OctomapUpdatePtr msg_ptr(new octomap_msgs::OctomapUpdate);
  msg_ptr->header.frame_id = frame;
  msg_ptr->header.stamp = stamp;
  msg_ptr->octomap_update.header.seq = update_seq_;
  msg_ptr->octomap_update.header.frame_id = frame;
  msg_ptr->octomap_update.header.stamp = stamp;
  msg_ptr->octomap_bounds.header.seq = update_seq_;
  msg_ptr->octomap_bounds.header.frame_id = frame;
  msg_ptr->octomap_bounds.header.stamp = stamp;
  if (first_map)
  {
    // On the first map, make the sequences unequal as sentinel
    msg_ptr->octomap_bounds.header.seq--;
  }
  else
  {
    // On regular map message, increment the sequence number
    update_seq_++;
  }
  // always send costs along with map
  octomap_msgs::fullMapToMsg(value_map, msg_ptr->octomap_update);
  // bounds map is only ever binary
  octomap_msgs::binaryMapToMsg(bounds_map, msg_ptr->octomap_bounds);
  return msg_ptr;
}

}  // end namespace costmap_2d
