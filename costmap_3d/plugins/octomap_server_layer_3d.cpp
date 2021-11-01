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
#include <costmap_3d/octomap_server_layer_3d.h>
#include <std_srvs/Empty.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_3d::OctomapServerLayer3D, costmap_3d::Layer3D)

namespace costmap_3d
{

OctomapServerLayer3D::OctomapServerLayer3D() : super(), using_updates_(false)
{
  current_ = false;
}

OctomapServerLayer3D::~OctomapServerLayer3D()
{
}

void OctomapServerLayer3D::initialize(LayeredCostmap3D* parent, std::string name, tf2_ros::Buffer *tf)
{
  super::initialize(parent, name, tf);
  // Note that the master costmap's resolution is not set yet.
  // Our setResolution method will be called later once it is set.

  // Get names of topics and servers from parameter server
  pnh_ = ros::NodeHandle("~/" + name);

  std::string octomap_server_ns = pnh_.param("octomap_server_namespace", std::string(""));
  std::string octomap_server_name = pnh_.param("octomap_server_node_name", std::string("octomap_server"));
  std::string topic_prefix = octomap_server_ns;
  if (octomap_server_ns.size() > 0 && octomap_server_ns.rbegin()[0] != '/')
  {
    topic_prefix = octomap_server_ns + "/";
  }
  std::string srv_prefix = topic_prefix + octomap_server_name + "/";

  reset_srv_name_ = srv_prefix + "reset";
  erase_bbx_srv_name_ = srv_prefix + "erase_bbx";
  map_topic_ = pnh_.param("map_topic", topic_prefix + "octomap_binary");
  map_update_topic_ = pnh_.param("map_update_topic", map_topic_ + "_updates");
  double data_valid_duration = pnh_.param("data_valid_duration", 0.0);
  data_valid_duration_ = ros::Duration(data_valid_duration);

  ROS_INFO_STREAM("OctomapServerLayer3D " << name << ": initializing");
  ROS_INFO_STREAM("  map_topic: " << map_topic_);
  ROS_INFO_STREAM("  map_update_topic: " << map_update_topic_);
  ROS_INFO_STREAM("  reset_srv_name: " << reset_srv_name_);
  ROS_INFO_STREAM("  erase_bbx_srv_name: " << erase_bbx_srv_name_);
  if (data_valid_duration > 0.0)
    ROS_INFO_STREAM("  data_valid_duration: " << data_valid_duration);

  dsrv_.reset(new dynamic_reconfigure::Server<costmap_3d::GenericPluginConfig>(pnh_));
  dsrv_->setCallback(std::bind(&OctomapServerLayer3D::reconfigureCallback, this,
                               std::placeholders::_1, std::placeholders::_2));

  activate();
}

void OctomapServerLayer3D::reconfigureCallback(costmap_3d::GenericPluginConfig &config, uint32_t level)
{
  std::lock_guard<Layer3D> lock(*this);
  if (enabled_ && !config.enabled)
  {
    if (costmap_) touch(*costmap_);
    // disable last, so touch does something
    enabled_ = false;
  }
  else if (!enabled_ && config.enabled)
  {
    // enable first, so touch does something
    enabled_ = true;
    if (costmap_) touch(*costmap_);
  }
  combination_method_ = config.combination_method;
}

void OctomapServerLayer3D::updateBounds(
    const geometry_msgs::Pose robot_pose,
    const geometry_msgs::Point& rolled_min,
    const geometry_msgs::Point& rolled_max,
    Costmap3D* bounds_map)
{
  std::lock_guard<Layer3D> lock(*this);
  bool current = true;
  if (data_valid_duration_ > ros::Duration(0.0))
  {
    if (ros::Time::now() - last_update_stamp_ > data_valid_duration_)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Layer " << name_ << " octomap not updated for "
                               << (ros::Time::now() - last_update_stamp_).toSec()
                               << " expected update in at least " <<  data_valid_duration_.toSec());
      current = false;
    }
  }
  current_ = current;
  super::updateBounds(robot_pose, rolled_min, rolled_max, bounds_map);
}

void OctomapServerLayer3D::deactivate()
{
  std::lock_guard<Layer3D> lock(*this);
  active_ = false;
  unsubscribe();
  super::deactivate();
  reset_client_.shutdown();
  erase_bbx_client_.shutdown();
}

void OctomapServerLayer3D::activate()
{
  ros::ServiceClient reset_client = pnh_.serviceClient<std_srvs::Empty>(reset_srv_name_);
  ros::ServiceClient erase_bbx_client = pnh_.serviceClient<octomap_msgs::BoundingBoxQuery>(erase_bbx_srv_name_);

  // Wait a while for the servers to exist, but not too long.
  // We will handle them not existing below.
  if (!reset_client.waitForExistence(ros::Duration(10.0)))
  {
    ROS_WARN_STREAM("OctomapServerLayer3D " << name_ << ": Failed to wait for service " << reset_srv_name_);
  }
  if (!erase_bbx_client.waitForExistence(ros::Duration(10.0)))
  {
    ROS_WARN_STREAM("OctomapServerLayer3D " << name_ << ": Failed to wait for service " << erase_bbx_srv_name_);
  }

  std::lock_guard<Layer3D> lock(*this);
  reset_client_ = reset_client;
  erase_bbx_client_ = erase_bbx_client;
  super::activate();
  subscribe();
  active_ = true;
}

void OctomapServerLayer3D::subscribe()
{
  std::lock_guard<Layer3D> lock(*this);
  map_sub_ = pnh_.subscribe<octomap_msgs::Octomap>(map_topic_, 1, std::bind(&OctomapServerLayer3D::mapCallback,
                                                                            this, std::placeholders::_1));
  subscribeUpdatesUnlocked();
}

void OctomapServerLayer3D::subscribeUpdatesUnlocked()
{
  first_full_map_update_received_ = false;
  num_map_updates_ = 0;
  map_update_sub_ = pnh_.subscribe<octomap_msgs::OctomapUpdate>(map_update_topic_, 10,
                                                                std::bind(&OctomapServerLayer3D::mapUpdateCallback,
                                                                          this, std::placeholders::_1));
}

void OctomapServerLayer3D::unsubscribe()
{
  std::lock_guard<Layer3D> lock(*this);
  map_sub_.shutdown();
  using_updates_ = false;
  unsubscribeUpdatesUnlocked();
}

void OctomapServerLayer3D::unsubscribeUpdatesUnlocked()
{
  map_update_sub_.shutdown();
}

void OctomapServerLayer3D::scheduleResubscribeUpdates()
{
  // We can't resubscribe a topic from its own callback due to internal
  // locking issues. So schedule resubscribing on a one-shot timer that fires
  // immediately.
  resub_timer_ = pnh_.createTimer(
      ros::Duration(0.0),
      boost::bind(&OctomapServerLayer3D::resubscribeUpdatesCallback, this),
      true);
}

void OctomapServerLayer3D::resubscribeUpdatesCallback()
{
  std::lock_guard<Layer3D> lock(*this);
  unsubscribeUpdatesUnlocked();
  subscribeUpdatesUnlocked();
}

void OctomapServerLayer3D::reset()
{
  std::lock_guard<Layer3D> lock(*this);

  super::reset();

  // reset octomap server
  if (reset_client_.exists())
  {
    std_srvs::Empty srv;
    reset_client_.call(srv);
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ <<
                             ": Unable to call octomap server's reset service");
  }

  // Keep the layer non-current until the first map update cycle after the
  // first data message is received to keep from driving blind.
  current_ = false;
  last_update_stamp_ = ros::Time(0.0);
}

void OctomapServerLayer3D::resetBoundingBoxUnlocked(Costmap3DIndex min, Costmap3DIndex max)
{
  super::resetBoundingBoxUnlocked(min, max);

  if (erase_bbx_client_.exists())
  {
    octomap_msgs::BoundingBoxQuery srv;
    srv.request.min = fromOctomapPoint(costmap_->keyToCoord(min));
    srv.request.max = fromOctomapPoint(costmap_->keyToCoord(max));
    erase_bbx_client_.call(srv);
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ <<
                             ": Unable to call octomap server's erase_bbx service");
  }

  // Keep the layer non-current until the first map update cycle after the
  // first data message is received to keep from driving blind.
  current_ = false;
  last_update_stamp_ = ros::Time(0.0);
};

void OctomapServerLayer3D::matchSize(const geometry_msgs::Point& min, const geometry_msgs::Point& max, double resolution)
{
  if (!costmap_ || resolution != costmap_->getResolution())
  {
    // Resolution will change, need to request full message.
    // Do this by re-subscribing to the map topics.
    unsubscribe();
    subscribe();
  }
  super::matchSize(min, max, resolution);

  // If octomap server ever exposes its limits via dynamic reconfigure, we
  // could set them here.
}

void OctomapServerLayer3D::mapCallback(const octomap_msgs::OctomapConstPtr& map_msg)
{
  std::lock_guard<Layer3D> lock(*this);
  if (active_ && !using_updates_)
    mapUpdateInternal(map_msg.get(), nullptr);
}

void OctomapServerLayer3D::mapUpdateCallback(const octomap_msgs::OctomapUpdateConstPtr& map_update_msg)
{
  std::lock_guard<Layer3D> lock(*this);
  if (!active_)
  {
    // We have been deactivated, but there was still an outstanding message
    // callback. Throw this message away.
    return;
  }
  num_map_updates_++;
  if (!first_full_map_update_received_)
  {
    // Check if this the full map published by the connect callback.
    // That map is signaled by having the octomap_bounds seq be different
    // from the octomap_update seq.
    // Ignore the first normal published update until the first full map
    // is received.
    if (map_update_msg->octomap_bounds.header.seq == map_update_msg->octomap_update.header.seq)
    {
      if (num_map_updates_ <= 1)
      {
        // Ignore the first normal update until the first full map is received.
        // There is a race where this may happen, and we must simply ignore the
        // first.
        return;
      }
      else
      {
        // Huh. We never got the first full map, this means we lost that
        // message. We have no choice but to re-subscribe.
        ROS_WARN("Lost first full map update message, resubscribing.");
        scheduleResubscribeUpdates();
        return;
      }
    }
    first_full_map_update_received_ = true;
  }
  else
  {
    if (last_seq_ + 1 < map_update_msg->octomap_bounds.header.seq)
    {
      ROS_WARN("Lost an update message, resubscribing to get entire map.");
      ROS_INFO_STREAM("Expected sequence number " << last_seq_ + 1 << " but received "
                      << map_update_msg->octomap_bounds.header.seq);
      scheduleResubscribeUpdates();
      return;
    }
    else if(last_seq_ + 1 > map_update_msg->octomap_bounds.header.seq)
    {
      // We have moved backwards. This means either the server restarted, or
      // we are running from a bagfile and it was rewound. In either case,
      // the message should already have a universal bounds, so there is
      // nothing to do, as it will reset our memory for us.
    }
  }
  last_seq_ = map_update_msg->octomap_bounds.header.seq;

  if (!using_updates_)
  {
    using_updates_ = true;
    // now that we are using updates, there is no need for the map sub
    map_sub_.shutdown();
  }
  mapUpdateInternal(&map_update_msg->octomap_update, &map_update_msg->octomap_bounds);
}

void OctomapServerLayer3D::mapUpdateInternal(const octomap_msgs::Octomap* map_msg,
                                             const octomap_msgs::Octomap* bounds_msg)
{
  if (!changed_cells_ || !costmap_)
  {
    // No costmap to update yet
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": received update before costmap was setup, ignoring");
    return;
  }

  // Verify binary, resolution and frame match.
  if (!map_msg->binary)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": received non-binary map, ignoring");
  }
  else if (bounds_msg != nullptr && !bounds_msg->binary)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": received non-binary bounds map, ignoring");
  }
  else if(std::abs(map_msg->resolution - costmap_->getResolution()) > 1e-6)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": received map with resolution " <<
                             map_msg->resolution << " but costmap resolution is " <<
                             costmap_->getResolution() << ", ignoring");
  }
  else if(bounds_msg != nullptr && std::abs(bounds_msg->resolution - costmap_->getResolution()) > 1e-6)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": received bounds map with resolution " <<
                             map_msg->resolution << " but costmap resolution is " <<
                             costmap_->getResolution() << ", ignoring");
  }
  else if(map_msg->header.frame_id != layered_costmap_3d_->getGlobalFrameID())
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": received map in frame " <<
                             map_msg->header.frame_id << " but global frame is " <<
                             layered_costmap_3d_->getGlobalFrameID() << ", ignoring");
  }
  else if(bounds_msg != nullptr && bounds_msg->header.frame_id != layered_costmap_3d_->getGlobalFrameID())
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": received bounds map in frame " <<
                             bounds_msg->header.frame_id << " but global frame is " <<
                             layered_costmap_3d_->getGlobalFrameID() << ", ignoring");
  }
  else
  {
    std::shared_ptr<octomap::AbstractOcTree> abstract_map(octomap_msgs::binaryMsgToMap(*map_msg));
    octomap::OcTree* map = dynamic_cast<octomap::OcTree*>(abstract_map.get());
    if (map == nullptr)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": unable to dynamic cast received tree");
    }
    else
    {
      ROS_DEBUG_STREAM("received value octomap with size " << map->size());
      last_update_stamp_ = map_msg->header.stamp;
      std::shared_ptr<octomap::AbstractOcTree> abstract_bounds_map;
      if (bounds_msg != nullptr)
      {
        abstract_bounds_map.reset(octomap_msgs::binaryMsgToMap(*bounds_msg));
      }
      else
      {
        // If no bounds are given, use the whole universe
        Costmap3D* universe_bounds_map = new Costmap3D(costmap_->getResolution());
        universe_bounds_map->setNodeValueAtDepth(Costmap3DIndex(), 0, LETHAL);
        abstract_bounds_map.reset(universe_bounds_map);
      }
      octomap::OcTree* bounds_map = dynamic_cast<octomap::OcTree*>(abstract_bounds_map.get());
      if (bounds_map == nullptr)
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "OctomapServerLayer3D " << name_ << ": unable to dynamic cast received bounds");
      }
      else
      {
        ROS_DEBUG_STREAM("using bounds octomap with size " << bounds_map->size());
        // Use zero log odds as the threshold for any incoming binary map
        updateCells(map, bounds_map, 0.0);
        ROS_DEBUG_STREAM("after update cells, costmap_ has size " << costmap_->size());
        ROS_DEBUG_STREAM(" and changed_cells_ has size " << changed_cells_->size());
      }
    }
  }
}

}  // namespace costmap_3d
