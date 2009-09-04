/*********************************************************************
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
*********************************************************************/

/** \author Ioan Sucan */

/**

@mainpage

@htmlinclude manifest.html

@b odom_localization Takes in ground truth pose information for a robot
base (e.g., from a simulator or motion capture system) and republishes
it as if a localization system were in use.

<hr>

@section usage Usage
@verbatim
$ fake_localization
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "base_pose_ground_truth" nav_msgs/Odometry : robot's odometric pose.  Only the position information is used (velocity is ignored).

Publishes to (name / type):
- @b "amcl_pose" geometry_msgs/PoseWithCovarianceStamped : robot's estimated pose in the map, with covariance
- @b "particlecloud" geometry_msgs/PoseArray : fake set of poses being maintained by the filter (one paricle only).

<hr>

@section parameters ROS parameters

- "~odom_frame_id" (string) : The odometry frame to be used, default: "odom"

 **/

#include <ros/node.h>
#include <ros/time.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <angles/angles.h>

#include "ros/console.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_notifier.h"


class FakeOdomNode: public ros::Node
{
public:
    FakeOdomNode(void) : ros::Node("fake_localization")
    {
      advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1);
      advertise<geometry_msgs::PoseArray>("particlecloud",1);
      m_tfServer = new tf::TransformBroadcaster();	
      m_tfListener = new tf::TransformListener(*this);
      m_lastUpdate = ros::Time::now();
      
      m_base_pos_received = false;

      param("~odom_frame_id", odom_frame_id_, std::string("odom"));
      m_particleCloud.header.stamp = ros::Time::now();
      m_particleCloud.header.frame_id = "/map";
      m_particleCloud.set_poses_size(1);
      notifier = new tf::MessageNotifier<nav_msgs::Odometry>(m_tfListener, this, 
                                                                           boost::bind(&FakeOdomNode::update, this, _1),
                                                                           "",//empty topic it will be manually stuffed
                                                                           odom_frame_id_, 100);
      subscribe("base_pose_ground_truth", m_basePosMsg, &FakeOdomNode::basePosReceived,1);
    }
    
    ~FakeOdomNode(void)
    {
      if (m_tfServer)
        delete m_tfServer; 
    }
   

  // Just kill time as spin is not working!
    void run(void)
    {
      // A duration for sleeping will be 100 ms
      ros::Duration snoozer;
      snoozer.fromSec(0.1);

      while(true){
	snoozer.sleep();
      }
    }  

    
private:
    tf::TransformBroadcaster       *m_tfServer;
    tf::TransformListener          *m_tfListener;
    tf::MessageNotifier<nav_msgs::Odometry>* notifier;
  
    ros::Time                      m_lastUpdate;
    double                         m_maxPublishFrequency;
    bool                           m_base_pos_received;
    
    nav_msgs::Odometry  m_basePosMsg;
    geometry_msgs::PoseArray      m_particleCloud;
    geometry_msgs::PoseWithCovarianceStamped      m_currentPos;

    //parameter for what odom to use
    std::string odom_frame_id_;
    
  void basePosReceived()
  {
    m_basePosMsg.header.frame_id = "base_footprint"; //hack to make the notifier do what I want (changed back later)
    boost::shared_ptr<nav_msgs::Odometry>  message(new nav_msgs::Odometry);
    *message = m_basePosMsg;
    notifier->enqueueMessage(message);
    //    update();
  }
public:
  void update(const tf::MessageNotifier<nav_msgs::Odometry>::MessagePtr & message){
    tf::Transform txi(tf::Quaternion(message->pose.pose.orientation.x,
				     message->pose.pose.orientation.y, 
				     message->pose.pose.orientation.z, 
				     message->pose.pose.orientation.w),
		      tf::Point(message->pose.pose.position.x,
				message->pose.pose.position.y,
                                0.0*message->pose.pose.position.z )); // zero height for base_footprint

    double x = txi.getOrigin().x();
    double y = txi.getOrigin().y();
    double z = txi.getOrigin().z();

    double yaw, pitch, roll;
    txi.getBasis().getEulerZYX(yaw, pitch, roll);
    yaw = angles::normalize_angle(yaw);

    tf::Transform txo(tf::Quaternion(yaw, pitch, roll), tf::Point(x, y, z));
    
    //tf::Transform txIdentity(tf::Quaternion(0, 0, 0), tf::Point(0, 0, 0));
    
    // Here we directly publish a transform from Map to base_link. We will skip the intermediate step of publishing a transform
    // to the base footprint, as it seems unnecessary. However, if down the road we wish to use the base footprint data,
    // and some other component is publishing it, this should change to publish the map -> base_footprint instead.
    // A hack links the two frames.
    // m_tfServer->sendTransform(tf::Stamped<tf::Transform>
    //                           (txIdentity.inverse(),
    //                            message->header.stamp,
    //                            "base_footprint", "base_link"));  // this is published by base controller
    // subtracting base to odom from map to base and send map to odom instead
    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
      m_tfListener->transformPose(odom_frame_id_,tf::Stamped<tf::Pose> (txo.inverse(),
                                                                message->header.stamp, "base_footprint"),odom_to_map);
    }
    catch(tf::TransformException &e){
      ROS_DEBUG("Failed to transform to odom %s\n",e.what());

      return;
    }

    m_tfServer->sendTransform(tf::Stamped<tf::Transform>
			      (odom_to_map.inverse(),
			       message->header.stamp,
			       odom_frame_id_, "/map"));

    // Publish localized pose
    m_currentPos.header = message->header;
    m_currentPos.header.frame_id = "/map"; ///\todo fixme hack
    m_currentPos.pose.pose.position.x = x;
    m_currentPos.pose.pose.position.y = y;
    // Leave z as zero
    tf::quaternionTFToMsg(tf::Quaternion(yaw, 0.0, 0.0),
                          m_currentPos.pose.pose.orientation);
    // Leave covariance as zero
    publish("amcl_pose", m_currentPos);

    // The particle cloud is the current position. Quite convenient.
    m_particleCloud.poses[0] = m_currentPos.pose.pose;
    publish("particlecloud", m_particleCloud);
  }   
};

int main(int argc, char** argv)
{
    ros::init(argc, argv);
    
    FakeOdomNode odom;
    odom.spin();

    
    
    return 0;
}
