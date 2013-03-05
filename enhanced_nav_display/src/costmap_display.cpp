/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreBillboardSet.h>

#include <tf/transform_listener.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/parse_color.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/display_context.h"

#include "costmap_display.h"

namespace enhanced_nav_display
{

Costmap::Costmap()
  : rviz::Display()
  , messages_received_(0)
  , last_frame_count_( uint64_t( -1 ))
{
  topic_property_ = new rviz::RosTopicProperty( "Topic", "",
                                          QString::fromStdString( ros::message_traits::datatype<nav_msgs::OccupancyGrid>() ),
                                          "nav_msgs::Occupancy topic to subscribe to. (also subscribed to <topic>_updates.",
                                          this, SLOT( updateTopic() ));

  hi_color_property_ = new rviz::ColorProperty( "High Color", QColor( 255, 0, 0 ),
                                               "Color of the highest grid cells.", this );
  lo_color_property_ = new rviz::ColorProperty( "Low Color", QColor( 0, 0, 255 ),
                                               "Color of the lowest grid cells.", this );

  max_property_ = new rviz::FloatProperty( "Max Value", 255.0,
                                       "Highest Value to Display.",
                                       this, SLOT( updateBounds() ));
  min_property_ = new rviz::FloatProperty( "Min Value", 0,
                                       "Lowest Value to Display.",
                                       this, SLOT( updateBounds() ));
                                       
  use_special_property_ = new rviz::BoolProperty( "Use Special Value", false, "Use Special Value or not", this);
  
  special_value_property_ = new rviz::FloatProperty( "Special Value", 0,
                                       "Special Value to Display in Special Color.",
                                       use_special_property_);

  special_color_property_ = new rviz::ColorProperty( "Special Color", QColor( 0, 255, 0 ),
                                               "Color of the special grid cells.", use_special_property_ );

}

void Costmap::onInitialize()
{
  grid_filter_ = new tf::MessageFilter<nav_msgs::OccupancyGrid>( *context_->getTFClient(), fixed_frame_.toStdString(),
                                                           10, update_nh_ );

  static int count = 0;
  std::stringstream ss;
  ss << "PolyLine" << count++;

  cloud_ = new rviz::PointCloud();
  cloud_->setRenderMode( rviz::PointCloud::RM_TILES );
  cloud_->setCommonDirection( Ogre::Vector3::UNIT_Z );
  cloud_->setCommonUpVector( Ogre::Vector3::UNIT_Y );
  scene_node_->attachObject( cloud_ );
  updateBounds();

  grid_filter_->connectInput( grid_sub_ );
  grid_filter_->registerCallback( boost::bind( &Costmap::incomingGrid, this, _1 ));
  context_->getFrameManager()->registerFilterForTransformStatusCheck( grid_filter_, this );
}

Costmap::~Costmap()
{
  unsubscribe();
  clear();
  scene_node_->detachObject( cloud_ );
  delete cloud_;
  delete grid_filter_;
}

void Costmap::clear()
{
  cloud_->clear();

  messages_received_ = 0;
  setStatus( rviz::StatusProperty::Warn, "Topic", "No messages received" );
}

void Costmap::updateTopic()
{
  unsubscribe();
  subscribe();
  context_->queueRender();
}

void Costmap::updateBounds()
{
  min_v_ = min_property_->getFloat(); 
  max_v_ = max_property_->getFloat(); 
  context_->queueRender();
}

void Costmap::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    grid_sub_.subscribe( update_nh_, topic_property_->getTopicStd(), 1 );
    setStatus( rviz::StatusProperty::Ok, "Topic", "OK" );
  }
  catch( ros::Exception& e )
  {
    setStatus( rviz::StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what() );
  }
  
  if(topic_property_->getTopicStd().length()>0)
  update_sub_ = update_nh_.subscribe(std::string(topic_property_->getTopicStd() + "_updates"), 10, &Costmap::incomingUpdate, this);
}

void Costmap::unsubscribe()
{
  grid_sub_.unsubscribe();
  update_sub_.shutdown();
}

void Costmap::onEnable()
{
  subscribe();
}

void Costmap::onDisable()
{
  unsubscribe();
  clear();
}

void Costmap::fixedFrameChanged()
{
  clear();

  grid_filter_->setTargetFrame( fixed_frame_.toStdString() );
}

void Costmap::incomingGrid( const nav_msgs::OccupancyGrid::ConstPtr& msg )
{
  if( !msg )
  {
    return;
  }

  ++messages_received_;

  if( context_->getFrameCount() == last_frame_count_ )
  {
    return;
  }
  last_frame_count_ = context_->getFrameCount();

  cloud_->clear();

  setStatus( rviz::StatusProperty::Ok, "Topic", QString::number( messages_received_ ) + " messages received" );

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !context_->getFrameManager()->getTransform( msg->header, position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
  }

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  /*if( msg->cell_width == 0 )
  {
    setStatus(rviz::StatusProperty::Error, "Topic", "Cell width is zero, cells will be invisible.");
  }*/

  cloud_->setDimensions(msg->info.resolution, msg->info.resolution, 0.0);

  Ogre::ColourValue hi_color = rviz::qtToOgre( hi_color_property_->getColor() );
  Ogre::ColourValue lo_color = rviz::qtToOgre( lo_color_property_->getColor() );
  Ogre::ColourValue color;
  color.a = 1.0;
  uint32_t num_points = msg->data.size();

  typedef std::vector< rviz::PointCloud::Point > V_Point;
  V_Point points;
  points.resize( num_points );
  
  bool use_special = use_special_property_->getBool();
  Ogre::ColourValue sp_color;
  float sp_value = 0;
  if(use_special){
    sp_color = rviz::qtToOgre( special_color_property_->getColor() );
    sp_value = special_value_property_->getFloat(); 
  }
  
  tf::Transformer t;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->info.origin.position.x, msg->info.origin.position.y, msg->info.origin.position.z) );
  transform.setRotation( tf::Quaternion(msg->info.origin.orientation.x, msg->info.origin.orientation.y, msg->info.origin.orientation.z, msg->info.origin.orientation.w) );
  ros::Time now = ros::Time::now();
  t.setTransform(tf::StampedTransform(transform, now, "world", "grid"));


  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
  double resolution = msg->info.resolution;

  for(uint32_t i = 0; i < num_points; i++)
  {
    int x = i % msg->info.width;
    int y = i / msg->info.width;
    
    transform.setOrigin( tf::Vector3(x*resolution, y*resolution, 0) );
    t.setTransform(tf::StampedTransform(transform, now, "grid", "pt"));
    tf::StampedTransform result;
    t.lookupTransform("world", "pt", ros::Time(0), result);
        
    rviz::PointCloud::Point& current_point = points[ i ];
    current_point.position.x = result.getOrigin().x();
    current_point.position.y = result.getOrigin().y();
    current_point.position.z = result.getOrigin().z();
   
    float value = (unsigned char) msg->data[i];

    if(use_special && fabs(value-sp_value)<.01){
        current_point.color = sp_color;
    }else if(value <= max_v_ and value >= min_v_){

        float pct = (value - min_v_) / (max_v_ - min_v_);
        color.r = lo_color.r - pct * (lo_color.r - hi_color.r);
        color.g = lo_color.g - pct * (lo_color.g - hi_color.g);
        color.b = lo_color.b - pct * (lo_color.b - hi_color.b);
        current_point.color = color;
    }else{
        current_point.color.a = 0.0;
    }
    
  }

  cloud_->clear();

  if ( !points.empty() )
  {
    cloud_->addPoints( &points.front(), points.size() );
  }
}

void Costmap::incomingUpdate(const map_msgs::OccupancyGridUpdate::ConstPtr& msg){

}

void Costmap::reset()
{
  rviz::Display::reset();
  clear();
}

} // namespace enhanced_nav_display

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( enhanced_nav_display::Costmap, rviz::Display )
