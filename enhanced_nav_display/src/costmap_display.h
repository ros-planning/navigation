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


#ifndef COSTMAP_DISPLAY_H
#define COSTMAP_DISPLAY_H

#include <rviz/display.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/MapMetaData.h>

#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#endif

#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/ros_topic_property.h"
#include <boost/shared_ptr.hpp>

typedef std::vector< rviz::PointCloud::Point > V_Point;

namespace Ogre
{
class ManualObject;
}

namespace enhanced_nav_display
{
/**
 * \class Costmap
 * \brief Displays a nav_msgs::GridCells message
 */
class Costmap : public rviz::Display
{
Q_OBJECT
public:
  Costmap();
  virtual ~Costmap();

  virtual void onInitialize();

  // Overrides from Display
  virtual void fixedFrameChanged();
  virtual void reset();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

private Q_SLOTS:
  void updateBounds();
  void updateTopic();
  void updateAlpha();

private:
  void subscribe();
  void unsubscribe();
  void clear();
  void incomingGrid( const nav_msgs::OccupancyGrid::ConstPtr& msg );
  void incomingUpdate( const map_msgs::OccupancyGridUpdate::ConstPtr& msg );
  
  bool updateBaseColors();
  void colorAll();
  void getColor(Ogre::ColourValue& color, char value);

  std::vector< char > values_;
  V_Point points_;
  rviz::PointCloud* cloud_;
  
  Ogre::ColourValue hi_color_, lo_color_, sp_color_;
  bool use_special_;
  int sp_value_;

  message_filters::Subscriber<nav_msgs::OccupancyGrid> grid_sub_;
  tf::MessageFilter<nav_msgs::OccupancyGrid>* grid_filter_;
  
  ros::Subscriber update_sub_;

  rviz::FloatProperty* alpha_property_;
  rviz::ColorProperty* hi_color_property_;
  rviz::ColorProperty* lo_color_property_;
  rviz::RosTopicProperty* topic_property_;
  rviz::IntProperty* max_property_;
  rviz::IntProperty* min_property_;
  rviz::BoolProperty* use_special_property_;
  rviz::IntProperty* special_value_property_;
  rviz::ColorProperty* special_color_property_;
  
  int width_;
  char min_v_, max_v_;

  uint32_t messages_received_;
  uint64_t last_frame_count_;
};

} // namespace enhanced_nav_display

#endif /* COSTMAP_DISPLAY_H */

