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

#ifndef NAV_VIEW_NAV_VIEW_PANEL_H
#define NAV_VIEW_NAV_VIEW_PANEL_H

#include "nav_view_panel_generated.h"

#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRenderOperation.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <ros/ros.h>

#include <list>

/**

@mainpage

@htmlinclude manifest.html

@b nav_view is a GUI for 2-D navigation.  It can:
 - display a map
 - display a robot's pose
 - display a cloud of particles (e.g., from a localization system)
 - display a path (e.g., from a path planner)
 - display a set of obstacles (and inflated obstacles)
 - send a goal to a planner

<hr>

@section usage Usage
@verbatim
$ nav_view
@endverbatim

@par Example

@verbatim
$ nav_view
@endverbatim

@par GUI controls
Mouse controls:
 - Movement tool (shortcut: m)
  - Left mouse -- pan the view
  - Right mosue -- zoom the view
 - Goal tool (shortcut: g)
  - Left mouse -- click and drag to select position + heading
 - Pose tool (shortcut: p)
  - Left mouse -- click and drag to select initial position + heading estimate

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "particlecloud" - geometry_msgs/PoseArray : a set particles from a probabilistic localization system.  Rendered is a set of small arrows.
- @b "global_plan" - nav_msgs/Path : a path from a planner.  Rendered as a line.
- @b "local_plan" - nav_msgs/Path: local path from a planner.  Rendered as a line.
- @b "robot_footprint" - geometry_msgs/Polygon : Box "footprint" around the robot.  Rendered as a dashed line
- @b "obstacles" - nav_msgs/GridCells : Raw obstacle data.  Rendered as points
- @b "inflated_obstacles" - nav_msgs/GridCells : Inflated obstacle data. Rendered as points

Publishes to (name / type):
- @b "goal" - geometry_msgs/PoseStamped : goal for planner.  Sent when using the Goal tool
- @b "initialpose" - geometry_msgs/PoseWithCovarianceStamped : pose to initialize localization system.  Sent when using the Pose tool

<hr>

@section parameters ROS parameters

- \b /global_frame_id - If set, everything is transformed to this frame.  Defaults to "/map"

 **/
namespace ogre_tools
{
class wxOgreRenderWindow;
}

namespace Ogre
{
class Root;
class SceneManager;
class Camera;
class ManualObject;
}

namespace tf
{
class TransformListener;
}

class wxTimer;
class wxTimerEvent;

namespace nav_view
{

class Tool;

class NavViewPanel : public NavViewPanelGenerated
{
public:
  NavViewPanel( wxWindow* parent );
  virtual ~NavViewPanel();

  void setTransient( bool transient );

  void queueRender();

  float getScale() { return scale_; }
  Ogre::SceneManager* getSceneManager() { return scene_manager_; }
  Ogre::Camera* getCamera() { return camera_; }
  Ogre::SceneNode* getRootNode() { return root_node_; }
  ogre_tools::wxOgreRenderWindow* getRenderPanel() { return render_panel_; }
  float getMapResolution() { return map_resolution_; }
  int getMapWidth() { return map_width_; }
  int getMapHeight() { return map_height_; }
  const std::string& getGlobalFrame() { return global_frame_id_; }

protected:

  void onRender( wxCommandEvent& event );
  void onUpdate( wxTimerEvent& event );
  virtual void onToolClicked( wxCommandEvent& event );
  virtual void onReloadMap( wxCommandEvent& event );
  virtual void onChar( wxKeyEvent& event );

  void loadMap();
  void clearMap();
  void incomingPoseArray(const geometry_msgs::PoseArray::ConstPtr& msg);
  void incomingGuiPath(const nav_msgs::Path::ConstPtr& msg);
  void incomingLocalPath(const nav_msgs::Path::ConstPtr& msg);
  void incomingRobotFootprint(const geometry_msgs::PolygonStamped::ConstPtr& msg);
  void incomingInflatedObstacles(const nav_msgs::GridCells::ConstPtr& msg);
  void incomingRawObstacles(const nav_msgs::GridCells::ConstPtr& msg);

  void onRenderWindowMouseEvents( wxMouseEvent& event );

  void createRadiusObject();
  void updateRadiusPosition();

  void createObjectFromPath(Ogre::ManualObject*& object, const nav_msgs::Path& path, const Ogre::ColourValue& color, float depth);
  void createObjectFromPolygon(Ogre::ManualObject*& object, const geometry_msgs::PolygonStamped& polygon, const Ogre::ColourValue& color, float depth);
  void createObjectFromGridCells(Ogre::ManualObject*& object, const nav_msgs::GridCells& cells, const Ogre::ColourValue& color, float depth);

  void createTransientObject();

  Ogre::Root* ogre_root_;
  Ogre::SceneManager* scene_manager_;
  Ogre::Camera* camera_;
  ogre_tools::wxOgreRenderWindow* render_panel_;

  ros::NodeHandle nh_;
  tf::TransformListener* tf_client_;

  float map_resolution_;
  int map_width_;
  int map_height_;
  Ogre::TexturePtr map_texture_;

  ros::Subscriber particle_cloud_sub_;

  typedef tf::MessageFilter<nav_msgs::Path> PathFilter;
  typedef boost::shared_ptr<PathFilter> PathFilterPtr;
  typedef tf::MessageFilter<nav_msgs::GridCells> GridCellsFilter;
  typedef boost::shared_ptr<GridCellsFilter> GridCellsFilterPtr;
  typedef tf::MessageFilter<geometry_msgs::PolygonStamped> PolygonFilter;
  typedef boost::shared_ptr<PolygonFilter> PolygonFilterPtr;
  message_filters::Subscriber<nav_msgs::Path> global_plan_sub_;
  PathFilterPtr global_plan_filter_;
  message_filters::Subscriber<nav_msgs::Path> local_plan_sub_;
  PathFilterPtr local_plan_filter_;
  message_filters::Subscriber<geometry_msgs::PolygonStamped> robot_footprint_sub_;
  PolygonFilterPtr robot_footprint_filter_;
  message_filters::Subscriber<nav_msgs::GridCells> inflated_obs_sub_;
  GridCellsFilterPtr inflated_obs_filter_;
  message_filters::Subscriber<nav_msgs::GridCells> raw_obs_sub_;
  GridCellsFilterPtr raw_obs_filter_;

  Ogre::ManualObject* map_object_;
  Ogre::MaterialPtr map_material_;

  Ogre::ManualObject* cloud_object_;
  Ogre::ManualObject* radius_object_;
  Ogre::ManualObject* global_plan_object_;
  Ogre::ManualObject* local_plan_object_;
  Ogre::ManualObject* footprint_object_;
  Ogre::ManualObject* inflated_obstacles_object_;
  Ogre::ManualObject* raw_obstacles_object_;
  Ogre::ManualObject* laser_scan_object_;

  Ogre::SceneNode* root_node_;

  // Mouse handling
  int mouse_x_;                                           ///< X position of the last mouse event
  int mouse_y_;                                           ///< Y position of the last mouse event

  float scale_;

  // Global frame (usually "map")
  std::string global_frame_id_;

  wxTimer* update_timer_;

  Tool* current_tool_;
};

} // namespace nav_view

#endif
