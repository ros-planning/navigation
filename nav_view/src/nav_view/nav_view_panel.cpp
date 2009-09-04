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

#include "nav_view_panel.h"

#include "ogre_tools/wx_ogre_render_window.h"

#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PoseArray.h"

#include <tf/transform_listener.h>

#include <OGRE/Ogre.h>

#include "tools.h"

#define MAP_DEPTH (-1.0f)
#define RADIUS_DEPTH (-0.99f)
#define CLOUD_DEPTH (-0.98f)
#define PATH_LINE_DEPTH (-0.97f)
#define LOCAL_PATH_DEPTH (-0.96f)
#define FOOTPRINT_DEPTH (-0.95f)
#define INFLATED_OBSTACLES_DEPTH (-0.94f)
#define RAW_OBSTACLES_DEPTH (-0.93f)
#define LASER_SCAN_DEPTH (-0.92f)

#define ROBOT_RADIUS (0.3f)

BEGIN_DECLARE_EVENT_TYPES()
DECLARE_EVENT_TYPE(EVT_RENDER, wxID_ANY)
END_DECLARE_EVENT_TYPES()

DEFINE_EVENT_TYPE(EVT_RENDER)

namespace nav_view
{

NavViewPanel::NavViewPanel( wxWindow* parent )
: NavViewPanelGenerated( parent )
, ogre_root_( Ogre::Root::getSingletonPtr() )
, map_object_( NULL )
, cloud_object_( NULL )
, radius_object_( NULL )
, global_plan_object_( NULL )
, local_plan_object_( NULL )
, footprint_object_( NULL )
, inflated_obstacles_object_( NULL )
, raw_obstacles_object_( NULL )
, laser_scan_object_( NULL )
, mouse_x_(0)
, mouse_y_(0)
, scale_(10.0f)
, current_tool_( NULL )
{
  tf_client_ = new tf::TransformListener();

  scene_manager_ = ogre_root_->createSceneManager( Ogre::ST_GENERIC );
  root_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  render_panel_ = new ogre_tools::wxOgreRenderWindow( ogre_root_, this );
  render_sizer_->Add( render_panel_, 1, wxALL|wxEXPAND, 0 );

  camera_ = scene_manager_->createCamera( "OrthoTopDown" );
  camera_->setProjectionType( Ogre::PT_ORTHOGRAPHIC );
  camera_->setPosition(0.0f, 0.0f, 30.0f);
  camera_->lookAt( 0.0f, 0.0f, 0.0f );
  camera_->setNearClipDistance(0.001f);
  camera_->setFarClipDistance(50.0f);

  render_panel_->getViewport()->setCamera( camera_ );

  nh_.param("/global_frame_id", global_frame_id_, std::string("/map"));

  particle_cloud_sub_ = nh_.subscribe("particlecloud", 1, &NavViewPanel::incomingPoseArray, this);

  global_plan_sub_.subscribe(nh_, "global_plan", 2);
  local_plan_sub_.subscribe(nh_, "local_plan", 2);
  robot_footprint_sub_.subscribe(nh_, "robot_footprint", 2);
  inflated_obs_sub_.subscribe(nh_, "inflated_obstacles", 2);
  raw_obs_sub_.subscribe(nh_, "obstacles", 2);
  global_plan_filter_.reset(new PathFilter(global_plan_sub_, *tf_client_, global_frame_id_, 2));
  local_plan_filter_.reset(new PathFilter(local_plan_sub_, *tf_client_, global_frame_id_, 2));
  robot_footprint_filter_.reset(new PolygonFilter(robot_footprint_sub_, *tf_client_, global_frame_id_, 2));
  inflated_obs_filter_.reset(new GridCellsFilter(inflated_obs_sub_, *tf_client_, global_frame_id_, 2));
  raw_obs_filter_.reset(new GridCellsFilter(raw_obs_sub_, *tf_client_, global_frame_id_, 2));

  global_plan_filter_->registerCallback(boost::bind(&NavViewPanel::incomingGuiPath, this, _1));
  local_plan_filter_->registerCallback(boost::bind(&NavViewPanel::incomingLocalPath, this, _1));
  robot_footprint_filter_->registerCallback(boost::bind(&NavViewPanel::incomingRobotFootprint, this, _1));
  inflated_obs_filter_->registerCallback(boost::bind(&NavViewPanel::incomingInflatedObstacles, this, _1));
  raw_obs_filter_->registerCallback(boost::bind(&NavViewPanel::incomingRawObstacles, this, _1));

  render_panel_->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MOTION, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );

  render_panel_->SetFocus();

  render_panel_->Connect( wxEVT_CHAR, wxKeyEventHandler( NavViewPanel::onChar ), NULL, this );

  Connect( EVT_RENDER, wxCommandEventHandler( NavViewPanel::onRender ), NULL, this );

  render_panel_->setOrthoScale( scale_ );

  update_timer_ = new wxTimer( this );
  Connect( update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler( NavViewPanel::onUpdate ), NULL, this );
  update_timer_->Start( 100 );

  createRadiusObject();

  current_tool_ = new MoveTool( this );
}

NavViewPanel::~NavViewPanel()
{
  Disconnect( EVT_RENDER, wxCommandEventHandler( NavViewPanel::onRender ), NULL, this );

  render_panel_->Disconnect( wxEVT_CHAR, wxKeyEventHandler( NavViewPanel::onChar ), NULL, this );

  render_panel_->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MOTION, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MIDDLE_UP, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );

  particle_cloud_sub_.shutdown();
  global_plan_sub_.unsubscribe();
  local_plan_sub_.unsubscribe();
  robot_footprint_sub_.unsubscribe();
  inflated_obs_sub_.unsubscribe();
  raw_obs_sub_.unsubscribe();
  global_plan_filter_.reset();
  local_plan_filter_.reset();
  robot_footprint_filter_.reset();
  inflated_obs_filter_.reset();
  raw_obs_filter_.reset();

  delete update_timer_;
  delete current_tool_;
  delete tf_client_;

  ogre_root_->destroySceneManager( scene_manager_ );
}

void NavViewPanel::queueRender()
{
  wxCommandEvent event( EVT_RENDER, GetId() );
  wxPostEvent( this, event );
}

void NavViewPanel::onRender( wxCommandEvent& event )
{
  render_panel_->Refresh();
}

void NavViewPanel::loadMap()
{
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...\n");
  if( !ros::service::call("/static_map", req, resp) )
  {
    ROS_INFO("request failed\n");

    return;
  }
  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
         resp.map.info.width,
         resp.map.info.height,
         resp.map.info.resolution);

  if (resp.map.info.width * resp.map.info.height == 0)
  {
    return;
  }

  map_resolution_ = resp.map.info.resolution;

  // Pad dimensions to power of 2
  map_width_ = resp.map.info.width;//(int)pow(2,ceil(log2(resp.map.info.width)));
  map_height_ = resp.map.info.height;//(int)pow(2,ceil(log2(resp.map.info.height)));

  //ROS_INFO("Padded dimensions to %d X %d\n", map_width_, map_height_);

  // Expand it to be RGB data
  int pixels_size = map_width_ * map_height_ * 3;
  unsigned char* pixels = new unsigned char[pixels_size];
  memset(pixels, 255, pixels_size);

  for(unsigned int j=0;j<resp.map.info.height;j++)
  {
    for(unsigned int i=0;i<resp.map.info.width;i++)
    {
      unsigned char val;
      if(resp.map.data[j*resp.map.info.width+i] == 100)
        val = 0;
      else if(resp.map.data[j*resp.map.info.width+i] == 0)
        val = 255;
      else
        val = 127;

      int pidx = 3*(j*map_width_ + i);
      pixels[pidx+0] = val;
      pixels[pidx+1] = val;
      pixels[pidx+2] = val;
    }
  }

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream( pixels, pixels_size ));
  static int tex_count = 0;
  std::stringstream ss;
  ss << "NavViewMapTexture" << tex_count++;
  map_texture_ = Ogre::TextureManager::getSingleton().loadRawData( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                   pixel_stream, map_width_, map_height_, Ogre::PF_BYTE_RGB, Ogre::TEX_TYPE_2D,
                                                                   0);

  delete [] pixels;

  Ogre::SceneNode* map_node = NULL;
  if ( !map_object_ )
  {
    static int map_count = 0;
    std::stringstream ss;
    ss << "NavViewMapObject" << map_count++;
    map_object_ = scene_manager_->createManualObject( ss.str() );
    map_node = root_node_->createChildSceneNode();
    map_node->attachObject( map_object_ );

    ss << "Material";
    map_material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    map_material_->setReceiveShadows(false);
    map_material_->getTechnique(0)->setLightingEnabled(false);
  }
  else
  {
    map_node = map_object_->getParentSceneNode();
  }

  Ogre::Pass* pass = map_material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > 0)
  {
    tex_unit = pass->getTextureUnitState(0);
  }
  else
  {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setTextureName(map_texture_->getName());
  tex_unit->setTextureFiltering( Ogre::TFO_NONE );

  map_object_->clear();
  map_object_->begin(map_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
  {
    // First triangle
    {
      // Top left
      map_object_->position( 0.0f, 0.0f, 0.0f );
      map_object_->textureCoord(0.0f, 0.0f);
      map_object_->normal( 0.0f, 0.0f, 1.0f );

      // Bottom right
      map_object_->position( map_resolution_*map_width_, map_resolution_*map_height_, 0.0f );
      map_object_->textureCoord(1.0f, 1.0f);
      map_object_->normal( 0.0f, 0.0f, 1.0f );

      // Bottom left
      map_object_->position( 0.0f, map_resolution_*map_height_, 0.0f );
      map_object_->textureCoord(0.0f, 1.0f);
      map_object_->normal( 0.0f, 0.0f, 1.0f );
    }

    // Second triangle
    {
      // Top left
      map_object_->position( 0.0f, 0.0f, 0.0f );
      map_object_->textureCoord(0.0f, 0.0f);
      map_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top right
      map_object_->position( map_resolution_*map_width_, 0.0f, 0.0f );
      map_object_->textureCoord(1.0f, 0.0f);
      map_object_->normal( 0.0f, 0.0f, 1.0f );

      // Bottom right
      map_object_->position( map_resolution_*map_width_, map_resolution_*map_height_, 0.0f );
      map_object_->textureCoord(1.0f, 1.0f);
      map_object_->normal( 0.0f, 0.0f, 1.0f );
    }
  }
  map_object_->end();

  root_node_->setPosition(Ogre::Vector3(-map_width_*map_resolution_/2, -map_height_*map_resolution_/2, 0.0f));
  map_node->setPosition(Ogre::Vector3(0.0f, 0.0f, MAP_DEPTH));

  queueRender();
}

void NavViewPanel::clearMap()
{
  if ( map_object_ )
  {
    scene_manager_->destroyManualObject( map_object_ );
    map_object_ = NULL;

    std::string tex_name = map_texture_->getName();
    map_texture_.setNull();
    Ogre::TextureManager::getSingleton().unload( tex_name );
  }
}

void NavViewPanel::onUpdate( wxTimerEvent& event )
{
  if ( !map_object_ )
  {
    static int count = 0;
    --count;

    if ( count < 0 )
    {
      loadMap();
      count = 100;
    }
  }

  updateRadiusPosition();

  ros::spinOnce();
}

void NavViewPanel::createRadiusObject()
{
  static int count = 0;
  std::stringstream ss;
  ss << "NavViewRadius" << count++;
  radius_object_ = scene_manager_->createManualObject( ss.str() );
  Ogre::SceneNode* node = root_node_->createChildSceneNode();
  node->attachObject( radius_object_ );

  radius_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );
  {
    Ogre::ColourValue color( 0.2, 1.0, 0.4 );
    for( float f = 0; f < Ogre::Math::TWO_PI; f += 0.5f )
    {
      radius_object_->position( ROBOT_RADIUS * cos(f), ROBOT_RADIUS * sin(f), 0.0f );
      radius_object_->colour(color);
    }

    radius_object_->position( ROBOT_RADIUS , 0.0f, 0.0f );
    radius_object_->colour(color);

    radius_object_->position( 0.0f, 0.0f, 0.0f );
    radius_object_->colour(color);
  }
  radius_object_->end();

  updateRadiusPosition();
}

void NavViewPanel::updateRadiusPosition()
{
  try
  {
    tf::Stamped<tf::Pose> robot_pose(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(), "base_link");
    tf::Stamped<tf::Pose> map_pose;

    tf_client_->transformPose(getGlobalFrame(), robot_pose, map_pose);
    double yaw, pitch, roll;
    map_pose.getBasis().getEulerZYX(yaw, pitch, roll);

    Ogre::SceneNode* node = radius_object_->getParentSceneNode();
    node->setPosition( Ogre::Vector3(map_pose.getOrigin().x(), map_pose.getOrigin().y(), RADIUS_DEPTH) );
    node->setOrientation( Ogre::Quaternion( Ogre::Radian( yaw ), Ogre::Vector3::UNIT_Z ) );

    queueRender();
  }
  catch ( tf::TransformException& e )
  {
  }
}

void NavViewPanel::createObjectFromPath(Ogre::ManualObject*& object, const nav_msgs::Path& path, const Ogre::ColourValue& color, float depth)
{
  if ( !object )
  {
    static int count = 0;
    std::stringstream ss;
    ss << "NavViewPath" << count++;
    object = scene_manager_->createManualObject( ss.str() );
    Ogre::SceneNode* node = root_node_->createChildSceneNode();
    node->attachObject( object );
  }

  object->clear();

  size_t num_points = path.poses.size();

  if ( num_points > 0 )
  {
    object->estimateVertexCount( num_points);
    object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );
    for( size_t i=0; i < num_points; ++i)
    {
      tf::Stamped<tf::Point> point;
      tf::pointMsgToTF(path.poses[i].pose.position, point);
      point.frame_id_ = path.header.frame_id;
      point.stamp_ = path.header.stamp;

      tf_client_->transformPoint(global_frame_id_, point, point);

      object->position(point.x(), point.y(), point.z());
      object->colour( color );
    }

    object->end();

    object->getParentSceneNode()->setPosition( Ogre::Vector3( 0.0f, 0.0f, depth ) );
  }

  queueRender();
}

void point32MsgToTF(const geometry_msgs::Point32& from, tf::Point& to)
{
  to.setX(from.x);
  to.setY(from.y);
  to.setZ(from.z);
}

void NavViewPanel::createObjectFromPolygon(Ogre::ManualObject*& object, const geometry_msgs::PolygonStamped& polygon, const Ogre::ColourValue& color, float depth)
{
  if ( !object )
  {
    static int count = 0;
    std::stringstream ss;
    ss << "NavViewPolygon" << count++;
    object = scene_manager_->createManualObject( ss.str() );
    Ogre::SceneNode* node = root_node_->createChildSceneNode();
    node->attachObject( object );
  }

  object->clear();

  size_t num_points = polygon.polygon.points.size();

  if ( num_points > 0 )
  {
    object->estimateVertexCount( num_points);
    object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );
    for( size_t i=0; i < num_points + 1; ++i)
    {
      tf::Stamped<tf::Point> point;
      point32MsgToTF(polygon.polygon.points[i % num_points], point);
      point.frame_id_ = polygon.header.frame_id;
      point.stamp_ = polygon.header.stamp;

      tf_client_->transformPoint(global_frame_id_, point, point);

      object->position(point.x(), point.y(), point.z());
      object->colour( color );
    }

    object->end();

    object->getParentSceneNode()->setPosition( Ogre::Vector3( 0.0f, 0.0f, depth ) );
  }

  queueRender();
}

void NavViewPanel::createObjectFromGridCells(Ogre::ManualObject*& object, const nav_msgs::GridCells& cells, const Ogre::ColourValue& color, float depth)
{
  if ( !object )
  {
    static int count = 0;
    std::stringstream ss;
    ss << "NavViewGridCells" << count++;
    object = scene_manager_->createManualObject( ss.str() );
    Ogre::SceneNode* node = root_node_->createChildSceneNode();
    node->attachObject( object );
  }

  object->clear();

  size_t num_points = cells.cells.size();

  if ( num_points > 0 )
  {
    object->estimateVertexCount( num_points);
    object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST );
    for( size_t i=0; i < num_points; ++i)
    {
      tf::Stamped<tf::Point> point;
      tf::pointMsgToTF(cells.cells[i], point);
      point.frame_id_ = cells.header.frame_id;
      point.stamp_ = cells.header.stamp;

      tf_client_->transformPoint(global_frame_id_, point, point);

      object->position(point.x(), point.y(), point.z());
      object->colour( color );
    }

    object->end();

    object->getParentSceneNode()->setPosition( Ogre::Vector3( 0.0f, 0.0f, depth ) );
  }

  queueRender();
}

void NavViewPanel::incomingPoseArray(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  if ( !cloud_object_ )
  {
    static int count = 0;
    std::stringstream ss;
    ss << "NavViewCloud" << count++;
    cloud_object_ = scene_manager_->createManualObject( ss.str() );
    Ogre::SceneNode* node = root_node_->createChildSceneNode();
    node->attachObject( cloud_object_ );
  }

  cloud_object_->clear();

  Ogre::ColourValue color( 1.0f, 0.0f, 0.0f, 1.0f );
  int num_particles = msg->poses.size();
  cloud_object_->estimateVertexCount( num_particles * 8 );
  cloud_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
  for( int i=0; i < num_particles; ++i)
  {
    Ogre::Vector3 pos( msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z );
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(msg->poses[i].orientation, orientation);
    double yaw, pitch, roll;
    btMatrix3x3(orientation).getEulerZYX(yaw, pitch, roll);
    Ogre::Quaternion orient( Ogre::Quaternion( Ogre::Radian( yaw ), Ogre::Vector3::UNIT_Z ) );

    Ogre::Vector3 vertices[8];
    vertices[0] = pos;
    vertices[1] = pos + orient * Ogre::Vector3(ROBOT_RADIUS, 0.0f, 0.0f);
    vertices[2] = vertices[1];
    vertices[3] = pos + orient * Ogre::Vector3(0.75*ROBOT_RADIUS, -0.25*ROBOT_RADIUS, 0.0f);
    vertices[4] = vertices[3];
    vertices[5] = pos + orient * Ogre::Vector3(0.75*ROBOT_RADIUS, 0.25*ROBOT_RADIUS, 0.0f);
    vertices[6] = vertices[5];
    vertices[7] = pos + orient * Ogre::Vector3(ROBOT_RADIUS, 0.0f, 0.0f);

    for ( int i = 0; i < 8; ++i )
    {
      cloud_object_->position( vertices[i] );
      cloud_object_->colour( color );
    }
  }
  cloud_object_->end();

  cloud_object_->getParentSceneNode()->setPosition( Ogre::Vector3( 0.0f, 0.0f, CLOUD_DEPTH ) );

  queueRender();
}

void NavViewPanel::incomingGuiPath(const nav_msgs::Path::ConstPtr& msg)
{
  createObjectFromPath( global_plan_object_, *msg, Ogre::ColourValue(0.0, 1.0, 0.0), PATH_LINE_DEPTH );
}

void NavViewPanel::incomingLocalPath(const nav_msgs::Path::ConstPtr& msg)
{
  createObjectFromPath( local_plan_object_, *msg, Ogre::ColourValue(0.0, 0.0, 1.0), LOCAL_PATH_DEPTH );
}

void NavViewPanel::incomingRobotFootprint(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
  createObjectFromPolygon( footprint_object_, *msg, Ogre::ColourValue(1.0, 0.0, 0.0), FOOTPRINT_DEPTH );
}

void NavViewPanel::incomingInflatedObstacles(const nav_msgs::GridCells::ConstPtr& msg)
{
  createObjectFromGridCells( inflated_obstacles_object_, *msg, Ogre::ColourValue(0.0, 0.0, 1.0), INFLATED_OBSTACLES_DEPTH );
}

void NavViewPanel::incomingRawObstacles(const nav_msgs::GridCells::ConstPtr& msg)
{
  createObjectFromGridCells( raw_obstacles_object_, *msg, Ogre::ColourValue(1.0, 0.0, 0.0), RAW_OBSTACLES_DEPTH );
}

void NavViewPanel::onRenderWindowMouseEvents( wxMouseEvent& event )
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event.GetX();
  mouse_y_ = event.GetY();

  int flags = current_tool_->processMouseEvent( event, last_x, last_y, scale_ );

  if ( flags & Tool::Render )
  {
    render_panel_->setOrthoScale( scale_ );
    queueRender();
  }

  if ( flags & Tool::Finished )
  {
    delete current_tool_;
    current_tool_ = new MoveTool( this );

    toolbar_->ToggleTool( ID_MOVE_TOOL, true );
  }
}

void NavViewPanel::onChar( wxKeyEvent& event )
{
  switch( event.GetKeyCode() )
  {
  case WXK_ESCAPE:
    {
      delete current_tool_;
      current_tool_ = new MoveTool( this );
      toolbar_->ToggleTool( ID_MOVE_TOOL, true );
    }
    break;
  case 'm':
    {
      delete current_tool_;
      current_tool_ = new MoveTool( this );
      toolbar_->ToggleTool( ID_MOVE_TOOL, true );
    }
    break;

  case 'g':
    {
      delete current_tool_;
      current_tool_ = new PoseTool( this, true );
      toolbar_->ToggleTool( ID_GOAL_TOOL, true );
    }
    break;

  case 'p':
    {
      delete current_tool_;
      current_tool_ = new PoseTool( this, false );
      toolbar_->ToggleTool( ID_POSE_TOOL, true );
    }
    break;

  default:
    event.Skip();
    break;
  }
}

void NavViewPanel::onToolClicked( wxCommandEvent& event )
{
  switch( event.GetId() )
  {
  case ID_MOVE_TOOL:
    {
      delete current_tool_;
      current_tool_ = new MoveTool( this );
    }
    break;

  case ID_GOAL_TOOL:
    {
      delete current_tool_;
      current_tool_ = new PoseTool( this, true );
    }
    break;

  case ID_POSE_TOOL:
    {
      delete current_tool_;
      current_tool_ = new PoseTool( this, false );
    }
    break;

  default:
    ROS_BREAK();
  }

  ROS_ASSERT( current_tool_ );
}

void NavViewPanel::onReloadMap( wxCommandEvent& event )
{
  clearMap();
  loadMap();
}

} // namespace nav_view
