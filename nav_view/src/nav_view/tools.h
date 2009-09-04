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

#ifndef NAV_VIEW_TOOLS_H
#define NAV_VIEW_TOOLS_H

#include <OGRE/OgreVector3.h>

#include <ros/ros.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
}

namespace ros
{
class Node;
}

namespace ogre_tools
{
class Arrow;
}

class wxMouseEvent;
class wxWindow;

namespace nav_view
{

class NavViewPanel;

class Tool
{
public:
  Tool( NavViewPanel* panel );
  virtual ~Tool() {}

  enum Flags
  {
    Render = 1 << 0,
    Finished = 1 << 1
  };
  virtual int processMouseEvent( wxMouseEvent& event, int last_x, int last_y, float& scale ) = 0;
protected:
  Ogre::SceneManager* scene_manager_;
  ros::NodeHandle nh_;
  NavViewPanel* panel_;
};

class MoveTool : public Tool
{
public:
  MoveTool( NavViewPanel* panel );

  virtual int processMouseEvent( wxMouseEvent& event, int last_x, int last_y, float& scale );
};

class PoseTool : public Tool
{
public:
  PoseTool( NavViewPanel* panel, bool goal );
  virtual ~PoseTool();

  virtual int processMouseEvent( wxMouseEvent& event, int last_x, int last_y, float& scale );

protected:
  Ogre::Vector3 getPositionFromMouseXY( int mouse_x, int mouse_y );

  ogre_tools::Arrow* arrow_;

  enum State
  {
    Position,
    Orientation
  };
  State state_;

  Ogre::Vector3 pos_;

  bool is_goal_;

  ros::Publisher goal_pub_;
  ros::Publisher pose_pub_;
};

} // nav_view

#endif
