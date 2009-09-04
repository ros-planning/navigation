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

/**
 * \file
 */

#include <wx/wx.h>

#include "ros/node.h"
#include "ros/common.h"
#include <ros/package.h>

#include "ogre_tools/initialization.h"

#include "nav_view_panel.h"

#include "OGRE/Ogre.h"

using namespace nav_view;

class MyFrame : public wxFrame
{
public:
  MyFrame(wxWindow* parent) : wxFrame(parent, -1, _("Nav View"),
                      wxDefaultPosition, wxSize(800,600),
                      wxDEFAULT_FRAME_STYLE)
  {
    ogre_tools::initializeOgre();

    nav_view_panel_ = new NavViewPanel( this );

    root_ = Ogre::Root::getSingletonPtr();

    ogre_tools::initializeResources( ogre_tools::V_string() );

  }

  ~MyFrame()
  {
    nav_view_panel_->Destroy();

    delete root_;
  }

private:

  Ogre::Root* root_;

  NavViewPanel* nav_view_panel_;
};

// our normal wxApp-derived class, as usual
class MyApp : public wxApp
{
public:
  char** localArgv;

  bool OnInit()
  {
    // create our own copy of argv, with regular char*s.
    localArgv =  new char*[ argc ];
    for ( int i = 0; i < argc; ++i )
    {
      localArgv[ i ] = strdup( wxString( argv[ i ] ).fn_str() );

      printf( "argv[%d]: %s\n", i, localArgv[i] );
    }

    ros::init(argc, localArgv, "nav_view");
    ros::NodeHandle nh;

    wxFrame* frame = new MyFrame(NULL);
    SetTopWindow(frame);
    frame->Show();

    return true;
  }

  int OnExit()
  {
    

    for ( int i = 0; i < argc; ++i )
    {
      free( localArgv[ i ] );
    }
    delete [] localArgv;

    return 0;
  }
};

DECLARE_APP(MyApp);
IMPLEMENT_APP(MyApp);

