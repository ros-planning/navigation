///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "nav_view_panel_generated.h"

///////////////////////////////////////////////////////////////////////////
using namespace nav_view;

NavViewPanelGenerated::NavViewPanelGenerated( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxVERTICAL );
	
	toolbar_ = new wxToolBar( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTB_HORIZONTAL|wxTB_NOICONS|wxTB_TEXT ); 
	toolbar_->AddTool( ID_MOVE_TOOL, wxT("Move"), wxNullBitmap, wxNullBitmap, wxITEM_RADIO, wxT("Use the mouse to move around the map.  Left click = move, right click = zoom."), wxEmptyString );
	toolbar_->AddTool( ID_GOAL_TOOL, wxT("Set Goal"), wxNullBitmap, wxNullBitmap, wxITEM_RADIO, wxT("Use the mouse to set the goal.  Click and drag to set the position and orientation."), wxEmptyString );
	toolbar_->AddTool( ID_POSE_TOOL, wxT("Set Pose"), wxNullBitmap, wxNullBitmap, wxITEM_RADIO, wxEmptyString, wxEmptyString );
	toolbar_->AddSeparator();
	toolbar_->AddTool( ID_RELOAD_MAP, wxT("Reload Map"), wxNullBitmap, wxNullBitmap, wxITEM_NORMAL, wxEmptyString, wxEmptyString );
	toolbar_->Realize();
	
	bSizer1->Add( toolbar_, 0, wxEXPAND, 5 );
	
	render_sizer_ = new wxBoxSizer( wxVERTICAL );
	
	bSizer1->Add( render_sizer_, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer1 );
	this->Layout();
	
	// Connect Events
	this->Connect( ID_MOVE_TOOL, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( NavViewPanelGenerated::onToolClicked ) );
	this->Connect( ID_GOAL_TOOL, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( NavViewPanelGenerated::onToolClicked ) );
	this->Connect( ID_POSE_TOOL, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( NavViewPanelGenerated::onToolClicked ) );
	this->Connect( ID_RELOAD_MAP, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( NavViewPanelGenerated::onReloadMap ) );
}

NavViewPanelGenerated::~NavViewPanelGenerated()
{
	// Disconnect Events
	this->Disconnect( ID_MOVE_TOOL, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( NavViewPanelGenerated::onToolClicked ) );
	this->Disconnect( ID_GOAL_TOOL, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( NavViewPanelGenerated::onToolClicked ) );
	this->Disconnect( ID_POSE_TOOL, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( NavViewPanelGenerated::onToolClicked ) );
	this->Disconnect( ID_RELOAD_MAP, wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( NavViewPanelGenerated::onReloadMap ) );
}
