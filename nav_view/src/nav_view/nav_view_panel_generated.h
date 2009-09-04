///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __nav_view_panel_generated__
#define __nav_view_panel_generated__

#include <wx/string.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/gdicmn.h>
#include <wx/toolbar.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/sizer.h>
#include <wx/panel.h>

///////////////////////////////////////////////////////////////////////////

namespace nav_view
{
	#define ID_MOVE_TOOL 1000
	#define ID_GOAL_TOOL 1001
	#define ID_POSE_TOOL 1002
	#define ID_RELOAD_MAP 1003
	
	///////////////////////////////////////////////////////////////////////////////
	/// Class NavViewPanelGenerated
	///////////////////////////////////////////////////////////////////////////////
	class NavViewPanelGenerated : public wxPanel 
	{
		private:
		
		protected:
			wxToolBar* toolbar_;
			wxBoxSizer* render_sizer_;
			
			// Virtual event handlers, overide them in your derived class
			virtual void onToolClicked( wxCommandEvent& event ){ event.Skip(); }
			virtual void onReloadMap( wxCommandEvent& event ){ event.Skip(); }
			
		
		public:
			NavViewPanelGenerated( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 665,500 ), long style = wxTAB_TRAVERSAL|wxWANTS_CHARS );
			~NavViewPanelGenerated();
		
	};
	
} // namespace nav_view

#endif //__nav_view_panel_generated__
