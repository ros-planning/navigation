//
// drawing fns for nav fn
// 

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Window.H>
#include <FL/fl_draw.H>

#include <navfn/navfn.h>

namespace navfn {
  class NavWin 
    : public Fl_Double_Window
  {
    public:
      NavWin(int w, int h, const char *name);
      ~NavWin();

      int nw,nh;			// width and height of image
      int pw,ph;			// width and height of pot field
      int dec, inc;			// decimation or expansion for display

      float maxval;			// max potential value
      void drawPot(NavFn *nav);	// draw everything...

      void drawOverlay();

      uchar *im;			// image for drawing
      int *pc, *pn, *po;		// priority buffers
      int pce, pne, poe;		// buffer sizes
      int goal[2];
      int start[2];
      int *path;			// path buffer, cell indices
      int pathlen;			// how many we have
      int pathbuflen;		// how big the path buffer is

      void draw();			// draw the image
  };
};
