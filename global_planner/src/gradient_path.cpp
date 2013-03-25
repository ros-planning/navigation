
#include <global_planner/gradient_path.h>
#include <algorithm>
#include <stdio.h>
#include <global_planner/planner_core.h>

namespace global_planner {

GradientPath::GradientPath() : pathStep(0.5), COST_OBS(253) {
    gradx = grady = NULL;
}

GradientPath::~GradientPath(){

    if(gradx)
      delete[] gradx;
    if(grady)
      delete[] grady;
}

void GradientPath::setSize(int xs, int ys){
    Traceback::setSize(xs, ys);
     if(gradx)
        delete[] gradx;
      if(grady)
        delete[] grady;
      gradx = new float[xs*ys];
      grady = new float[xs*ys];
}

bool GradientPath::getPath(float* potential, int end_x, int end_y, std::vector<std::pair<float, float> >& path){
    std::pair<float, float> current;
    current.first = end_x;
    current.second = end_y;
    int stc = getIndex(end_x, end_y);
    
    // set up offset
      float dx=0;
      float dy=0;
    path.push_back(current);
    
    while(1){
        // check if near goal
        int nearest_point = getNearestPoint(stc, dx, dy);
        if(potential[nearest_point] == 0){
            current.first = nearest_point % xs_;
            current.second = nearest_point / xs_;
            path.push_back(current);
            return true;
        }
        
        if (stc < xs_ || stc > xs_*ys_-xs_) // would be out of bounds
        {
          printf("[PathCalc] Out of bounds\n");
          return false;
        }
        
        current.first =  stc%xs_ + dx;
        current.second = stc/xs_ + dy;
        
        ROS_INFO("%d %d | %f %f ", stc%xs_, stc/xs_, dx, dy);
        
        path.push_back(current);
        
        bool oscillation_detected = false;
        int npath = path.size();
        if( npath > 2 &&
            path[npath-1].first  == path[npath-3].first &&
            path[npath-1].second == path[npath-3].second )
        {
          ROS_DEBUG("[PathCalc] oscillation detected, attempting fix.");
          oscillation_detected = true;
        }
        
        int stcnx = stc+xs_;
        int stcpx = stc-xs_;
        
                // check for potentials at eight positions near cell
        if (potential[stc] >= POT_HIGH ||
            potential[stc+1] >= POT_HIGH ||
            potential[stc-1] >= POT_HIGH ||
            potential[stcnx] >= POT_HIGH ||
            potential[stcnx+1] >= POT_HIGH ||
            potential[stcnx-1] >= POT_HIGH ||
            potential[stcpx] >= POT_HIGH ||
            potential[stcpx+1] >= POT_HIGH ||
            potential[stcpx-1] >= POT_HIGH || oscillation_detected)
        {
          ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", potential[stc], (int) path.size());
          // check eight neighbors to find the lowest
          int minc = stc;
          int minp = potential[stc];
          int st = stcpx - 1;
          if (potential[st] < minp) {minp = potential[st]; minc = st; }
          st++;
          if (potential[st] < minp) {minp = potential[st]; minc = st; }
          st++;
          if (potential[st] < minp) {minp = potential[st]; minc = st; }
          st = stc-1;
          if (potential[st] < minp) {minp = potential[st]; minc = st; }
          st = stc+1;
          if (potential[st] < minp) {minp = potential[st]; minc = st; }
          st = stcnx-1;
          if (potential[st] < minp) {minp = potential[st]; minc = st; }
          st++;
          if (potential[st] < minp) {minp = potential[st]; minc = st; }
          st++;
          if (potential[st] < minp) {minp = potential[st]; minc = st; }
          stc = minc;
          dx = 0;
          dy = 0;

          //ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
          //    potential[stc], pathx[npath-1], pathy[npath-1]);

          if (potential[stc] >= POT_HIGH)
          {
            ROS_DEBUG("[PathCalc] No path found, high potential");
            //savemap("navfn_highpot");
            return 0;
          }
        }

        // have a good gradient here
        else			
        {

          // get grad at four positions near cell
          gradCell(potential, stc);
          gradCell(potential, stc+1);
          gradCell(potential, stcnx);
          gradCell(potential, stcnx+1);


          // get interpolated gradient
          float x1 = (1.0-dx)*gradx[stc] + dx*gradx[stc+1];
          float x2 = (1.0-dx)*gradx[stcnx] + dx*gradx[stcnx+1];
          float x = (1.0-dy)*x1 + dy*x2; // interpolated x
          float y1 = (1.0-dx)*grady[stc] + dx*grady[stc+1];
          float y2 = (1.0-dx)*grady[stcnx] + dx*grady[stcnx+1];
          float y = (1.0-dy)*y1 + dy*y2; // interpolated y

          // show gradients
          ROS_DEBUG("[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n",
                    gradx[stc], grady[stc], gradx[stc+1], grady[stc+1], 
                    gradx[stcnx], grady[stcnx], gradx[stcnx+1], grady[stcnx+1],
                    x, y);

          // check for zero gradient, failed
          if (x == 0.0 && y == 0.0)
          {
            ROS_DEBUG("[PathCalc] Zero gradient");	  
            return 0;
          }

          // move in the right direction
          float ss = pathStep/sqrt(x*x+y*y);
          dx += x*ss;
          dy += y*ss;

          // check for overflow
          if (dx > 1.0) { stc++; dx -= 1.0; }
          if (dx < -1.0) { stc--; dx += 1.0; }
          if (dy > 1.0) { stc+=xs_; dy -= 1.0; }
          if (dy < -1.0) { stc-=xs_; dy += 1.0; }

        }

    }
    
    
    
    return false;
}



/*
  int
    NavFn::calcPath(int n, int *st)
    {
     // set up start position at cell
      // st is always upper left corner for 4-point bilinear interpolation 
      if (st == NULL) st = start;
      int stc = st[1]*nx + st[0];

      // go for <n> cycles at most
      for (int i=0; i<n; i++)
      {
        


        //      ROS_INFO("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
        //	     potential[stc], x, y, pathx[npath-1], pathy[npath-1]);
      }

      //  return npath;			// out of cycles, return failure
      ROS_DEBUG("[PathCalc] No path found, path too long");
      //savemap("navfn_pathlong");
      return 0;			// out of cycles, return failure
    }
*/



  //
  // gradient calculations
  //

  // calculate gradient at a cell
  // positive value are to the right and down
  float				
    GradientPath::gradCell(float* potential, int n)
    {
      if (gradx[n]+grady[n] > 0.0)	// check this cell
        return 1.0;			

      if (n < xs_ || n > xs_*ys_-xs_)	// would be out of bounds
        return 0.0;

      float cv = potential[n];
      float dx = 0.0;
      float dy = 0.0;

      // check for in an obstacle
      if (cv >= POT_HIGH) 
      {
        if (potential[n-1] < POT_HIGH)
          dx = -COST_OBS;
        else if (potential[n+1] < POT_HIGH)
          dx = COST_OBS;

        if (potential[n-xs_] < POT_HIGH)
          dy = -COST_OBS;
        else if (potential[xs_+1] < POT_HIGH)
          dy = COST_OBS;
      }

      else				// not in an obstacle
      {
        // dx calc, average to sides
        if (potential[n-1] < POT_HIGH)
          dx += potential[n-1]- cv;	
        if (potential[n+1] < POT_HIGH)
          dx += cv - potential[n+1]; 

        // dy calc, average to sides
        if (potential[n-xs_] < POT_HIGH)
          dy += potential[n-xs_]- cv;	
        if (potential[n+xs_] < POT_HIGH)
          dy += cv - potential[n+xs_]; 
      }

      // normalize
      float norm = sqrtf(dx*dx+dy*dy);
      if (norm > 0)
      {
        norm = 1.0/norm;
        gradx[n] = norm*dx;
        grady[n] = norm*dy;
      }
      return norm;
    }




}; //end namespace global_planner

