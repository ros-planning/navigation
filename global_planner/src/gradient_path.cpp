/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <global_planner/gradient_path.h>
#include <algorithm>
#include <stdio.h>
#include <global_planner/planner_core.h>

namespace global_planner {

GradientPath::GradientPath(PotentialCalculator* p_calc) :
        Traceback(p_calc), pathStep_(0.5) {
    gradx_ = grady_ = NULL;
}

GradientPath::~GradientPath() {

    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
}

void GradientPath::setSize(int xs, int ys) {
    Traceback::setSize(xs, ys);
    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
    gradx_ = new float[xs * ys];
    grady_ = new float[xs * ys];
}

bool GradientPath::getPath(float* potential, double start_x, double start_y, double goal_x, double goal_y, std::vector<std::pair<float, float> >& path) {
    std::pair<float, float> current;
    int stc = getIndex(goal_x, goal_y);

    //not sure if this is advisable - maybe worth rounding to an int (after multiplying with to a precision)
    std::set<std::pair<float, float> > position_set; 
    // set up offset
    float dx = goal_x - (int)goal_x;
    float dy = goal_y - (int)goal_y;
    int ns = xs_ * ys_;
    memset(gradx_, 0, ns * sizeof(float));
    memset(grady_, 0, ns * sizeof(float));

    int neighbors[8]; 

    int c = 0;
    while (c++<ns*4) {
        // check if near goal
        double nx = stc % xs_ + dx, ny = stc / xs_ + dy;

	//this smoothens the path - but will map to the same index 

        if (fabs(nx - start_x) < .5 && fabs(ny - start_y) < .5) {
            current.first = start_x;
            current.second = start_y;
            path.push_back(current);
            return true;
        }

        if (stc < xs_ || stc > xs_ * ys_ - xs_) // would be out of bounds
        {
            printf("[PathCalc] Out of bounds\n");
            return false;
        }

        current.first = nx;
        current.second = ny;

        bool oscillation_detected = false;

	path.push_back(current);
	position_set.insert(current);

        int stcnx = stc + xs_;
        int stcpx = stc - xs_;

	//why is all 8 neighbours valid?? 
	//this has wrap around - maybe its the POT_HIGH?? 
	//check gradiant first 
	bool check_gradient = true; 
	bool zero_gradient = false; 

	// check for potentials at eight positions near cell
        if (potential[stc] >= POT_HIGH || potential[stc + 1] >= POT_HIGH || potential[stc - 1] >= POT_HIGH
	    || potential[stcnx] >= POT_HIGH || potential[stcnx + 1] >= POT_HIGH || potential[stcnx - 1] >= POT_HIGH
	    || potential[stcpx] >= POT_HIGH || potential[stcpx + 1] >= POT_HIGH || potential[stcpx - 1] >= POT_HIGH){
	  check_gradient = false; 
	}

	//set the neighbors 
	neighbors[0] = stcpx - 1; 
	neighbors[1] = stcpx; 
	neighbors[2] = stcpx + 1; 
	neighbors[3] = stc - 1;
	neighbors[4] = stc + 1;
	neighbors[5] = stcnx - 1; 
	neighbors[6] = stcnx; 
	neighbors[7] = stcnx+1; 

	if(check_gradient){
	    gradCell(potential, stc);
            gradCell(potential, stc + 1);
            gradCell(potential, stcnx);
            gradCell(potential, stcnx + 1);

            // get interpolated gradient
            float x1 = (1.0 - dx) * gradx_[stc] + dx * gradx_[stc + 1];
            float x2 = (1.0 - dx) * gradx_[stcnx] + dx * gradx_[stcnx + 1];
            float x = (1.0 - dy) * x1 + dy * x2; // interpolated x
            float y1 = (1.0 - dx) * grady_[stc] + dx * grady_[stc + 1];
            float y2 = (1.0 - dx) * grady_[stcnx] + dx * grady_[stcnx + 1];
            float y = (1.0 - dy) * y1 + dy * y2; // interpolated y

            // show gradients
            ROS_DEBUG("[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n", 
		     gradx_[stc], grady_[stc], gradx_[stc+1], grady_[stc+1], 
		     gradx_[stcnx], grady_[stcnx], gradx_[stcnx+1], grady_[stcnx+1], x, y);

            // check for zero gradient, failed
            if (x == 0.0 && y == 0.0) {
	      //hitting here 
	      ROS_ERROR("[PathCalc] Zero gradient - Distance from goal : %f, %f\n", 
			fabs(nx - start_x), fabs(ny - start_y));
	      zero_gradient = true; 
                //return false;		
            }

	    if(!zero_gradient){
	      // move in the right direction
	      float ss = pathStep_ / ::hypot(x, y);
	      dx += x * ss;
	      dy += y * ss;

	      //should we clamp this to some resolution?? - so that we can use the hash set? 

	      int stc_new = stc; 
	    
	      // check for overflow
	      if (dx > 1.0) {
                stc_new++;
                dx -= 1.0;
	      }
	      if (dx < -1.0) {
                stc_new--;
                dx += 1.0;
	      }
	      if (dy > 1.0) {
                stc_new += xs_;
                dy -= 1.0;
	      }
	      if (dy < -1.0) {
                stc_new -= xs_;
                dy += 1.0;
	      }

	      double nx_next = stc_new % xs_ + dx, ny_next = stc_new / xs_ + dy;
	      
	      int npath = path.size();

	      if (npath > 1 && path[npath - 2].first == nx_next 
		  && path[npath - 2].second == ny_next) {
		oscillation_detected = true;
	      }

	      if(!oscillation_detected){
		stc = stc_new; 
	      }
	    }
	}

        
	if( !check_gradient || oscillation_detected || zero_gradient) {
	  ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", potential[stc], (int) path.size());
            // check eight neighbors to find the lowest
            int minc = stc;
            float minp = 1e10;
	    
	    int npath = path.size();
	    int previous_1 = getIndex(path[npath - 2].first, path[npath - 2].second);
	    int previous = getIndex(path[npath - 1].first, path[npath - 1].second);

	    for(int k=0; k < 8; k++){
	      if(neighbors[k] == previous || neighbors[k] == previous_1){
		continue; 
	      }

	      std::pair<float, float> cp;
	      cp.first = neighbors[k] % xs_;
	      cp.second = neighbors[k] / xs_;

	      if(position_set.find(cp) != position_set.end()){
		continue; 
	      }

	      if(potential[neighbors[k]] < minp){
		minp = potential[neighbors[k]]; 
		minc = neighbors[k]; 
	      }
	    }
           
	    //just move to the neighbor with the lowest potential //we have to ignore cycles also 
	    if(minc == stc){
	      fprintf(stderr, "Grid motion didn't find valid neighbor - declaring failure\n");
	      return false; 
	    }

            stc = minc;
            dx = 0;
            dy = 0;

            //ROS_DEBUG("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
            //    potential[stc], path[npath-1].first, path[npath-1].second);

            if (potential[stc] >= POT_HIGH) {
                ROS_WARN("[PathCalc] No path found, high potential");
                //savemap("navfn_highpot");
                return 0;
            }
        }       
    }

    return false;
}

//
// gradient calculations
//
// calculate gradient at a cell
// positive value are to the right and down
float GradientPath::gradCell(float* potential, int n) {
    if (gradx_[n] + grady_[n] > 0.0)    // check this cell
        return 1.0;

    if (n < xs_ || n > xs_ * ys_ - xs_)    // would be out of bounds
        return 0.0;

    //why aren't we skipping the x=0 and x = max_xs??
    /*if(n % xs_ == 0 || n % xs_ == (xs_ - 1)){
      return 0.0; 
      }*/
    

    float cv = potential[n];
    float dx = 0.0;
    float dy = 0.0;

    // check for in an obstacle
    if (cv >= POT_HIGH) {
        if (potential[n - 1] < POT_HIGH)
            dx = -lethal_cost_;
        else if (potential[n + 1] < POT_HIGH)
            dx = lethal_cost_;

        if (potential[n - xs_] < POT_HIGH)
            dy = -lethal_cost_;
        else if (potential[xs_ + 1] < POT_HIGH)
            dy = lethal_cost_;
    }

    else                // not in an obstacle
    {
        // dx calc, average to sides
        if (potential[n - 1] < POT_HIGH)
            dx += potential[n - 1] - cv;
        if (potential[n + 1] < POT_HIGH)
            dx += cv - potential[n + 1];

        // dy calc, average to sides
        if (potential[n - xs_] < POT_HIGH)
            dy += potential[n - xs_] - cv;
        if (potential[n + xs_] < POT_HIGH)
            dy += cv - potential[n + xs_];
    }

    // normalize
    float norm = ::hypot(dx, dy);
    if (norm > 0) {
        norm = 1.0 / norm;
        gradx_[n] = norm * dx;
        grady_[n] = norm * dy;
    }
    return norm;
}

} //end namespace global_planner

