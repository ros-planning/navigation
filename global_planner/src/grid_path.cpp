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
#include <global_planner/grid_path.h>
#include <algorithm>
#include <stdio.h>
#include <set>

namespace global_planner {

bool GridPath::getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path) {
    std::pair<float, float> current;
    std::set<int> path_set; //this is used to ensure we don't get stuck in a loop (as it seems possible to do) 
    current.first = end_x;
    current.second = end_y;

    int start_index = getIndex(start_x, start_y);
    path_set.insert(getIndex(end_x, end_y));
    path.push_back(current);
    int c = 0;
    int ns = xs_ * ys_;
    
    while (getIndex(current.first, current.second) != start_index) {
        float min_val = 1e10;
        int min_x = 0, min_y = 0;
	int min_ind = -1; 
	//check the neighbors to find the node with the lowest cost 
        for (int xd = -1; xd <= 1; xd++) {
            for (int yd = -1; yd <= 1; yd++) {
                if (xd == 0 && yd == 0)
                    continue;
		
		int x = current.first + xd, y = current.second + yd;
		
		//make sure we don't connect because of wrap around 
		if(x < 0 || x > xs_ -1 || y > ys_ -1 || y < 0){
		  continue; 
		}

                int index = getIndex(x, y);
		if(path_set.find(index) != path_set.end()){
		  continue;
		}
                if (potential[index] < min_val) {
                    min_val = potential[index];
                    min_x = x;
                    min_y = y;
		    min_ind = index;
                }
            }
        }
        if (min_x == 0 && min_y == 0){
	  return false;
	}

        current.first = min_x;
        current.second = min_y;
        path.push_back(current);
        
        if(c++>ns*4){
            return false;
        }

	if(min_ind >= 0){
	  if(path_set.find(min_ind) != path_set.end()){
	    return false;
	  }
	  path_set.insert(min_ind);
	}

        
        if(c++>ns*4){
            return false;
        }

    }
    return true;
}

} //end namespace global_planner

