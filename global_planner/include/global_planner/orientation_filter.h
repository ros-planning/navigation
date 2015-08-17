/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, David V. Lu!!
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
 *   * Neither the name of David V. Lu nor the names of its
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
 * Author: David V. Lu!!
 *********************************************************************/
#ifndef GLOBAL_PLANNER_ORIENTATION_FILTER_H
#define GLOBAL_PLANNER_ORIENTATION_FILTER_H
#include <nav_msgs/Path.h>

namespace global_planner {

enum OrientationMode { NONE, FORWARD, INTERPOLATE, FORWARDTHENINTERPOLATE };

class OrientationFilter {
    public:
        OrientationFilter() : omode_(NONE) {}
    
    
        virtual void processPath(const geometry_msgs::PoseStamped& start,
                                 std::vector<geometry_msgs::PoseStamped>& path);
                                 
        void pointToNext(std::vector<geometry_msgs::PoseStamped>& path, int index);
        void interpolate(std::vector<geometry_msgs::PoseStamped>& path, 
                         int start_index, int end_index);
                         
        void setMode(OrientationMode new_mode){ omode_ = new_mode; }
        void setMode(int new_mode){ setMode((OrientationMode) new_mode); }
    protected:
        OrientationMode omode_;        
};

} //end namespace global_planner
#endif
