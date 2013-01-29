/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
#include <costmap_2d/layered_costmap.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>

using namespace std;

namespace costmap_2d {
  LayeredCostmap::LayeredCostmap(string global_frame, bool rolling_window) :
                    costmap_(), global_frame_(global_frame), rolling_window_(rolling_window)
                    {
  }

  LayeredCostmap::~LayeredCostmap() {
  }

  void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y) {
    boost::recursive_mutex::scoped_lock lock(lock_);

    costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
      for (vector<boost::shared_ptr<CostmapPlugin> >::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin) {
        (*plugin)->matchSize();
      }
  }

  void LayeredCostmap::updateMap(double origin_x, double origin_y, double origin_yaw) {
    boost::recursive_mutex::scoped_lock lock(lock_);
    
    // if we're using a rolling buffer costmap... we need to update the origin using the robot's position
    if (rolling_window_) { 
        double new_origin_x = origin_x - costmap_.getSizeInMetersX() / 2;
        double new_origin_y = origin_y - costmap_.getSizeInMetersY() / 2;
        costmap_.updateOrigin(new_origin_x, new_origin_y);
    }

    if (plugins_.size() == 0)
        return;

    minx_ = miny_ = 1e30;
    maxx_ = maxy_ = -1e30;

    for (vector<boost::shared_ptr<CostmapPlugin> >::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin) {
        (*plugin)->update_bounds(origin_x, origin_y, origin_yaw, &minx_, &miny_, &maxx_, &maxy_);
    }

    int x0, xn, y0, yn;
    costmap_.worldToMapNoBounds(minx_, miny_, x0, y0);
    costmap_.worldToMapNoBounds(maxx_, maxy_, xn, yn);

    x0 = std::max(0, x0);
    xn = std::min(int(costmap_.getSizeInCellsX()), xn+1);
    y0 = std::max(0, y0);
    yn = std::min(int(costmap_.getSizeInCellsY()), yn+1);
    
    costmap_.resetMap(x0,y0,xn,yn);

    for (vector<boost::shared_ptr<CostmapPlugin> >::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin) {
      (*plugin)->update_costs(costmap_, x0, y0, xn, yn);
    }
  }

  bool LayeredCostmap::isCurrent() {
    current_ = true;
    for (vector<boost::shared_ptr<CostmapPlugin> >::iterator plugin = plugins_.begin();
                                                        plugin != plugins_.end(); ++plugin) {
        current_ = current_ && (*plugin)->isCurrent();
    }
    return current_;
  }


};  // namespace layered_costmap
