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
 * Author: David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_PLUGIN_BASE_H_
#define COSTMAP_PLUGIN_BASE_H_
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <string>

namespace costmap_2d
{
class LayeredCostmap;
class CostmapPlugin
{
public:
  virtual void initialize(LayeredCostmap* costmap, std::string name) = 0;
  virtual void update_bounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y,
                             double* max_x, double* max_y) = 0;
  virtual void update_costs(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) = 0;

  virtual void deactivate() = 0;   // stop publishers
  virtual void activate() = 0;     // restart publishers if they've been stopped
  virtual ~CostmapPlugin()
  {
  }

  bool isCurrent()
  {
    return current_;
  }
  virtual void matchSize()
  {
  }  // matches the size of the parent costmap
  std::string getName()
  {
    return name_;
  }

protected:
  CostmapPlugin()
  {
  }
  LayeredCostmap* layered_costmap_;
  bool current_;
  bool enabled_;
  std::string name_;
};
} // namespace layered_costmap
#endif
