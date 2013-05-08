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
#ifndef LAYERED_COSTMAP_H_
#define LAYERED_COSTMAP_H_

#include <costmap_2d/cost_values.h>
#include <costmap_2d/plugin_base.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>
#include <pluginlib/class_loader.h>
#include <boost/thread.hpp>
#include <vector>
#include <string>

namespace costmap_2d
{
class CostmapPlugin;

/**
 * @class LayeredCostmap
 * @brief Instantiates different layer plugins and aggregates them into one score
 */
class LayeredCostmap
{
public:
  /**
   * @brief  Constructor for a costmap
   */
  LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);

  /**
   * @brief  Destructor
   */
  ~LayeredCostmap();

  /**
   * @brief  Update the underlying costmap with new data.
   * If you want to update the map outside of the update loop that runs, you can call this.
   */
  void updateMap(double origin_x, double origin_y, double origin_yaw);

  std::string getGlobalFrameID() const
  {
    return global_frame_;
  }

  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y,
                 bool size_locked = false);

  void getUpdatedBounds(double& minx, double& miny, double& maxx, double& maxy)
  {
    minx = minx_;
    miny = miny_;
    maxx = maxx_;
    maxy = maxy_;
  }

  bool isCurrent();

  Costmap2D* getCostmap()
  {
    return &costmap_;
  }

  bool isRolling()
  {
    return rolling_window_;
  }

  std::vector<boost::shared_ptr<CostmapPlugin> >* getPlugins()
  {
    return &plugins_;
  }

  void addPlugin(boost::shared_ptr<CostmapPlugin> plugin)
  {
    plugins_.push_back(plugin);
  }

  bool isSizeLocked()
  {
    return size_locked_;
  }

  void getBounds(unsigned int* x0, unsigned int* xn, unsigned int* y0, unsigned int* yn)
  {
    *x0 = bx0_;
    *xn = bxn_;
    *y0 = by0_;
    *yn = byn_;
  }

private:
  void updateUsingPlugins(std::vector<boost::shared_ptr<CostmapPlugin> > &plugins);

  Costmap2D costmap_;
  std::string global_frame_;

  bool rolling_window_;  /// < @brief Whether or not the costmap should roll with the robot

  bool current_;
  double minx_, miny_, maxx_, maxy_;
  unsigned int bx0_, bxn_, by0_, byn_;

  std::vector<boost::shared_ptr<CostmapPlugin> > plugins_;

  bool size_locked_;
};
}
;
// namespace layered_costmap

#endif
