/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef VOXEL_WITH_FOOTPRINT_COSTMAP_PLUGIN_H_
#define VOXEL_WITH_FOOTPRINT_COSTMAP_PLUGIN_H_

#include <costmap_2d/voxel_costmap_plugin.h>
#include <costmap_2d/footprint_costmap_plugin.h>
#include <dynamic_reconfigure/server.h>

namespace common_costmap_plugins
{

class VoxelWithFootprintCostmapPlugin : public common_costmap_plugins::VoxelCostmapPlugin
{
public:
  virtual void initialize(costmap_2d::LayeredCostmap* costmap, std::string name);
  virtual void update_bounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y,
                             double* max_x, double* max_y);
  virtual void update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  FootprintCostmapPlugin footprint_layer_;
};

} // end namespace common_costmap_plugins

#endif // VOXEL_WITH_FOOTPRINT_COSTMAP_PLUGIN_H_
