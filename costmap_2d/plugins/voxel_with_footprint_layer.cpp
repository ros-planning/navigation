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

#include <costmap_2d/voxel_with_footprint_layer.h>

namespace costmap_2d
{

void VoxelWithFootprintLayer::onInitialize()
{
  VoxelLayer::onInitialize();
  footprint_layer_.initialize( layered_costmap_, name_ + "_footprint", tf_);
}

void VoxelWithFootprintLayer::update_bounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                                    double* min_y, double* max_x, double* max_y)
{
  VoxelLayer::update_bounds(origin_x, origin_y, origin_yaw, min_x, min_y, max_x, max_y);
  footprint_layer_.update_bounds(origin_x, origin_y, origin_yaw, min_x, min_y, max_x, max_y);
}

void VoxelWithFootprintLayer::update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                                   int max_j)
{
  footprint_layer_.update_costs(*this, min_i, min_j, max_i, max_j);
  VoxelLayer::update_costs(master_grid, min_i, min_j, max_i, max_j);
}

void VoxelWithFootprintLayer::onFootprintChanged()
{
  footprint_layer_.setFootprint( getFootprint() );
}

} // end namespace costmap_2d

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( costmap_2d::VoxelWithFootprintLayer, costmap_2d::Layer )
