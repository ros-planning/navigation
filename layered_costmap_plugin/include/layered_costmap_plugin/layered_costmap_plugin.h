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
#ifndef LAYERED_COSTMAP_PLUGIN_H_
#define LAYERED_COSTMAP_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

namespace layered_costmap_plugin
{
class LayeredCostmapPlugin : public costmap_2d::Layer
{
    public:
        LayeredCostmapPlugin(): plugin_loader_("costmap_2d", "costmap_2d::CostmapPluginROS")
        {
            layered_costmap_ = NULL;
            sub_layered_costmap_ = NULL;
        }
        
        ~LayeredCostmapPlugin();

        void initialize(costmap_2d::LayeredCostmap* costmap, std::string name);
        void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y,
                           double* max_x, double* max_y);
        void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

        void activate();

        void deactivate();

        bool isDiscretized()
        {
            return true;
        }

        void matchSize();

    private:
        void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

        std::string global_frame_; ///< @brief The global frame for the costmap

        mutable boost::recursive_mutex lock_;
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
        
        costmap_2d::LayeredCostmap* sub_layered_costmap_;
        pluginlib::ClassLoader<CostmapPluginROS> plugin_loader_;
};
}
#endif

