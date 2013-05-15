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

#include<layered_costmap_plugin/layered_costmap_plugin.h>
#include<costmap_2d/costmap_math.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(layered_costmap_plugin::LayeredCostmapPlugin, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;

namespace layered_costmap_plugin
{
    void LayeredCostmapPlugin::initialize(costmap_2d::LayeredCostmap* costmap, std::string name)
    {
        ros::NodeHandle nh("~/" + name), g_nh;
        layered_costmap_ = costmap;
        name_ = name;
        current_ = true;

        global_frame_ = costmap->getGlobalFrameID();
        
        sub_layered_costmap_ = new costmap_2d::LayeredCostmap(global_frame_, costmap->isRolling(), costmap->getCostmap()->getDefaultValue());
        
        if (nh.hasParam("plugins")) {
            XmlRpc::XmlRpcValue my_list;
            nh.getParam("plugins", my_list);
            for (int32_t i = 0; i < my_list.size(); ++i) {
                std::string pname = static_cast<std::string>(my_list[i]["name"]);
                std::string type = static_cast<std::string>(my_list[i]["type"]);
                ROS_INFO("Using plugin \"%s\"", pname.c_str());

                boost::shared_ptr<CostmapPluginROS> plugin = plugin_loader_.createInstance(type);
                sub_layered_costmap_->addPlugin(plugin);
                plugin->initialize(sub_layered_costmap_, std::string(name + "/" + pname), *tf_);
            }
        } else {
            ROS_INFO("No plugins");
        }
        
        
        matchSize();

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&LayeredCostmapPlugin::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }
    
    LayeredCostmapPlugin::~LayeredCostmapPlugin(){
        delete sub_layered_costmap_;
    }
    
    void LayeredCostmapPlugin::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
        if(config.enabled != enabled_){
            enabled_ = config.enabled;
        }
    }

    void LayeredCostmapPlugin::matchSize(){
        costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
        sub_layered_costmap_->resizeMap(
                  master->getSizeInCellsX(), master->getSizeInCellsY(),
                  master->getResolution(),
                  master->getOriginX(), master->getOriginY(), true);
    }

    void LayeredCostmapPlugin::updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y){
        if(!enabled_) return;
        
        sub_layered_costmap_->updateMap(origin_x, origin_y, origin_z);
        sub_layered_costmap_->getUpdatedBounds(*min_x, *min_y, *max_x, *max_y);
    }
    
    void LayeredCostmapPlugin::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        if(!enabled_) return; 
        costmap_2d::Costmap2D* cmap = sub_layered_costmap_->getCostmap();
        unsigned char* master = master_grid.getCharMap();
        unsigned char* composite = cmap->getCharMap();
        for(int j=min_j; j<max_j; j++){
            for(int i=min_i; i<max_i; i++){
                int index = cmap->getIndex(i, j);
                unsigned char old_value = master[index];
                unsigned char new_value = composite[index];
                
                if(old_value == NO_INFORMATION)
                    master[index] = new_value;
                else if(new_value == NO_INFORMATION)
                    continue;
                else if(old_value < new_value)
                    master[index] = new_value;
            }
        }
    }
    
    void LayeredCostmapPlugin::activate(){
        std::vector<boost::shared_ptr<costmap_2d::CostmapPlugin> >* plugins = sub_layered_costmap_->getPlugins();
        // unsubscribe from topics
        for (std::vector<boost::shared_ptr<costmap_2d::CostmapPlugin> >::iterator plugin = plugins->begin(); plugin != plugins->end();
                ++plugin) {
            (*plugin)->activate();
        }
    
    }
    
    void LayeredCostmapPlugin::deactivate(){
        std::vector<boost::shared_ptr<costmap_2d::CostmapPlugin> >* plugins = sub_layered_costmap_->getPlugins();
        // unsubscribe from topics
        for (std::vector<boost::shared_ptr<costmap_2d::CostmapPlugin> >::iterator plugin = plugins->begin(); plugin != plugins->end();
                ++plugin) {
            (*plugin)->deactivate();
        }
    }



}
