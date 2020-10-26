/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*********************************************************************/
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <vector>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(clear_costmap_recovery::ClearCostmapRecovery, nav_core::RecoveryBehavior)

using costmap_2d::NO_INFORMATION;

namespace clear_costmap_recovery {

/**
 * @brief return the names of the layers in the costmap, each prefixed with prefix
 */
std::vector<std::string> getLayerNames(const costmap_2d::Costmap2DROS& costmap)
{
  std::vector<std::string> names;
  std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap.getLayeredCostmap()->getPlugins();
  for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::const_iterator pluginp = plugins->begin();
       pluginp != plugins->end();
       ++pluginp) {
    names.push_back((*pluginp)->getName());
  }
  return names;
}

/**
 * @brief A layer is considered "matching" if the raw name (after the slash) is in the name_specs set
 */
bool isMatchingLayerName(const std::string& layer_name, const std::set<std::string>& name_specs)
{
  size_t slash = layer_name.rfind('/');
  if(slash != std::string::npos)
  {
    // If no slash, use whole string
    return name_specs.count(layer_name) != 0;
  }
  else
  {
    // Check the part after the slash
    return name_specs.count(layer_name.substr(slash + 1)) != 0;
  }
}

/**
 * @brief Returns the subset of layer_names which match the name spec
 */
std::vector<std::string> getLayerMatches(const std::vector<std::string>& layer_names,
                                         const std::set<std::string>& name_specs)
{
  std::vector<std::string> matches;
  for (unsigned int i = 0; i < layer_names.size(); i++)
  {
    if (isMatchingLayerName(layer_names[i], name_specs))
    {
      matches.push_back(layer_names[i]);
    }
  }
  return matches;
}


ClearCostmapRecovery::ClearCostmapRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  tf_(NULL), initialized_(false) {}

void ClearCostmapRecovery::initialize(std::string name, tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);

    private_nh.param("reset_distance", reset_distance_, 3.0);
    private_nh.param("invert_area_to_clear", invert_area_to_clear_, false);
    private_nh.param("force_updating", force_updating_, false);
    private_nh.param("affected_maps", affected_maps_, std::string("both"));
    if (affected_maps_ != "local" && affected_maps_ != "global" && affected_maps_ != "both")
    {
      ROS_WARN("Wrong value for affected_maps parameter: '%s'; valid values are 'local', 'global' or 'both'; " \
               "defaulting to 'both'", affected_maps_.c_str());
      affected_maps_ = "both";
    }

    // Get a list of layer names
    std::vector<std::string> layer_names = getLayerNames(*global_costmap_);
    std::vector<std::string> local_layer_names = getLayerNames(*local_costmap_);
    layer_names.insert(layer_names.end(), local_layer_names.begin(), local_layer_names.end());

    std::vector<std::string> clearable_layers_vector;
    if (private_nh.getParam("layer_names", clearable_layers_vector))
    {
      clearable_layers_ = std::set<std::string>(clearable_layers_vector.begin(), clearable_layers_vector.end());
    }
    else
    {
      // If the parameter is not specified, try two different defaults
      std::vector<std::string> possible_defaults {"obstacles", "obstacle_layer"};
      for (unsigned int i = 0; i < possible_defaults.size(); i++)
      {
        std::set<std::string> default_set;
        default_set.insert(possible_defaults[i]);
        if (getLayerMatches(layer_names, default_set).size() > 0)
        {
          clearable_layers_ = default_set;
          break;
        }
      }
    }

    std::vector<std::string> clearable_layer_names = getLayerMatches(layer_names, clearable_layers_);
    if (clearable_layer_names.empty())
    {
      ROS_ERROR("In recovery behavior %s, none of the layer names match the layer_names parameter.", name.c_str());
      ROS_ERROR("Layer names:");
      for (unsigned int i = 0; i < layer_names.size(); i++)
      {
        ROS_ERROR("\t%s", layer_names[i].c_str());
      }
      ROS_ERROR("Value of %s:", private_nh.resolveName("layer_names").c_str());
      for (std::set<std::string>::const_iterator it = clearable_layers_.begin();
           it != clearable_layers_.end();
           ++it) {
        ROS_ERROR("\t%s", it->c_str());
      }
    }
    else
    {
      ROS_INFO("Recovery behavior %s will clear layers:", name.c_str());
      for(unsigned int i = 0; i < clearable_layer_names.size(); i++)
      {
        ROS_INFO("\t%s", clearable_layer_names[i].c_str());
      }
    }

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void ClearCostmapRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
    return;
  }

  if (!invert_area_to_clear_){
    ROS_WARN("Clearing %s costmap%s outside a square (%.2fm) large centered on the robot.", affected_maps_.c_str(),
           affected_maps_ == "both" ? "s" : "", reset_distance_);
  }else {
    ROS_WARN("Clearing %s costmap%s inside a square (%.2fm) large centered on the robot.", affected_maps_.c_str(),
           affected_maps_ == "both" ? "s" : "", reset_distance_);
  }

  ros::WallTime t0 = ros::WallTime::now();
  if (affected_maps_ == "global" || affected_maps_ == "both")
  {
    clear(global_costmap_);

    if (force_updating_)
      global_costmap_->updateMap();

    ROS_DEBUG("Global costmap cleared in %fs", (ros::WallTime::now() - t0).toSec());
  }

  t0 = ros::WallTime::now();
  if (affected_maps_ == "local" || affected_maps_ == "both")
  {
    clear(local_costmap_);

    if (force_updating_)
      local_costmap_->updateMap();

    ROS_DEBUG("Local costmap cleared in %fs", (ros::WallTime::now() - t0).toSec());
  }
}

void ClearCostmapRecovery::clear(costmap_2d::Costmap2DROS* costmap){
  std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();

  geometry_msgs::PoseStamped pose;

  if(!costmap->getRobotPose(pose)){
    ROS_ERROR("Cannot clear map because pose cannot be retrieved");
    return;
  }

  double x = pose.pose.position.x;
  double y = pose.pose.position.y;

  for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
    boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
    if (isMatchingLayerName(plugin->getName(), clearable_layers_))
    {
      boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
      costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
      clearMap(costmap, x, y);
    }
  }
}


void ClearCostmapRecovery::clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap,
                                        double pose_x, double pose_y){
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  double start_point_x = pose_x - reset_distance_ / 2;
  double start_point_y = pose_y - reset_distance_ / 2;
  double end_point_x = start_point_x + reset_distance_;
  double end_point_y = start_point_y + reset_distance_;

  int start_x, start_y, end_x, end_y;
  costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

  costmap->clearArea(start_x, start_y, end_x, end_y, invert_area_to_clear_);

  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
  return;
}

};
