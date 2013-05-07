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
#ifndef COSTMAP_PLUGIN_ROS_H_
#define COSTMAP_PLUGIN_ROS_H_
#include <costmap_2d/plugin_base.h>
#include <geometry_msgs/Polygon.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace costmap_2d
{
class CostmapPluginROS : public CostmapPlugin
{
public:
  void initialize(LayeredCostmap* costmap, std::string name, tf::TransformListener &tf)
  {
    tf_ = &tf;
    initialize(costmap, name);
  }
  virtual void initialize(LayeredCostmap* costmap, std::string name)= 0;

  void setFootprint(const geometry_msgs::Polygon& footprint_spec)
  {
    // TODO: Ideally this would actually check if the footprint had changed or not.
    footprint_spec_ = footprint_spec;
    onFootprintChanged();
  }
  const geometry_msgs::Polygon& getFootprint() const { return footprint_spec_; }

protected:
  CostmapPluginROS()
  {
  }

  /** @brief This is called at the end of setFootprint().  Override to be notified of changes in the robot's footprint. */
  virtual void onFootprintChanged() {}

  tf::TransformListener* tf_;

private:
  geometry_msgs::Polygon footprint_spec_;
};
}  // namespace layered_costmap
#endif
