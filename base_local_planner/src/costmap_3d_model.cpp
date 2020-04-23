/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, Badger Technologies LLC
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
* Author: C. Andy Martin
*********************************************************************/
#include <base_local_planner/costmap_3d_model.h>

namespace base_local_planner
{

Costmap3DModel::Costmap3DModel(costmap_3d::Costmap3DROS& costmap_3d)
  :  CostmapModel(*costmap_3d.getCostmap()),
     costmap_3d_(costmap_3d)
{
}

double Costmap3DModel::footprintCost(
    const geometry_msgs::Point& position,
    const std::vector<geometry_msgs::Point>& footprint,
    double inscribed_radius,
    double circumscribed_radius)
{
  // Be sure to warn once that this method was called, as it is unlikely to
  // be the method that most users will want. We are stuck with this method
  // due to being a WorldModel.
  ROS_WARN_ONCE("orientationless footprintCost() called, assuming unity orientation!");
  return footprintCost(position.x, position.y, 0.0, footprint, inscribed_radius, circumscribed_radius);
}

double Costmap3DModel::footprintCost(
    double x,
    double y,
    double theta,
    const std::vector<geometry_msgs::Point>& footprint_spec,
    double inscribed_radius,
    double circumscribed_radius)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  pose.orientation = tf2::toMsg(q);
  if ( costmap_3d_query_ )
  {
    // Use any direct query, if available
    return costmap_3d_query_->footprintCost(pose);
  }
  return costmap_3d_.footprintCost(pose);
}

}  // end namespace base_local_planner
