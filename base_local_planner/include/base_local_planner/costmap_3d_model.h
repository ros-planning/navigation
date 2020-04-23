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
#ifndef TRAJECTORY_ROLLOUT_COSTMAP_3D_MODEL_H_
#define TRAJECTORY_ROLLOUT_COSTMAP_3D_MODEL_H_

#include <base_local_planner/costmap_model.h>
// To enable querying costmap 3D
#include <costmap_3d/costmap_3d_ros.h>

namespace base_local_planner
{

/**
 * @class Costmap3DModel
 * @brief A class that implements the CostmapModel interface to provide
 * collision checks for the trajectory controller using the Costmap3DROS.
 */
class Costmap3DModel : public CostmapModel
{
 public:
  /**
   * @brief Constructor for the Costmap3DModel
   * @param costmap The Costmap3DROS to use
   */
  Costmap3DModel(costmap_3d::Costmap3DROS& costmap_3d);

  /**
   * @brief Destructor for the world model
   */
  virtual ~Costmap3DModel() {}

  /**
   * @brief Set the query object to use
   * @param costmap_3d_query The Costmap3DQuery to use
   */
  void setQuery(costmap_3d::Costmap3DQueryPtr costmap_3d_query) { costmap_3d_query_ = costmap_3d_query; }

  /**
   * @brief Checks if any obstacles in the 3D costmap lie inside the default 3D footprint
   *
   * The caller must have the costmap 3D associated with this object locked
   * when making this call.
   *
   * Since there is no way to pass a 3D footprint model in via this method,
   * it uses the default model at unity orientation.
   *
   * It makes little sense to use this version of this method, unless all the
   * poses to query have unity orientation.
   *
   * @param  position The position of the robot in world coordinates
   * @param  footprint Ignored
   * @param  inscribed_radius Ignored
   * @param  circumscribed_radius Ignored
   * @return Negative if any obstacle lies within the default 3D footprint
   */
  virtual double footprintCost(
      const geometry_msgs::Point& position,
      const std::vector<geometry_msgs::Point>& footprint,
      double inscribed_radius,
      double circumscribed_radius);

  /**
   * @brief Checks if any obstacles in the 3D costmap lie inside the default 3D footprint
   *
   * The caller must have the costmap 3D associated with this object locked
   * when making this call.
   *
   * Since there is no way to pass a 3D footprint model in via this method,
   * it uses the default footprint model stored in the Costmap3DROS.
   *
   * @param  x The x position of the robot in world coordinates
   * @param  y The y position of the robot in world coordinates
   * @param  theta The theta orientation of the robot in world coordinates
   * @param  footprint_spec Ignored
   * @param  inscribed_radius Ignored
   * @param  circumscribed_radius Ignored
   * @return Negative if any obstacle lies within the default 3D footprint
   */
  virtual double footprintCost(
      double x,
      double y,
      double theta,
      const std::vector<geometry_msgs::Point>& footprint_spec,
      double inscribed_radius=0.0,
      double circumscribed_radius=0.0);

 protected:
  costmap_3d::Costmap3DROS& costmap_3d_; ///< @brief To query 3D costmap
  costmap_3d::Costmap3DQueryPtr costmap_3d_query_; ///< @brief Alternate interface, query direct
};

}  // namespace base_local_planner

#endif  // TRAJECTORY_ROLLOUT_COSTMAP_3D_MODEL_H_
