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
#ifndef COSTMAP_2D_AISLE_BIAS_INFLATION_LAYER_H_
#define COSTMAP_2D_AISLE_BIAS_INFLATION_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/InflationPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#include <costmap_2d/inflation_layer.h>

namespace costmap_2d
{

class AisleBiasInflationLayer : public Layer
{
public:
  AisleBiasInflationLayer();

  virtual ~AisleBiasInflationLayer()
  {
    deleteKernels();
    if (dsrv_)
    {
        delete dsrv_;
    }
    if (inflation_access_)
    {
      delete inflation_access_;
    }
  }

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual bool isDiscretized()
  {
    return true;
  }
  virtual void matchSize();

  virtual void reset() { onInitialize(); }

  /** @brief  Given a distance, compute a cost.
   * @param  obs_distance The distance from an obstacle in cells
   * @param  vor_distance
   * @return A cost value for the distance */
  inline unsigned char computeCost(double obs_distance, double vor_distance) const
  {
    unsigned char cost = 0;
    if (obs_distance == 0)
    {
      return LETHAL_OBSTACLE;
    }
  
    double euclidean_distance = obs_distance * resolution_;
    if (obs_distance * resolution_ <= inscribed_radius_)
    {
      return INSCRIBED_INFLATED_OBSTACLE;
    }

    if (vor_distance <= 0.001 && euclidean_distance < (lane_width_ + inscribed_radius_))
    {
      return 0;
    }
    
    if (euclidean_distance > max_effect_distance_)
    {
      euclidean_distance = max_effect_distance_;
    }


    double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));

    double cell_dist = euclidean_distance - inscribed_radius_;
    double cell_from_lane_dist = cell_dist - lane_width_;
    // double sigma = 1 / (2 * M_PI) * exp(weight_ * lane_dist);
    // double sub_factor = 1 / (2 * M_PI * sigma) 
    //                     * exp(-1 * (cell_from_lane_dist * cell_from_lane_dist) / (2 * sigma * sigma));
    double sigma = 0.5 * lane_width_;
    double sub_factor = exp(-1 * (cell_from_lane_dist * cell_from_lane_dist) / (2 * sigma * sigma));

      double full_factor = factor * (1 - sub_factor);
    cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * full_factor);
    if (std::fabs(cell_from_lane_dist) < resolution_ / 2) {
      cost = 0;
    }
    // ROS_INFO("inflation: %f, lane_width: %f, sigma: %f, cell_dist: %f, factor %f, sub_factor %f", 
    // inscribed_radius_, lane_width_, sigma, cell_dist, factor, sub_factor);


      // cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    return cost;
  }

  /**
   * @brief Change the values of the inflation radius parameters
   * @param inflation_radius The new inflation radius
   * @param cost_scaling_factor The new weight
   */
  void setInflationParameters(double inflation_radius, double cost_scaling_factor, double lane_width);

  virtual bool needsUpdate() {return need_reinflation_;};

  virtual std::shared_ptr<std::vector<double>> getDistancesFromStaticMap() {
    return obstacle_distance_map_;
  }

  virtual std::shared_ptr<std::vector<int>> getAnglesFromStaticMap() {
    return obstacle_angle_map_;
  }

protected:
  virtual void onFootprintChanged();
  boost::recursive_mutex* inflation_access_;

private:
  /**
   * @brief  Lookup pre-computed distances
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline double distanceLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    // return std::max(dx, dy);
    return cached_distances_[dx][dy];
  }

  /**
   * @brief  Lookup pre-computed costs
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline unsigned char costLookup(double obs_dist, double vor_dist)
  {
    return cached_costs_[obs_dist][vor_dist];
  }

  void computeCaches();
  void deleteKernels();
  void inflate_area(int min_i, int min_j, int max_i, int max_j, unsigned char* master_grid);

  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }

  inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                      unsigned int src_x, unsigned int src_y,
                      std::map<double, std::vector<CellData> >& queue_map);

  double inflation_radius_, inscribed_radius_, weight_;
  unsigned int cell_inflation_radius_;
  unsigned int cached_cell_inflation_radius_;

  std::map<double, std::map<double, unsigned char> > cached_costs_;


  double resolution_;

  std::shared_ptr<std::vector<double>> obstacle_distance_map_;
  std::shared_ptr<std::vector<int>> obstacle_angle_map_;

  double** cached_distances_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig> *dsrv_;
  void reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level);

  bool need_reinflation_;  ///< Indicates that the entire costmap should be reinflated next time around.

  double max_effect_distance_ = 1.0;
  double lane_width_ = 0.5;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_AISLE_BIAS_INFLATION_LAYER_H_
