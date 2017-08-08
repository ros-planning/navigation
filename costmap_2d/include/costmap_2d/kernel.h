/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2017, 6 River Systems, Inc.
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
 *         Daniel Grieneisen
 *********************************************************************/
#ifndef COSTMAP_2D_KERNEL_H_
#define COSTMAP_2D_KERNEL_H_

#include <ros/ros.h>
#include <vector>

namespace costmap_2d
{

/**
 * A kernel is the map of cost that should be placed at the obstruction location in the costmap.
 */
struct Kernel
{
public:
  Kernel() {};

  ~Kernel() {};

  Kernel& operator=(const Kernel& kern)
  {
    // check for self assignement
    if (this == &kern)
    {
      return *this;
    }

    size_x_ = kern.size_x_;
    size_y_ = kern.size_y_;
    resolution_ = kern.resolution_;
    radius_ = kern.radius_;

    values_ = kern.values_;
    ignore_freespace_ = kern.ignore_freespace_;

    return *this;
  };

  Kernel(const Kernel& kern)
  {
    *this = kern;
  };

  unsigned int size_x_ = 1; // size
  unsigned int size_y_ = 1; // size
  std::vector<unsigned char> values_; // square array that should get centered on obstruction
  float resolution_ = 1; // resolution of the costmap
  float radius_ = 1; // radius of affect of the costmap
  unsigned char max_cost_ = 255; // Maximum cost in the kernel
  bool ignore_freespace_ = false; // Whether the kernel should ignore freespace when being applied
};

/**
 * Factory class to create kernels.
 */
class KernelFactory
{
public:
  /**
   * Generates radial inflation cost.
   * @param max_value The maximum value (placed at the center of the kernel)
   * @param inscribed_value The value to use within the inscribed radius
   * @param inscribed_radius The radius of the largest inscribed circle within the robot's footprint
   * @param inflation_radius The max distance away from the kernel center for which there is cost
   * @param cost_scaling_factor The factor used in the exponential decay cost function
   * @param resolution The resolution of the grid
   * @param ignore_freespace Do not overwrite freespace with this kernel
   */
  static std::shared_ptr<Kernel> generateRadialInflationKernel(unsigned char max_value, unsigned char inscribed_value,
    double inscribed_radius, double inflation_radius, double cost_scaling_factor, double resolution,
    bool ignore_freespace)
  {

    auto kernel = std::make_shared<Kernel>();

    kernel->ignore_freespace_ = ignore_freespace;
    kernel->resolution_ = resolution;
    kernel->radius_ = inflation_radius;
    kernel->max_cost_ = max_value;

    unsigned int cell_inflation_radius = (unsigned int) std::max(0.0, ceil(inflation_radius / resolution));
    unsigned int size =  cell_inflation_radius * 2 + 1;
    kernel->size_x_ = size;
    kernel->size_y_ = size;
    int center_x = kernel->size_x_ / 2 + 1;
    int center_y = kernel->size_y_ / 2 + 1;

    ROS_DEBUG("Inflating: max_val %d, insc val %d, cell rad %d, size %d, center_x: %d, center_y: %d",
      max_value, inscribed_value, cell_inflation_radius, size, center_x, center_y);

    kernel->values_.resize(kernel->size_x_ * kernel->size_y_);

    for (int yy = 0; yy < kernel->size_y_; ++yy)
    {
      for (int xx = 0; xx < kernel->size_x_; ++xx)
      {
        double cell_distance = std::hypot(center_x - xx, center_y - yy);
        double real_distance = cell_distance * resolution;
        ROS_DEBUG("yy %d, xx %d, cell_dist %f", yy, xx, cell_distance);

        unsigned char cost = 0;
        if (cell_distance == 0)
        {
          cost = max_value;
        }
        else if (real_distance <= inscribed_radius)
        {
          cost = inscribed_value;
        }
        else if (inscribed_value > 0 && real_distance <= inflation_radius)
        {
          double factor = exp(-1.0 * cost_scaling_factor * (real_distance - inscribed_radius));
          cost = (unsigned char)((inscribed_value - 1) * factor);
          ROS_DEBUG("Inflating dist: %f, factor %f, cost %d", real_distance, factor, cost);
        }

        kernel->values_[kernel->size_x_ * yy + xx] = cost;
      }
    }
    return kernel;
  }

    /**
   * Generates trinomial radial inflation cost.
   * @param max_value The maximum value (placed at the center of the kernel)
   * @param inscribed_value The value to use within the inscribed radius
   * @param inscribed_radius The radius of the largest inscribed circle within the robot's footprint
   * @param inflation_radius The max distance away from the kernel center for which there is cost
   * @param cost_scaling_factor The factor used in the exponential decay cost function
   * @param resolution The resolution of the grid
   * @param ignore_freespace Do not overwrite freespace with this kernel
   */
  static std::shared_ptr<Kernel> generateTrinomialRadialInflationKernel(unsigned char max_value, unsigned char inscribed_value,
    double inscribed_radius, double inflation_radius, double cost_scaling_factor, double resolution,
    bool ignore_freespace)
  {

    auto kernel = std::make_shared<Kernel>();

    kernel->ignore_freespace_ = ignore_freespace;
    kernel->resolution_ = resolution;
    kernel->radius_ = inflation_radius;
    kernel->max_cost_ = max_value;

    unsigned int cell_inflation_radius = (unsigned int) std::max(0.0, ceil(inflation_radius / resolution));
    unsigned int size =  cell_inflation_radius * 2 + 1;
    kernel->size_x_ = size;
    kernel->size_y_ = size;
    int center_x = kernel->size_x_ / 2 + 1;
    int center_y = kernel->size_y_ / 2 + 1;

    ROS_DEBUG("Inflating: max_val %d, insc val %d, cell rad %d, size %d, center_x: %d, center_y: %d",
      max_value, inscribed_value, cell_inflation_radius, size, center_x, center_y);

    kernel->values_.resize(kernel->size_x_ * kernel->size_y_);

    for (int yy = 0; yy < kernel->size_y_; ++yy)
    {
      for (int xx = 0; xx < kernel->size_x_; ++xx)
      {
        double cell_distance = std::hypot(center_x - xx, center_y - yy);
        double real_distance = cell_distance * resolution;
        ROS_DEBUG("yy %d, xx %d, cell_dist %f", yy, xx, cell_distance);

        unsigned char cost = 0;
        if (cell_distance == 0)
        {
          cost = max_value;
        }
        else if (real_distance <= inscribed_radius)
        {
          cost = inscribed_value;
        }
        else if (inscribed_value > 0 && real_distance <= inflation_radius)
        {

          double dO = real_distance - inscribed_radius;
          double dV = inflation_radius - real_distance;
          double factor = (cost_scaling_factor / (cost_scaling_factor + dO))
            * (dV / (dO + dV))
            * pow((dO - inflation_radius)/inflation_radius, 2);

          cost = (unsigned char)((inscribed_value - 1) * factor);
          ROS_DEBUG("Inflating dist: %f, factor %f, cost %d", real_distance, factor, cost);
        }

        kernel->values_[kernel->size_x_ * yy + xx] = cost;
      }
    }
    return kernel;
  }
};

}

#endif // COSTMAP_2D_KERNEL_H_
