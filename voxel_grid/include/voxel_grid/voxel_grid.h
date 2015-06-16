/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#ifndef VOXEL_GRID_VOXEL_GRID_H
#define VOXEL_GRID_VOXEL_GRID_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>
#include <algorithm>
#include <ros/console.h>
#include <ros/assert.h>
#include <voxel_grid/abstract_grid_updater.h>

/**
 * @class VoxelGrid
 * @brief A 3D grid structure that stores points as an integer array.
 *        X and Y index the array and Z selects which bit of the integer
 *        is used giving a limit of 16 vertical cells.
 */
namespace voxel_grid
{

enum VoxelStatus {
  FREE = 0,
  UNKNOWN = 1,
  MARKED = 2,
};

class VoxelGrid
{
public:
  /**
   * @brief  Constructor for a voxel grid
   * @param size_x The x size of the grid
   * @param size_y The y size of the grid
   * @param size_z The z size of the grid, only sizes <= 16 are supported
   */
  VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z);

  ~VoxelGrid();

  /**
   * @brief  Resizes a voxel grid to the desired size
   * @param size_x The x size of the grid
   * @param size_y The y size of the grid
   * @param size_z The z size of the grid, only sizes <= 16 are supported
   */
  void resize(unsigned int size_x, unsigned int size_y, unsigned int size_z);

  void reset();
  uint32_t* getData() { return data_; }

  /**
   * @brief Sets the multiplier which determines accuracy
   * @param Number of bits which should be used for accuracy
   *
   * Number of bits should be less than:
   * (bits representing the max of an integer) minus
   * (bits needed to represent the maximum raytrace distance (= maximum sensor range) in cells)
   *
   * E. g.:
   * 32 Bit machine -> INT_MAX is 2147483647 = 2^31 => 31 bits (1 bit for the sign)
   * Max raytrace range is 3 meters, cell resolution is 0.01 meters -> 3 / 0.01 => 300 cells
   * 300 can be represented by 9 bits (300 <= 2^9)
   *
   * Number of maximum accuracy multiplier bits: 31 - 9 = 22
   *
   */
  void setAccuracyMultiplierBits(unsigned int number_of_bits)
  {
    accuracy_multiplier_ = pow(2, number_of_bits);
  };

  inline void markVoxel(unsigned int x, unsigned int y, unsigned int z)
  {
    if (x >= size_x_ || y >= size_y_ || z >= size_z_)
    {
      ROS_DEBUG("Error, voxel out of bounds.\n");
      return;
    }
    uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
    data_[y * size_x_ + x] |= full_mask; //clear unknown and mark cell
  }

  inline bool markVoxelInMap(unsigned int x, unsigned int y, unsigned int z, unsigned int marked_threshold)
  {
    if (x >= size_x_ || y >= size_y_ || z >= size_z_)
    {
      ROS_DEBUG("Error, voxel out of bounds.\n");
      return false;
    }

    int index = y * size_x_ + x;
    uint32_t* col = &data_[index];
    uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
    *col |= full_mask; //clear unknown and mark cell

    unsigned int marked_bits = *col>>16;

    //make sure the number of bits in each is below our thesholds
    return !bitsBelowThreshold(marked_bits, marked_threshold);
  }

  inline void clearVoxel(unsigned int x, unsigned int y, unsigned int z)
  {
    if (x >= size_x_ || y >= size_y_ || z >= size_z_)
    {
      ROS_DEBUG("Error, voxel out of bounds.\n");
      return;
    }
    uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
    data_[y * size_x_ + x] &= ~(full_mask); //clear unknown and clear cell
  }

  inline void clearVoxelColumn(unsigned int index)
  {
    ROS_ASSERT(index < size_x_ * size_y_);
    data_[index] = 0;
  }

  inline void clearVoxelInMap(unsigned int x, unsigned int y, unsigned int z)
  {
    if(x >= size_x_ || y >= size_y_ || z >= size_z_)
    {
      ROS_DEBUG("Error, voxel out of bounds.\n");
      return;
    }
    int index = y * size_x_ + x;
    uint32_t* col = &data_[index];
    uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
    *col &= ~(full_mask); //clear unknown and clear cell

    unsigned int unknown_bits = uint16_t(*col>>16) ^ uint16_t(*col);
    unsigned int marked_bits = *col>>16;

    //make sure the number of bits in each is below our thesholds
    if (bitsBelowThreshold(unknown_bits, 1) && bitsBelowThreshold(marked_bits, 1))
    {
      costmap[index] = 0;
    }
  }

  inline bool bitsBelowThreshold(unsigned int n, unsigned int bit_threshold)
  {
    unsigned int bit_count;
    for (bit_count = 0; n;)
    {
      ++bit_count;
      if (bit_count > bit_threshold)
      {
        return false;
      }
      n &= n - 1; //clear the least significant bit set
    }
    return true;
  }

  static inline unsigned int numBits(unsigned int n)
  {
    unsigned int bit_count;
    for (bit_count = 0; n; ++bit_count)
    {
      n &= n - 1; //clear the least significant bit set
    }
    return bit_count;
  }

  static VoxelStatus getVoxel(
    unsigned int x, unsigned int y, unsigned int z,
    unsigned int size_x, unsigned int size_y, unsigned int size_z, const uint32_t* data)
  {
    if (x >= size_x || y >= size_y || z >= size_z)
    {
      ROS_DEBUG("Error, voxel out of bounds. (%d, %d, %d)\n", x, y, z);
      return UNKNOWN;
    }
    uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
    uint32_t result = data[y * size_x + x] & full_mask;
    unsigned int bits = numBits(result);

    // known marked: 11 = 2 bits, unknown: 01 = 1 bit, known free: 00 = 0 bits
    if (bits < 2)
    {
      if (bits < 1)
      {
        return FREE;
      }
      return UNKNOWN;
    }
    return MARKED;
  }

  void markVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length = UINT_MAX);
  void clearVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length = UINT_MAX);
  void clearVoxelLineInMap(double x0, double y0, double z0, double x1, double y1, double z1, unsigned char *map_2d,
                           AbstractGridUpdater* clearer, unsigned int area_width,
                           unsigned int max_length = UINT_MAX, bool include_corner_cases = false);

  VoxelStatus getVoxel(unsigned int x, unsigned int y, unsigned int z);

  //Are there any obstacles at that (x, y) location in the grid?
  VoxelStatus getVoxelColumn(unsigned int x, unsigned int y,
                             unsigned int unknown_threshold = 0, unsigned int marked_threshold = 0);

  void printVoxelGrid();
  void printColumnGrid();
  unsigned int sizeX();
  unsigned int sizeY();
  unsigned int sizeZ();


  /**
   * @brief Raytrace a line with the integer Bresenham algorithm
   *
   * See also:
   * http://graphics.idav.ucdavis.edu/education/GraphicsNotes/CAGDNotes/Bresenhams-Algorithm.pdf
   *
  **/
  inline void raytraceLine(AbstractGridUpdater* clearer, double x0, double y0, double z0, double x1,
                           double y1, double z1, unsigned int width, unsigned int max_length = UINT_MAX, bool include_corner_cases = false)
  {
    double dx = x1 - x0;
    double dy = y1 - y0;
    double dz = z1 - z0;

    double abs_dx = fabs(accuracy_multiplier_ * dx);
    double abs_dy = fabs(accuracy_multiplier_ * dy);
    double abs_dz = fabs(accuracy_multiplier_ * dz);

    if (abs_dx > INT_MAX || abs_dy > INT_MAX || abs_dz > INT_MAX)
    {
      ROS_ERROR_THROTTLE(1.0,
                         "Voxel grid: Accuracy multiplier is set too high. Possible integer overflow while clearing.");
    }

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * width;
    int offset_dz = sign(dz);

    unsigned int z_mask = ((1 << 16) | 1) << (unsigned int)z0;
    if(include_corner_cases)
      z_mask <<= 1; //to prevent underflows, is undone later

    unsigned int offset = (unsigned int)y0 * width + (unsigned int)x0;

    GridOffset index_updater_xy(offset);
    ZOffset index_updater_z(z_mask);

    double temp = 0;
    double x_lost_rounding = dx > 0 ? 1.0 - fabs(modf(x0, &temp)) : fabs(modf(x0, &temp));
    double y_lost_rounding = dy > 0 ? 1.0 - fabs(modf(y0, &temp)) : fabs(modf(y0, &temp));
    double z_lost_rounding = dz > 0 ? 1.0 - fabs(modf(z0, &temp)) : fabs(modf(z0, &temp));

    double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
    double scale = std::min(1.0, max_length / dist);

    if (abs_dx >= max(abs_dy, abs_dz))
    {
      int error_y = (int)(abs_dy * x_lost_rounding - abs_dx * y_lost_rounding);
      int error_z = (int)(abs_dz * x_lost_rounding - abs_dx * z_lost_rounding);
      unsigned int number_of_steps = scale * abs(int(x0) - int(x1));

      if (include_corner_cases)
      {
        bresenham3DIncludingCorners(clearer, index_updater_xy, index_updater_xy, index_updater_z, abs_dx, abs_dy, abs_dz,
                                    error_y, error_z, offset_dx, offset_dy, offset_dz, offset, z_mask,
                                    number_of_steps);
        return;
      }

      bresenham3D(clearer, index_updater_xy, index_updater_xy, index_updater_z, abs_dx, abs_dy, abs_dz, error_y, error_z,
                  offset_dx, offset_dy, offset_dz, offset, z_mask, number_of_steps);
      return;
    }

    if (abs_dy >= abs_dz)
    {
      int error_x = (int)(abs_dx * y_lost_rounding - abs_dy * x_lost_rounding);
      int error_z = (int)(abs_dz * y_lost_rounding - abs_dy * z_lost_rounding);
      unsigned int number_of_steps = scale * abs(int(y0) - int(y1));

      if (include_corner_cases)
      {
        bresenham3DIncludingCorners(clearer, index_updater_xy, index_updater_xy, index_updater_z, abs_dy, abs_dx, abs_dz,
                                    error_x, error_z, offset_dy, offset_dx, offset_dz, offset, z_mask,
                                    number_of_steps);
        return;
      }

      bresenham3D(clearer, index_updater_xy, index_updater_xy, index_updater_z, abs_dy, abs_dx, abs_dz, error_x, error_z,
                  offset_dy, offset_dx, offset_dz, offset,  z_mask, number_of_steps);
      return;
    }

    int error_x = (int)(abs_dx * z_lost_rounding - abs_dz * x_lost_rounding);
    int error_y = (int)(abs_dy * z_lost_rounding - abs_dz * y_lost_rounding);
    unsigned int number_of_steps = scale * abs(int(z0) - int(z1));

    if (include_corner_cases)
    {
      bresenham3DIncludingCorners(clearer, index_updater_z, index_updater_xy, index_updater_xy, abs_dz, abs_dx, abs_dy,
                                  error_x, error_y, offset_dz, offset_dx, offset_dy, offset, z_mask,
                                  number_of_steps);
      return;
    }

    bresenham3D(clearer, index_updater_z, index_updater_xy, index_updater_xy, abs_dz, abs_dx, abs_dy, error_x, error_y,
                offset_dz, offset_dx, offset_dy, offset,  z_mask, number_of_steps);
  }

private:
  //the real work is done here... 3D bresenham implementation
  template<class OffA, class OffB, class OffC>
  inline void bresenham3D(AbstractGridUpdater* clearer, OffA off_a, OffB off_b, OffC off_c,
                          unsigned int abs_da, unsigned int abs_db, unsigned int abs_dc, int error_b, int error_c,
                          int offset_a, int offset_b, int offset_c, unsigned int &offset,
                          unsigned int &z_mask, unsigned int number_of_steps)
  {

    for (unsigned int i = 0; i < number_of_steps; ++i)
    {
      (*clearer)(offset, z_mask);
      off_a(offset_a);

      if (error_b >= 0)
      {
        off_b(offset_b);
        error_b -= abs_da;
      }

      if (error_c >= 0)
      {
        off_c(offset_c);
        error_c -= abs_da;
      }

      error_b += abs_db;
      error_c += abs_dc;
    }
  }

  /**
   * @brief Bresenham raytracing algorithm including corner cases.
   *
   * The raytracing on discrete cells. Includes corner cases.
   *
   * |   |   |   |   |
   * -----------------  Legend:
   * |   | # | = | = |          = : Cells added by original Bresenham
   * -----------------          # : Additional corner case cells which are added
   * | = | = | # |   |
   * -----------------
  **/
  template<class OffA, class OffB, class OffC>
  inline void bresenham3DIncludingCorners(AbstractGridUpdater* clearer, OffA off_a, OffB off_b,
                                          OffC off_c, unsigned int abs_da, unsigned int abs_db, unsigned int abs_dc,
                                          int error_b, int error_c, int offset_a, int offset_b, int offset_c,
                                          unsigned int &offset, unsigned int &z_mask,
                                          unsigned int number_of_steps)
  {

    //-1 because we don't want to clear the corners _after_ the last cell
    //last cell is cleared after the loop
    for (unsigned int i = 0; i < number_of_steps - 1; ++i)
    {
      (*clearer)(offset, z_mask);
      off_a(offset_a);

      if (error_b >= 0)
      {
        //adjusted not original Bresenham
        (*clearer)(offset, z_mask); //set voxel on the same row

        off_b(offset_b); //go one row up
        off_a(-offset_a); //go one column back
        (*clearer)(offset, z_mask); //set voxel

        off_a(offset_a); //go back to next column

        error_b -= abs_da;
      }

      if (error_c >= 0)
      {
        //adjusted not original Bresenham
        (*clearer)(offset, z_mask); //set voxel on the same row

        off_c(offset_c); //go one row up
        off_a(-offset_a); //go one column back
        (*clearer)(offset, z_mask); //set voxel

        off_a(offset_a); //go back to next column

        error_c -= abs_da;
      }

      error_b += abs_db;
      error_c += abs_dc;
    }

    //clearing of last cell because loop only goes to number_of_steps - 1
    off_a(offset_a);
  }

  inline int sign(int i)
  {
    return i >= 0 ? 1 : -1;
  }

  inline unsigned int max(unsigned int x, unsigned int y)
  {
    return x > y ? x : y;
  }

  unsigned int size_x_, size_y_, size_z_;
  uint32_t *data_;
  unsigned char *costmap;

  /**
   * @brief Used for Bresenham raytracing, increases accuracy
   *
   * Because we use integers to be fast we lose the floating point precision.
   * By multiplying our values before converting to integers we can get higher accuracy.
   * There is a maximum for this as we don't want to overflow our integers.
   *
   **/
  unsigned int accuracy_multiplier_;

  class GridOffset
  {
  public:
    GridOffset(unsigned int &offset) : offset_(offset) {}
    inline void operator()(int offset_val)
    {
      offset_ += offset_val;
    }
  private:
    unsigned int &offset_;
  };

  class ZOffset
  {
  public:
    ZOffset(unsigned int &z_mask) : z_mask_(z_mask) {}
    inline void operator()(int offset_val)
    {
      offset_val > 0 ? z_mask_ <<= 1 : z_mask_ >>= 1;
    }
  private:
    unsigned int & z_mask_;
  };
};

}  // namespace voxel_grid

#endif  // VOXEL_GRID_VOXEL_GRID_H
