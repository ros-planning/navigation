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
#include <costmap_2d/costmap_2d.h>

#include <ros/console.h>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <cmath>
#include <iostream>

using namespace std;

namespace costmap_2d
{
Costmap2D::Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
                     double origin_x, double origin_y, unsigned char default_value) :
    size_x_(cells_size_x), size_y_(cells_size_y), resolution_(resolution), origin_x_(origin_x),
    origin_y_(origin_y), costmap_(NULL), default_value_(default_value)
{
  access_ = new mutex_t();

  // create the costmap
  initMaps(size_x_, size_y_);
  resetMaps();
}

void Costmap2D::deleteMaps()
{
  // clean up data
  boost::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  costmap_ = NULL;
}

void Costmap2D::initMaps(unsigned int size_x, unsigned int size_y)
{
  boost::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  costmap_ = new unsigned char[size_x * size_y];
}

void Costmap2D::resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                          double origin_x, double origin_y)
{
  size_x_ = size_x;
  size_y_ = size_y;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;

  initMaps(size_x, size_y);

  // reset our maps to have no information
  resetMaps();
}

void Costmap2D::resetMaps()
{
  boost::unique_lock<mutex_t> lock(*access_);
  memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

void Costmap2D::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
  boost::unique_lock<mutex_t> lock(*(access_));
  unsigned int len = xn - x0;
  for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_)
    memset(costmap_ + y, default_value_, len * sizeof(unsigned char));
}

bool Costmap2D::copyCostmapWindow(const Costmap2D& map, double win_origin_x, double win_origin_y, double win_size_x,
                                  double win_size_y)
{
  // check for self windowing
  if (this == &map)
  {
    // ROS_ERROR("Cannot convert this costmap into a window of itself");
    return false;
  }

  // clean up old data
  deleteMaps();

  // compute the bounds of our new map
  unsigned int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  if (!map.worldToMap(win_origin_x, win_origin_y, lower_left_x, lower_left_y)
      || !map.worldToMap(win_origin_x + win_size_x, win_origin_y + win_size_y, upper_right_x, upper_right_y))
  {
    // ROS_ERROR("Cannot window a map that the window bounds don't fit inside of");
    return false;
  }

  size_x_ = upper_right_x - lower_left_x;
  size_y_ = upper_right_y - lower_left_y;
  resolution_ = map.resolution_;
  origin_x_ = win_origin_x;
  origin_y_ = win_origin_y;

  // initialize our various maps and reset markers for inflation
  initMaps(size_x_, size_y_);

  // copy the window of the static map and the costmap that we're taking
  copyMapRegion(map.costmap_, lower_left_x, lower_left_y, map.size_x_, costmap_, 0, 0, size_x_, size_x_, size_y_);
  return true;
}

Costmap2D& Costmap2D::operator=(const Costmap2D& map)
{
  // check for self assignement
  if (this == &map)
    return *this;

  // clean up old data
  deleteMaps();

  size_x_ = map.size_x_;
  size_y_ = map.size_y_;
  resolution_ = map.resolution_;
  origin_x_ = map.origin_x_;
  origin_y_ = map.origin_y_;
  default_value_ = map.default_value_;

  // initialize our various maps
  initMaps(size_x_, size_y_);

  // copy the cost map
  memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));

  return *this;
}

Costmap2D::Costmap2D(const Costmap2D& map) :
    costmap_(NULL)
{
  access_ = new mutex_t();
  *this = map;
}

// just initialize everything to NULL by default
Costmap2D::Costmap2D() :
    size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL)
{
  access_ = new mutex_t();
}

Costmap2D::~Costmap2D()
{
  deleteMaps();
  delete access_;
}

unsigned int Costmap2D::cellDistance(double world_dist)
{
  double cells_dist = max(0.0, ceil(world_dist / resolution_));
  return (unsigned int)cells_dist;
}

unsigned char* Costmap2D::getCharMap() const
{
  return costmap_;
}

unsigned char Costmap2D::getCost(unsigned int mx, unsigned int my) const
{
  return costmap_[getIndex(mx, my)];
}

void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
  costmap_[getIndex(mx, my)] = cost;
}

void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

bool Costmap2D::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
  if (wx < origin_x_ || wy < origin_y_)
    return false;

  mx = (int)((wx - origin_x_) / resolution_);
  my = (int)((wy - origin_y_) / resolution_);

  if (mx < size_x_ && my < size_y_)
    return true;

  return false;
}

void Costmap2D::worldToMapNoBounds(double wx, double wy, int& mx, int& my) const
{
  mx = (int)((wx - origin_x_) / resolution_);
  my = (int)((wy - origin_y_) / resolution_);
}

void Costmap2D::worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const
{
  // Here we avoid doing any math to wx,wy before comparing them to
  // the bounds, so their values can go out to the max and min values
  // of double floating point.
  if (wx < origin_x_)
  {
    mx = 0;
  }
  else if (wx >= resolution_ * size_x_ + origin_x_)
  {
    mx = size_x_ - 1;
  }
  else
  {
    mx = (int)((wx - origin_x_) / resolution_);
  }

  if (wy < origin_y_)
  {
    my = 0;
  }
  else if (wy >= resolution_ * size_y_ + origin_y_)
  {
    my = size_y_ - 1;
  }
  else
  {
    my = (int)((wy - origin_y_) / resolution_);
  }
}

void Costmap2D::updateOrigin(double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = int((new_origin_x - origin_x_) / resolution_);
  cell_oy = int((new_origin_y - origin_y_) / resolution_);

  // Nothing to update
  if (cell_ox == 0 && cell_oy == 0)
    return;

  // update the origin with the appropriate world coordinates
  origin_x_ += cell_ox * resolution_;
  origin_y_ += cell_oy * resolution_;

  if (std::abs(cell_ox) >= size_x_ || std::abs(cell_oy) >= size_y_)
  {
    // If the new and old maps don't overlap, we can just reset the costmap.
    ROS_DEBUG("Maps don't overlap. Dropping entire data");
    resetMaps();
    return;
  }

  // The size of the windows to copy.
  const unsigned int window_width = size_x_ - std::abs(cell_ox);
  const unsigned int window_height = size_y_ - std::abs(cell_oy);

  assert(window_width <= size_x_ && "window_width out of bounds");
  assert(window_height <= size_y_ && "window_height out of bounds");

  // The stride has the sign of the cell_oy offset; if we move the costmap up,
  // we have to copy from the bottom up; if we move it down, we copy from the
  // top down.
  const std::ptrdiff_t stride = std::copysign(size_x_, cell_oy);

  // The rows will underflow in case of an error.
  const unsigned int start_target_row = cell_oy < 0 ? size_y_ - 1 : 0;
  const unsigned int start_source_row = cell_oy < 0 ? size_y_ - 1 + cell_oy : cell_oy;
  const unsigned int start_target_col = std::max(0, -cell_ox);
  const unsigned int start_source_col = std::max(0, cell_ox);

  assert(start_target_row <= size_y_ && "start_target_row out of bounds");
  assert(start_source_row <= size_y_ && "start_source_row out of bounds");
  assert(start_target_col <= size_x_ && "start_target_col out of bounds");
  assert(start_source_col <= size_x_ && "start_source_col out of bounds");

  unsigned char *source_data = costmap_ + (start_source_row * size_x_ + start_source_col);
  unsigned char *target_data = costmap_ + (start_target_row * size_x_ + start_target_col);

  for (unsigned int row = 0; row != window_height; ++row)
  {
    std::copy_n(source_data, window_width, target_data);
    source_data += stride;
    target_data += stride;
  }

  // Now we have to clear the remaining two rectangles.
  {
    // Horizontal rectangle.
    const unsigned int start_row = cell_oy < 0 ? 0 : size_y_ - cell_oy;
    const unsigned int end_row = cell_oy < 0 ? -cell_oy : size_y_;

    assert(start_row <= end_row && "start_row must be smaller than end_row");
    assert(end_row <= size_y_ && "end_row out of bounds");

    resetMap(0, start_row, size_x_, end_row);
  }

  {
    // Vertical rectangle (smaller one since the map is row-major).
    const unsigned int start_col = cell_ox < 0 ? 0 : size_x_ - cell_ox;
    const unsigned int start_row = std::max(-cell_oy, 0);
    const unsigned int end_col = cell_ox < 0 ? -cell_ox : size_x_;
    const unsigned int end_row = start_row + window_height;

    assert(start_col <= end_col && "start_col must be smaller than end_col");
    assert(start_row <= end_row && "start_row must be smaller than end_row");
    assert(end_col <= size_x_ && "end_col out of bounds");
    assert(end_row <= size_y_ && "end_row out of bounds");

    resetMap(start_col, start_row, end_col, end_row);
  }
}

bool Costmap2D::setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value)
{
  // we assume the polygon is given in the global_frame... we need to transform it to map coordinates
  std::vector<MapLocation> map_polygon;
  for (unsigned int i = 0; i < polygon.size(); ++i)
  {
    MapLocation loc;
    if (!worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y))
    {
      // ("Polygon lies outside map bounds, so we can't fill it");
      return false;
    }
    map_polygon.push_back(loc);
  }

  std::vector<MapLocation> polygon_cells;

  // get the cells that fill the polygon
  convexFillCells(map_polygon, polygon_cells);

  // set the cost of those cells
  for (unsigned int i = 0; i < polygon_cells.size(); ++i)
  {
    unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
    costmap_[index] = cost_value;
  }
  return true;
}

void Costmap2D::polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells)
{
  PolygonOutlineCells cell_gatherer(*this, costmap_, polygon_cells);
  for (unsigned int i = 0; i < polygon.size() - 1; ++i)
  {
    raytraceLine(cell_gatherer, polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
  }
  if (!polygon.empty())
  {
    unsigned int last_index = polygon.size() - 1;
    // we also need to close the polygon by going from the last point to the first
    raytraceLine(cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y);
  }
}

void Costmap2D::convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells)
{
  // we need a minimum polygon of a triangle
  if (polygon.size() < 3)
    return;

  // first get the cells that make up the outline of the polygon
  polygonOutlineCells(polygon, polygon_cells);

  // quick bubble sort to sort points by x
  MapLocation swap;
  unsigned int i = 0;
  while (i < polygon_cells.size() - 1)
  {
    if (polygon_cells[i].x > polygon_cells[i + 1].x)
    {
      swap = polygon_cells[i];
      polygon_cells[i] = polygon_cells[i + 1];
      polygon_cells[i + 1] = swap;

      if (i > 0)
        --i;
    }
    else
      ++i;
  }

  i = 0;
  MapLocation min_pt;
  MapLocation max_pt;
  unsigned int min_x = polygon_cells[0].x;
  unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;

  // walk through each column and mark cells inside the polygon
  for (unsigned int x = min_x; x <= max_x; ++x)
  {
    if (i >= polygon_cells.size() - 1)
      break;

    if (polygon_cells[i].y < polygon_cells[i + 1].y)
    {
      min_pt = polygon_cells[i];
      max_pt = polygon_cells[i + 1];
    }
    else
    {
      min_pt = polygon_cells[i + 1];
      max_pt = polygon_cells[i];
    }

    i += 2;
    while (i < polygon_cells.size() && polygon_cells[i].x == x)
    {
      if (polygon_cells[i].y < min_pt.y)
        min_pt = polygon_cells[i];
      else if (polygon_cells[i].y > max_pt.y)
        max_pt = polygon_cells[i];
      ++i;
    }

    MapLocation pt;
    // loop though cells in the column
    for (unsigned int y = min_pt.y; y <= max_pt.y; ++y)
    {
      pt.x = x;
      pt.y = y;
      polygon_cells.push_back(pt);
    }
  }
}

unsigned int Costmap2D::getSizeInCellsX() const
{
  return size_x_;
}

unsigned int Costmap2D::getSizeInCellsY() const
{
  return size_y_;
}

double Costmap2D::getSizeInMetersX() const
{
  return (size_x_ - 1 + 0.5) * resolution_;
}

double Costmap2D::getSizeInMetersY() const
{
  return (size_y_ - 1 + 0.5) * resolution_;
}

double Costmap2D::getOriginX() const
{
  return origin_x_;
}

double Costmap2D::getOriginY() const
{
  return origin_y_;
}

double Costmap2D::getResolution() const
{
  return resolution_;
}

bool Costmap2D::saveMap(std::string file_name)
{
  FILE *fp = fopen(file_name.c_str(), "w");

  if (!fp)
  {
    return false;
  }

  fprintf(fp, "P2\n%u\n%u\n%u\n", size_x_, size_y_, 0xff);
  for (unsigned int iy = 0; iy < size_y_; iy++)
  {
    for (unsigned int ix = 0; ix < size_x_; ix++)
    {
      unsigned char cost = getCost(ix, iy);
      fprintf(fp, "%d ", cost);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return true;
}

}  // namespace costmap_2d
