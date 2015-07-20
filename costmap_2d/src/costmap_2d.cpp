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
#include <costmap_2d/layer.h>
#include <cstdio>

using namespace std;

#include <string>      // for string
#include <algorithm>   // for min
#include <vector>

namespace costmap_2d
{
Costmap2D::Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
                     double origin_x, double origin_y, unsigned char default_value) :
    size_x_(cells_size_x), size_y_(cells_size_y), resolution_(resolution), origin_x_(origin_x), 
    origin_y_(origin_y), costmap_(NULL), default_value_(default_value)
{
  access_ = new boost::shared_mutex();

  //create the costmap
  initMaps(size_x_, size_y_);
  resetMaps();
}

void Costmap2D::deleteMaps()
{
  //clean up data
  boost::unique_lock < boost::shared_mutex > lock(*access_);
  if(costmap_)
    delete[] costmap_;
  costmap_ = NULL;
}

void Costmap2D::initMaps(unsigned int size_x, unsigned int size_y)
{
  boost::unique_lock < boost::shared_mutex > lock(*access_);
  if (costmap_)
    delete[] costmap_;
  costmap_ = new unsigned char[size_x * size_y];
}


Costmap2DPtr Costmap2D::addNamedCostmap2D(const std::string& map_name, Costmap2DPtr map)
{
  child_maps_[map_name] = map;
  return map;
}

Costmap2DPtr Costmap2D::getNamedCostmap2D(const std::string &map_name)
{
  if (child_maps_.find(map_name) == child_maps_.end())
    return Costmap2DPtr();  // NULL pointer
  return child_maps_[map_name];
}


void Costmap2D::removeNamedCostmap2D(const std::string& map_name)
{
    std::map<std::string, Costmap2DPtr>::iterator it;

    it = child_maps_.find( map_name );
    
    if( it != child_maps_.end() )
    {
      child_maps_.erase(it);
    }
}
  
void Costmap2D::removeAllNamedCostmap2D()
{
    child_maps_.clear();
}

int& Costmap2D::namedFlag(const std::string& flag_name)
{
  return named_flags_[flag_name];
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


// templated version so that the compiler can optimize out the switch statement
template<int policy>
void T_copyCellsTo(Costmap2D& src_map, Costmap2D& dst_map,
                          int src_x0, int src_y0,
                          int dst_x0, int dst_y0,
                          unsigned int num_x,  unsigned int num_y)
{
  if (policy == Costmap2D::None)
    return;

  unsigned char* src = src_map.getCharMap();
  unsigned char* dst = dst_map.getCharMap();

  if (!src)
  {
    ROS_ERROR("Source buffer NULL in Costmap2D::copyCellsTo()");
    return;
  }
  if (!dst)
  {
    ROS_ERROR("Destination buffer NULL in Costmap2D::copyCellsTo()");
    return;
  }

  const int src_nx = src_map.getSizeInCellsX();
  const int src_ny = src_map.getSizeInCellsY();

  const int dst_nx = dst_map.getSizeInCellsX();
  const int dst_ny = dst_map.getSizeInCellsY();

  // inelegant but setup to accomodate data propagation to other maps
  for (int y = 0; y < num_y; y++)
  {
    // first checking for in bound data
    if ((src_y0 + y >= 0 )    && (dst_y0 + y >= 0 ))
    if ((src_y0 + y < src_ny) && (dst_y0 + y < dst_ny))
    {
      for (int x = 0; x < num_x; x++)
      {
        if ((src_x0 + x >= 0 )    && (dst_x0 + x >= 0 ))
        if ((src_x0 + x < src_nx) && (dst_x0 + x < dst_nx))
        {
          const int dst_idx = dst_x0 + x + (dst_y0 + y) * dst_nx;
          const int src_idx = src_x0 + x + (src_y0 + y) * src_nx;

          unsigned char& dst_val = dst[dst_idx];
          unsigned char& src_val = src[src_idx];

          switch(policy)
          {
            case Costmap2D::TrueOverwrite:
              dst_val = src_val;
              break;

            case Costmap2D::Overwrite:
              if (src_val != NO_INFORMATION)
                dst_val = src_val;
              break;

            case Costmap2D::Max:
              if(dst_val == NO_INFORMATION)
              {
                dst_val = src_val;
                break;
              }
              if(src_val == NO_INFORMATION)
                break;
              dst_val = std::max(dst_val, src_val);
              break;
            case Costmap2D::Zero:
              dst_val = 0;
              break;
            case Costmap2D::NoInformation:
              dst_val = NO_INFORMATION;
              break;
            case Costmap2D::None:
              break;
          }

        }
      }
    }
  }
}


void Costmap2D::copyCellsTo(Costmap2D& costmap, int src_x0, int src_y0,
                                                int dst_x0, int dst_y0,
                                                int num_x,  int num_y, CopyCellPolicy policy)
{
  switch(policy)
  {
  case None:
    return T_copyCellsTo<None>(*this, costmap, src_x0, src_y0, dst_x0, dst_y0, num_x, num_y);
  case TrueOverwrite:
    return T_copyCellsTo<TrueOverwrite>(*this, costmap, src_x0, src_y0, dst_x0, dst_y0, num_x, num_y);
  case Overwrite:
    return T_copyCellsTo<Overwrite>(*this, costmap, src_x0, src_y0, dst_x0, dst_y0, num_x, num_y);
  case Max:
    return T_copyCellsTo<Max>(*this, costmap, src_x0, src_y0, dst_x0, dst_y0, num_x, num_y);
  case Zero:
    return T_copyCellsTo<Zero>(*this, costmap, src_x0, src_y0, dst_x0, dst_y0, num_x, num_y);
  case NoInformation:
    return T_copyCellsTo<NoInformation>(*this, costmap, src_x0, src_y0, dst_x0, dst_y0, num_x, num_y);
  }
}

void Costmap2D::copyCellsTo(Costmap2DPtr map, int src_x0, int src_y0,
                            int dst_x0, int dst_y0,
                            int num_x, int num_y, CopyCellPolicy policy)
{
  if (map.get())
    copyCellsTo(*map.get(), src_x0, src_y0, dst_x0, dst_y0, num_x, num_y, policy);
  else
    ROS_ERROR("NULL pointer in Costmap2D::copyCellsTo");
}

void Costmap2D::copyCellsTo(Costmap2D &map, int x0, int y0, int num_x, int num_y, CopyCellPolicy policy)
{
  copyCellsTo(map, x0, y0, x0, y0, num_x, num_y, policy);
}

void Costmap2D::copyCellsTo(Costmap2DPtr map, int x0, int y0, int num_x, int num_y, CopyCellPolicy policy)
{
  copyCellsTo(map, x0, y0, x0, y0, num_x, num_y, policy);
}

void Costmap2D::copyCellsTo(Costmap2D &map, CopyCellPolicy policy)
{
  copyCellsTo(map, 0, 0, 0, 0, getSizeInCellsX(), getSizeInCellsY(), policy);
}

void Costmap2D::copyCellsTo(Costmap2DPtr map, CopyCellPolicy policy)
{
  copyCellsTo(map, 0, 0, 0, 0, getSizeInCellsX(), getSizeInCellsY(), policy);
}

void Costmap2D::resetMaps()
{
  boost::unique_lock < boost::shared_mutex > lock(*access_);
  memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

void Costmap2D::resetMap(int x0, int y0, int xn, int yn)
{
  setMapCost(x0, y0, xn, yn, getDefaultValue());
}

void Costmap2D::resetMap()
{
  setMapCost(0, 0, getSizeInCellsX(), getSizeInCellsY(), getDefaultValue());
}

void Costmap2D::setMapCost(int x0, int y0, int xn, int yn, const unsigned char value)
{
  boost::unique_lock < boost::shared_mutex > lock(*(access_));
  
  // if the size is zero for some reason, don't do anything
  if (size_x_ * size_y_ == 0)
    return;
  
  // Make sure the bounds of the write are within range
  x0 = std::max(0, x0);
  y0 = std::max(0, y0);

  xn = std::min((int)size_x_, xn);
  yn = std::min((int)size_y_, yn);
  
  unsigned int len = xn - x0;
  for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_)
    memset(costmap_ + y, value, len * sizeof(unsigned char));
}

void Costmap2D::setMapCost(const unsigned char value)
{
  setMapCost(0, 0, getSizeInCellsX(), getSizeInCellsY(), value);
}

bool Costmap2D::copyCostmapWindow(const Costmap2D& map, double win_origin_x, double win_origin_y, double win_size_x,
                                  double win_size_y)
{
  //check for self windowing
  if (this == &map)
  {
    // ROS_ERROR("Cannot convert this costmap into a window of itself");
    return false;
  }

  //clean up old data
  deleteMaps();

  //compute the bounds of our new map
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

  //initialize our various maps and reset markers for inflation
  initMaps(size_x_, size_y_);

  //copy the window of the static map and the costmap that we're taking
  copyMapRegion(map.costmap_, lower_left_x, lower_left_y, map.size_x_, costmap_, 0, 0, size_x_, size_x_, size_y_);
  return true;
}

Costmap2D& Costmap2D::operator=(const Costmap2D& map)
{

  //check for self assignement
  if (this == &map)
    return *this;

  //clean up old data
  deleteMaps();

  size_x_ = map.size_x_;
  size_y_ = map.size_y_;
  resolution_ = map.resolution_;
  origin_x_ = map.origin_x_;
  origin_y_ = map.origin_y_;

  //initialize our various maps
  initMaps(size_x_, size_y_);

  //copy the cost map
  memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));

  return *this;
}

Costmap2DPtr Costmap2D::createReducedResolutionMap(int factor)
{
  return Costmap2DPtr(
        new Costmap2D( (getSizeInCellsX()  + (factor-1)) / factor,
                        (getSizeInCellsY()  + (factor-1)) / factor,
                        getResolution() * factor,
                        getOriginX(),
                        getOriginY(),
                        getDefaultValue() ) );
}

Costmap2D::Costmap2D(const Costmap2D& map) :
    costmap_(NULL)
{
  *this = map;
}

//just initialize everything to NULL by default
Costmap2D::Costmap2D() :
    size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL)
{
  access_ = new boost::shared_mutex();
}

Costmap2D::~Costmap2D()
{
  deleteMaps();
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


unsigned char Costmap2D::getCost(unsigned int idx) const
{
  return costmap_[idx];
}

void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
  setCost(getIndex(mx, my), cost);
}

void Costmap2D::setCost(unsigned int index, unsigned char cost)
{
  if(index >= 0 && index < size_x_ * size_y_)
    costmap_[index] = cost;
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
  if( wx < origin_x_ )
  {
    mx = 0;
  }
  else if( wx > resolution_ * size_x_ + origin_x_ )
  {
    mx = size_x_ - 1;
  }
  else
  {
    mx = (int)((wx - origin_x_) / resolution_);
  }

  if( wy < origin_y_ )
  {
    my = 0;
  }
  else if( wy > resolution_ * size_y_ + origin_y_ )
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
  //project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = int((new_origin_x - origin_x_) / resolution_);
  cell_oy = int((new_origin_y - origin_y_) / resolution_);

  //compute the associated world coordinates for the origin cell
  //because we want to keep things grid-aligned
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  //To save casting from unsigned int to int a bunch of times
  int size_x = size_x_;
  int size_y = size_y_;

  //we need to compute the overlap of the new and existing windows
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = min(max(cell_ox, 0), size_x);
  lower_left_y = min(max(cell_oy, 0), size_y);
  upper_right_x = min(max(cell_ox + size_x, 0), size_x);
  upper_right_y = min(max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  //we need a map to store the obstacles in the window temporarily
  unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];

  //copy the local window in the costmap to the local map
  copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

  //now we'll set the costmap to be completely unknown if we track unknown space
  resetMaps();

  //update the origin with the appropriate world coordinates
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  //compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  //now we want to copy the overlapping information back into the map, but in its new location
  copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);

  //make sure to clean up
  delete[] local_map;
}

bool Costmap2D::setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value)
{
  //we assume the polygon is given in the global_frame... we need to transform it to map coordinates
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

  //get the cells that fill the polygon
  convexFillCells(map_polygon, polygon_cells);

  //set the cost of those cells
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
    //we also need to close the polygon by going from the last point to the first
    raytraceLine(cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y);
  }
}

void Costmap2D::convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells)
{
  //we need a minimum polygon of a triangle
  if (polygon.size() < 3)
    return;

  //first get the cells that make up the outline of the polygon
  polygonOutlineCells(polygon, polygon_cells);

  //quick bubble sort to sort points by x
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

  //walk through each column and mark cells inside the polygon
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
    //loop though cells in the column
    for (unsigned int y = min_pt.y; y < max_pt.y; ++y)
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
