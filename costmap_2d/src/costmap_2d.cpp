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
#include <costmap_2d/costmap_2d.h>
#include <cstdio>
#include <sensor_msgs/PointCloud2.h>


#include <costmap_2d/Costmap2DConfig.h>

using namespace std;

namespace costmap_2d{
  Costmap2D::Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, 
      double resolution, double origin_x, double origin_y, double inscribed_radius,
      double circumscribed_radius, double inflation_radius, double max_obstacle_range,
      double max_obstacle_height, double max_raytrace_range, double weight,
      const std::vector<unsigned char>& static_data, unsigned char lethal_threshold, bool track_unknown_space, unsigned char unknown_cost_value) : size_x_(cells_size_x),
  size_y_(cells_size_y), resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y), static_map_(NULL),
  costmap_(NULL), markers_(NULL), max_obstacle_range_(max_obstacle_range), 
  max_obstacle_height_(max_obstacle_height), max_raytrace_range_(max_raytrace_range), cached_costs_(NULL), cached_distances_(NULL), 
  inscribed_radius_(inscribed_radius), circumscribed_radius_(circumscribed_radius), inflation_radius_(inflation_radius),
  weight_(weight), lethal_threshold_(lethal_threshold), track_unknown_space_(track_unknown_space), unknown_cost_value_(unknown_cost_value), inflation_queue_(){
    //create the costmap, static_map, and markers
    costmap_ = new unsigned char[size_x_ * size_y_];
    static_map_ = new unsigned char[size_x_ * size_y_];
    markers_ = new unsigned char[size_x_ * size_y_];
    memset(markers_, 0, size_x_ * size_y_ * sizeof(unsigned char));

    //convert our inflations from world to cell distance
    cell_inscribed_radius_ = max( 0.0, inscribed_radius / resolution_ );
    cell_circumscribed_radius_ = cellDistance(circumscribed_radius);
    cell_inflation_radius_ = cellDistance(inflation_radius);

    //set the cost for the circumscribed radius of the robot
    circumscribed_cost_lb_ = computeCost(cell_circumscribed_radius_);

    computeCaches();

    if(!static_data.empty()){
      ROS_ASSERT_MSG(size_x_ * size_y_ == static_data.size(), "If you want to initialize a costmap with static data, their sizes must match.");

      //make sure the inflation queue is empty at the beginning of the cycle (should always be true)
      ROS_ASSERT_MSG(inflation_queue_.empty(), "The inflation queue must be empty at the beginning of inflation");

      unsigned int index = 0;
      unsigned char* costmap_index = costmap_;
      std::vector<unsigned char>::const_iterator static_data_index =  static_data.begin();

      //initialize the costmap with static data
      for(unsigned int i = 0; i < size_y_; ++i){
        for(unsigned int j = 0; j < size_x_; ++j){
          //check if the static value is above the unknown or lethal thresholds
          if(track_unknown_space_ && unknown_cost_value_ > 0 && *static_data_index == unknown_cost_value_)
            *costmap_index = NO_INFORMATION;
          else if(*static_data_index >= lethal_threshold_)
            *costmap_index = LETHAL_OBSTACLE;
          else
            *costmap_index = FREE_SPACE;

          if(*costmap_index == LETHAL_OBSTACLE){
            unsigned int mx, my;
            indexToCells(index, mx, my);
            enqueue(index, mx, my, mx, my, inflation_queue_);
          }

          ++costmap_index;
          ++static_data_index;
          ++index;
        }
      }

      //now... let's inflate the obstacles
      inflateObstacles(inflation_queue_);

      //we also want to keep a copy of the current costmap as the static map
      memcpy(static_map_, costmap_, size_x_ * size_y_ * sizeof(unsigned char));
    }
    else{
      //everything is unknown initially if we don't have a static map unless we aren't tracking unkown space in which case it is free
      resetMaps();
    }
  }

  void Costmap2D::reconfigure(Costmap2DConfig &config, const Costmap2DConfig &last_config) {
      boost::recursive_mutex::scoped_lock rel(configuration_mutex_);

      max_obstacle_height_ = config.max_obstacle_height;
      max_obstacle_range_ = config.max_obstacle_range;
      max_raytrace_range_ = config.raytrace_range;
      
      if(last_config.inflation_radius != config.inflation_radius)
      {
        inflation_radius_ = config.inflation_radius;
        cell_inflation_radius_ = cellDistance(inflation_radius_);
        computeCaches();
      }

      //only update the origin for the map if the
      if(!config.static_map && (last_config.origin_x != config.origin_x || last_config.origin_y != config.origin_y))
        updateOrigin(config.origin_x, config.origin_y);

      unknown_cost_value_ = config.unknown_cost_value;
      lethal_threshold_ = config.lethal_cost_threshold;

      weight_ = config.cost_scaling_factor;

      if((config.footprint == "" || config.footprint == "[]") && config.robot_radius > 0) {
        inscribed_radius_ = config.robot_radius;
        circumscribed_radius_ = inscribed_radius_;
      }

      finishConfiguration(config);
  }

  void Costmap2D::finishConfiguration(costmap_2d::Costmap2DConfig &config) {
  }

  void Costmap2D::replaceFullMap(double win_origin_x, double win_origin_y,
                             unsigned int data_size_x, unsigned int data_size_y,
                             const std::vector<unsigned char>& static_data){
    boost::recursive_mutex::scoped_lock rfml(configuration_mutex_);

    //delete our old maps
    deleteMaps();

    //update the origin and size of the new map
    size_x_ = data_size_x;
    size_y_ = data_size_y;
    origin_x_ = win_origin_x;
    origin_y_ = win_origin_y;

    //initialize our various maps
    initMaps(size_x_, size_y_);

    //make sure the inflation queue is empty at the beginning of the cycle (should always be true)
    ROS_ASSERT_MSG(inflation_queue_.empty(), "The inflation queue must be empty at the beginning of inflation");

    unsigned int index = 0;
    unsigned char* costmap_index = costmap_;
    std::vector<unsigned char>::const_iterator static_data_index =  static_data.begin();

    //copy static data into the costmap
    for(unsigned int i = 0; i < size_y_; ++i){
      for(unsigned int j = 0; j < size_x_; ++j){
        //check if the static value is above the unknown or lethal thresholds
        if(track_unknown_space_ && unknown_cost_value_ > 0 && *static_data_index == unknown_cost_value_)
          *costmap_index = NO_INFORMATION;
        else if(*static_data_index >= lethal_threshold_)
          *costmap_index = LETHAL_OBSTACLE;
        else
          *costmap_index = FREE_SPACE;

        if(*costmap_index == LETHAL_OBSTACLE){
          unsigned int mx, my;
          indexToCells(index, mx, my);
          enqueue(index, mx, my, mx, my, inflation_queue_);
        }
        ++costmap_index;
        ++static_data_index;
        ++index;
      }
    }

    //now... let's inflate the obstacles
    inflateObstacles(inflation_queue_);

    //we also want to keep a copy of the current costmap as the static map
    memcpy(static_map_, costmap_, size_x_ * size_y_ * sizeof(unsigned char));
  }

  void Costmap2D::replaceStaticMapWindow(double win_origin_x, double win_origin_y, 
                                         unsigned int data_size_x, unsigned int data_size_y, 
                                         const std::vector<unsigned char>& static_data){
    boost::recursive_mutex::scoped_lock stwl(configuration_mutex_);

    unsigned int start_x, start_y;
    if(!worldToMap(win_origin_x, win_origin_y, start_x, start_y) || (start_x + data_size_x) > size_x_ || (start_y + data_size_y) > size_y_){
      ROS_ERROR("You must call replaceStaticMapWindow with a window origin and size that is contained within the map");
      return;
    }


    //we need to compute the region of the costmap that could change from inflation of new obstacles
    unsigned int max_inflation_change = 2 * cell_inflation_radius_;


    
    //make sure that we don't go out of map bounds
    unsigned int copy_sx = std::min(std::max(0, (int)start_x - (int)max_inflation_change), (int)size_x_);
    unsigned int copy_sy = std::min(std::max(0, (int)start_y - (int)max_inflation_change), (int)size_x_);
    unsigned int copy_ex = std::max(std::min((int)size_x_, (int)start_x + (int)data_size_x + (int)max_inflation_change), 0);
    unsigned int copy_ey = std::max(std::min((int)size_y_, (int)start_y + (int)data_size_y + (int)max_inflation_change), 0);

    unsigned int copy_size_x = copy_ex - copy_sx;
    unsigned int copy_size_y = copy_ey - copy_sy;

    //now... we have to compute the window
    double ll_x, ll_y, ur_x, ur_y;
    mapToWorld(copy_sx, copy_sy, ll_x, ll_y);
    mapToWorld(copy_ex, copy_ey, ur_x, ur_y);
    double mid_x = (ll_x + ur_x) / 2;
    double mid_y = (ll_y + ur_y) / 2;
    double size_x = ur_x - ll_x;
    double size_y = ur_y - ll_y;

    //finally... we'll clear all non-lethal costs in the area that could be affected by the map update
    //we'll reinflate after the map update is complete
    clearNonLethal(mid_x, mid_y, size_x, size_y);

    //copy static data into the costmap
    unsigned int start_index = start_y * size_x_ + start_x;
    unsigned char* costmap_index = costmap_ + start_index;
    std::vector<unsigned char>::const_iterator static_data_index =  static_data.begin();
    for(unsigned int i = 0; i < data_size_y; ++i){
      for(unsigned int j = 0; j < data_size_x; ++j){
        //check if the static value is above the unknown or lethal thresholds
        if(track_unknown_space_ && unknown_cost_value_ > 0 && *static_data_index == unknown_cost_value_)
          *costmap_index = NO_INFORMATION;
        else if(*static_data_index >= lethal_threshold_)
          *costmap_index = LETHAL_OBSTACLE;
        else
          *costmap_index = FREE_SPACE;

        ++costmap_index;
        ++static_data_index;
      }
      costmap_index += size_x_ - data_size_x;
    }

    //now, we're ready to reinflate obstacles in the window that has been updated
    //we won't clear all non-lethal obstacles first because the static map update
    //may have included non-lethal costs
    reinflateWindow(mid_x, mid_y, size_x, size_y, false);


    //we also want to keep a copy of the current costmap as the static map... we'll only need to write the region that has changed
    copyMapRegion(costmap_, copy_sx, copy_sy, size_x_, static_map_, copy_sx, copy_sy, size_x_, copy_size_x, copy_size_y);

  }

  void Costmap2D::reshapeStaticMap(double win_origin_x, double win_origin_y,
                                 unsigned int data_size_x, unsigned int data_size_y, const std::vector<unsigned char>& static_data){
    int m_ox, m_oy;
    worldToMapNoBounds(win_origin_x, win_origin_y, m_ox, m_oy);

    //compute the bounds for the size of our new map
    int bl_x = std::min(m_ox, 0);
    int bl_y = std::min(m_oy, 0);
    int ur_x = std::max(m_ox + data_size_x, size_x_);
    int ur_y = std::max(m_oy + data_size_y, size_y_);

    //create a temporary map to hold our static data and copy the old static map into it
    unsigned char* static_map_copy = new unsigned char[size_x_ * size_y_];
    memcpy(static_map_copy, static_map_, size_x_ * size_y_ * sizeof(unsigned char));

    //delete our old maps... the user will lose any 
    //cost information not stored in the static map when reshaping a map
    deleteMaps();

    //update the origin and sizes, and cache data we'll need
    double old_origin_x = origin_x_;
    double old_origin_y = origin_y_;
    unsigned int old_size_x = size_x_;
    unsigned int old_size_y = size_y_;

    size_x_ = ur_x - bl_x;
    size_y_ = ur_y - bl_y;
    origin_x_ = std::min(origin_x_, win_origin_x);
    origin_y_ = std::min(origin_y_, win_origin_y);

    //initialize our various maps
    initMaps(size_x_, size_y_);

    //reset our maps to be full of unknown space if appropriate
    resetMaps();

    //now, copy the old static map into the new costmap
    unsigned int start_x, start_y;
    worldToMap(old_origin_x, old_origin_y, start_x, start_y);
    copyMapRegion(static_map_copy, 0, 0, old_size_x, costmap_, start_x, start_y, size_x_, old_size_x, old_size_y);

    delete[] static_map_copy;

    //now we want to update the map with the new static map data
    replaceStaticMapWindow(win_origin_x, win_origin_y, data_size_x, data_size_y, static_data);
  }

  void Costmap2D::updateStaticMapWindow(double win_origin_x, double win_origin_y, 
                                      unsigned int data_size_x, unsigned int data_size_y, 
                                      const std::vector<unsigned char>& static_data){

    if(data_size_x * data_size_y != static_data.size()){
      ROS_ERROR("The sizes passed in are incorrect for the size of the static data char array. Doing nothing.");
      return;
    }

    //get the map coordinates of the origin of static map window
    int m_ox, m_oy;
    worldToMapNoBounds(win_origin_x, win_origin_y, m_ox, m_oy);

    //if the static map contains the full costmap, then we'll just overwrite the costmap
    if(m_ox <= 0 && m_oy <= 0 && (m_ox + data_size_x) >= size_x_ && (m_oy + data_size_y) >= size_y_){
      replaceFullMap(win_origin_x, win_origin_y, data_size_x, data_size_y, static_data);
    }
    //if the static map overlaps with the costmap, but not completely... we'll have to resize the costmap and maintain certain information
    else if(m_ox < 0 || m_oy < 0 || (m_ox + data_size_x) > size_x_ || (m_oy + data_size_y) > size_y_){
      reshapeStaticMap(win_origin_x, win_origin_y, data_size_x, data_size_y, static_data);
    }
    //if the costmap fully contains the changes we'll make with the static map... then we can just overwrite a portion of the costmap
    else{
      replaceStaticMapWindow(win_origin_x, win_origin_y, data_size_x, data_size_y, static_data);
    }
  }

  void Costmap2D::deleteMaps(){
    //clean up old data
    delete[] costmap_;
    delete[] static_map_;
    delete[] markers_;
  }

  void Costmap2D::deleteKernels(){
    if(cached_distances_ != NULL){
      for(unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i){
        delete[] cached_distances_[i];
      }
      delete[] cached_distances_;
    }

    if(cached_costs_ != NULL){
      for(unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i){
        delete[] cached_costs_[i];
      }
      delete[] cached_costs_;
    }
  }

  void Costmap2D::initMaps(unsigned int size_x, unsigned int size_y){
    costmap_ = new unsigned char[size_x * size_y];
    static_map_ = new unsigned char[size_x * size_y];
    markers_ = new unsigned char[size_x * size_y];

    //reset markers for inflation
    memset(markers_, 0, size_x_ * size_y_ * sizeof(unsigned char));
  }

  void Costmap2D::resetMaps(){
    //reset our maps to have no information
    if(track_unknown_space_){
      memset(static_map_, NO_INFORMATION, size_x_ * size_y_ * sizeof(unsigned char));
      memset(costmap_, NO_INFORMATION, size_x_ * size_y_ * sizeof(unsigned char));
    }
    else{
      memset(static_map_, FREE_SPACE, size_x_ * size_y_ * sizeof(unsigned char));
      memset(costmap_, FREE_SPACE, size_x_ * size_y_ * sizeof(unsigned char));
    }
  }
  
  void Costmap2D::copyKernels(const Costmap2D& map, unsigned int cell_inflation_radius){
    cached_costs_ = new unsigned char*[cell_inflation_radius + 2];
    cached_distances_ = new double*[cell_inflation_radius + 2];
    for(unsigned int i = 0; i <= cell_inflation_radius + 1; ++i){
      cached_costs_[i] = new unsigned char[cell_inflation_radius + 2];
      cached_distances_[i] = new double[cell_inflation_radius + 2];
      for(unsigned int j = 0; j <= cell_inflation_radius + 1; ++j){
        cached_distances_[i][j] = map.cached_distances_[i][j];
        cached_costs_[i][j] = map.cached_costs_[i][j];
      }
    }
  }

  void Costmap2D::copyCostmapWindow(const Costmap2D& map, double win_origin_x, double win_origin_y, double win_size_x, double win_size_y){
    boost::recursive_mutex::scoped_lock cpl(configuration_mutex_);

    //check for self windowing
    if(this == &map){
      ROS_ERROR("Cannot convert this costmap into a window of itself");
      return;
    }

    //clean up old data
    deleteMaps();
    deleteKernels();

    //compute the bounds of our new map
    unsigned int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    if(!map.worldToMap(win_origin_x, win_origin_y, lower_left_x, lower_left_y) 
        || ! map.worldToMap(win_origin_x + win_size_x, win_origin_y + win_size_y, upper_right_x, upper_right_y)){
      ROS_ERROR("Cannot window a map that the window bounds don't fit inside of");
      return;
    }

    size_x_ = upper_right_x - lower_left_x;
    size_y_ = upper_right_y - lower_left_y;
    resolution_ = map.resolution_;
    origin_x_ = win_origin_x;
    origin_y_ = win_origin_y;

    ROS_DEBUG("ll(%d, %d), ur(%d, %d), size(%d, %d), origin(%.2f, %.2f)", 
        lower_left_x, lower_left_y, upper_right_x, upper_right_y, size_x_, size_y_, origin_x_, origin_y_);


    //initialize our various maps and reset markers for inflation
    initMaps(size_x_, size_y_);

    //copy the window of the static map and the costmap that we're taking
    copyMapRegion(map.costmap_, lower_left_x, lower_left_y, map.size_x_, costmap_, 0, 0, size_x_, size_x_, size_y_);
    copyMapRegion(map.static_map_, lower_left_x, lower_left_y, map.size_x_, static_map_, 0, 0, size_x_, size_x_, size_y_);
    
    max_obstacle_range_ = map.max_obstacle_range_;
    max_obstacle_height_ = map.max_obstacle_height_;
    max_raytrace_range_ = map.max_raytrace_range_;

    inscribed_radius_ = map.inscribed_radius_;
    circumscribed_radius_ = map.circumscribed_radius_;
    inflation_radius_ = map.inflation_radius_;

    cell_inscribed_radius_ = map.cell_inscribed_radius_;
    cell_circumscribed_radius_ = map.cell_circumscribed_radius_;
    cell_inflation_radius_ = map.cell_inflation_radius_;

    //set the cost for the circumscribed radius of the robot
    circumscribed_cost_lb_ = map.circumscribed_cost_lb_;

    weight_ = map.weight_;

    //copy the cost and distance kernels
    copyKernels(map, cell_inflation_radius_);
  }

  Costmap2D& Costmap2D::operator=(const Costmap2D& map) {

    //check for self assignement
    if(this == &map)
      return *this;

    //clean up old data
    deleteMaps();
    deleteKernels();

    size_x_ = map.size_x_;
    size_y_ = map.size_y_;
    resolution_ = map.resolution_;
    origin_x_ = map.origin_x_;
    origin_y_ = map.origin_y_;

    //initialize our various maps
    initMaps(size_x_, size_y_);

    //copy the static map
    memcpy(static_map_, map.static_map_, size_x_ * size_y_ * sizeof(unsigned char));

    //copy the cost map
    memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));

    max_obstacle_range_ = map.max_obstacle_range_;
    max_obstacle_height_ = map.max_obstacle_height_;
    max_raytrace_range_ = map.max_raytrace_range_;

    inscribed_radius_ = map.inscribed_radius_;
    circumscribed_radius_ = map.circumscribed_radius_;
    inflation_radius_ = map.inflation_radius_;

    cell_inscribed_radius_ = map.cell_inscribed_radius_;
    cell_circumscribed_radius_ = map.cell_circumscribed_radius_;
    cell_inflation_radius_ = map.cell_inflation_radius_;

    //set the cost for the circumscribed radius of the robot
    circumscribed_cost_lb_ = map.circumscribed_cost_lb_;

    weight_ = map.weight_;

    //copy the cost and distance kernels
    copyKernels(map, cell_inflation_radius_);

    return *this;
  }

  Costmap2D::Costmap2D(const Costmap2D& map) : static_map_(NULL), costmap_(NULL), markers_(NULL), cached_costs_(NULL), cached_distances_(NULL) {
    *this = map;
  }

  //just initialize everything to NULL by default
  Costmap2D::Costmap2D() : size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), static_map_(NULL),
  costmap_(NULL), markers_(NULL), cached_costs_(NULL), cached_distances_(NULL) {}

  Costmap2D::~Costmap2D(){
    deleteMaps();
    deleteKernels();
  }

  unsigned int Costmap2D::cellDistance(double world_dist){
    double cells_dist = max(0.0, ceil(world_dist / resolution_));
    return (unsigned int) cells_dist;
  }

  const unsigned char* Costmap2D::getCharMap() const {
    return costmap_;
  }

  unsigned char Costmap2D::getCost(unsigned int mx, unsigned int my) const {
    ROS_ASSERT_MSG(mx < size_x_ && my < size_y_, "You cannot get the cost of a cell that is outside the bounds of the costmap");
    return costmap_[getIndex(mx, my)];
  }

  void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost) {
    ROS_ASSERT_MSG(mx < size_x_ && my < size_y_, "You cannot set the cost of a cell that is outside the bounds of the costmap");
    costmap_[getIndex(mx, my)] = cost;
  }

  void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const {
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
  }

  bool Costmap2D::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const {
    if(wx < origin_x_ || wy < origin_y_)
      return false;

    mx = (int) ((wx - origin_x_) / resolution_);
    my = (int) ((wy - origin_y_) / resolution_);

    if(mx < size_x_ && my < size_y_)
      return true;

    return false;
  }

  void Costmap2D::worldToMapNoBounds(double wx, double wy, int& mx, int& my) const {
    mx = (int) ((wx - origin_x_) / resolution_);
    my = (int) ((wy - origin_y_) / resolution_);
  }

  void Costmap2D::resetMapOutsideWindow(double wx, double wy, double w_size_x, double w_size_y){
    ROS_ASSERT_MSG(w_size_x >= 0 && w_size_y >= 0, "You cannot specify a negative size window");

    double start_point_x = wx - w_size_x / 2;
    double start_point_y = wy - w_size_y / 2;
    double end_point_x = start_point_x + w_size_x;
    double end_point_y = start_point_y + w_size_y;

    //check start bounds
    start_point_x = max(origin_x_, start_point_x);
    start_point_y = max(origin_y_, start_point_y);

    //check end bounds
    end_point_x = min(origin_x_ + getSizeInMetersX(), end_point_x);
    end_point_y = min(origin_y_ + getSizeInMetersY(), end_point_y);

    unsigned int start_x, start_y, end_x, end_y;

    //check for legality just in case
    if(!worldToMap(start_point_x, start_point_y, start_x, start_y) || !worldToMap(end_point_x, end_point_y, end_x, end_y))
      return;

    ROS_ASSERT(end_x >= start_x && end_y >= start_y);
    unsigned int cell_size_x = end_x - start_x;
    unsigned int cell_size_y = end_y - start_y;

    //we need a map to store the obstacles in the window temporarily
    unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];

    //copy the local window in the costmap to the local map
    copyMapRegion(costmap_, start_x, start_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

    //now we'll reset the costmap to the static map
    memcpy(costmap_, static_map_, size_x_ * size_y_ * sizeof(unsigned char));

    //now we want to copy the local map back into the costmap
    copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);

    //clean up
    delete[] local_map;
  }

  void Costmap2D::updateWorld(double robot_x, double robot_y, 
      const vector<Observation>& observations, const vector<Observation>& clearing_observations){
    boost::recursive_mutex::scoped_lock uwl(configuration_mutex_);

    //reset the markers for inflation
    memset(markers_, 0, size_x_ * size_y_ * sizeof(unsigned char));

    //make sure the inflation queue is empty at the beginning of the cycle (should always be true)
    ROS_ASSERT_MSG(inflation_queue_.empty(), "The inflation queue must be empty at the beginning of inflation");

    //raytrace freespace
    raytraceFreespace(clearing_observations);

    //if we raytrace X meters out... we must re-inflate obstacles within the containing square of that circle
    double inflation_window_size = 2 * (max_raytrace_range_ + inflation_radius_);

    //clear all non-lethal obstacles in preparation for re-inflation
    clearNonLethal(robot_x, robot_y, inflation_window_size, inflation_window_size);

    //reset the inflation window
    resetInflationWindow(robot_x, robot_y, inflation_window_size + 2 * inflation_radius_, inflation_window_size + 2 * inflation_radius_, inflation_queue_, false);

    //now we also want to add the new obstacles we've received to the cost map
    updateObstacles(observations, inflation_queue_);

    inflateObstacles(inflation_queue_);
  }
  
  void Costmap2D::reinflateWindow(double wx, double wy, double w_size_x, double w_size_y, bool clear){
    boost::recursive_mutex::scoped_lock rwl(configuration_mutex_);
    //reset the markers for inflation
    memset(markers_, 0, size_x_ * size_y_ * sizeof(unsigned char));

    //make sure the inflation queue is empty at the beginning of the cycle (should always be true)
    ROS_ASSERT_MSG(inflation_queue_.empty(), "The inflation queue must be empty at the beginning of inflation");

    //reset the inflation window.. adds all lethal costs to the queue for re-propagation
    resetInflationWindow(wx, wy, w_size_x, w_size_y, inflation_queue_, clear);

    //inflate the obstacles
    inflateObstacles(inflation_queue_);

  }

  void Costmap2D::updateObstacles(const vector<Observation>& observations, priority_queue<CellData>& inflation_queue){
    //place the new obstacles into a priority queue... each with a priority of zero to begin with
    for(vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it){
      const Observation& obs = *it;

      const pcl::PointCloud<pcl::PointXYZ>& cloud =obs.cloud_;

      double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;


      for(unsigned int i = 0; i < cloud.points.size(); ++i){
        //if the obstacle is too high or too far away from the robot we won't add it
        if(cloud.points[i].z > max_obstacle_height_){
          ROS_DEBUG("The point is too high");
          continue;
        }

        //compute the squared distance from the hitpoint to the pointcloud's origin
        double sq_dist = (cloud.points[i].x - obs.origin_.x) * (cloud.points[i].x - obs.origin_.x)
          + (cloud.points[i].y - obs.origin_.y) * (cloud.points[i].y - obs.origin_.y)
          + (cloud.points[i].z - obs.origin_.z) * (cloud.points[i].z - obs.origin_.z);

        //if the point is far enough away... we won't consider it
        if(sq_dist >= sq_obstacle_range){
          ROS_DEBUG("The point is too far away");
          continue;
        }

        //now we need to compute the map coordinates for the observation
        unsigned int mx, my;
        if(!worldToMap(cloud.points[i].x, cloud.points[i].y, mx, my)){
          ROS_DEBUG("Computing map coords failed");
          continue;
        }

        unsigned int index = getIndex(mx, my);

        //push the relevant cell index back onto the inflation queue
        enqueue(index, mx, my, mx, my, inflation_queue);
      }
    }
  }

  void Costmap2D::inflateObstacles(priority_queue<CellData>& inflation_queue){
    while(!inflation_queue.empty()){
      //get the highest priority cell and pop it off the priority queue
      const CellData& current_cell = inflation_queue.top();

      unsigned int index = current_cell.index_;
      unsigned int mx = current_cell.x_;
      unsigned int my = current_cell.y_;
      unsigned int sx = current_cell.src_x_;
      unsigned int sy = current_cell.src_y_;

      //attempt to put the neighbors of the current cell onto the queue
      if(mx > 0)
        enqueue(index - 1, mx - 1, my, sx, sy, inflation_queue); 
      if(my > 0)
        enqueue(index - size_x_, mx, my - 1, sx, sy, inflation_queue);
      if(mx < size_x_ - 1)
        enqueue(index + 1, mx + 1, my, sx, sy, inflation_queue);
      if(my < size_y_ - 1)
        enqueue(index + size_x_, mx, my + 1, sx, sy, inflation_queue);

      //remove the current cell from the priority queue
      inflation_queue.pop();
    }
  }


  void Costmap2D::raytraceFreespace(const std::vector<Observation>& clearing_observations){
    for(unsigned int i = 0; i < clearing_observations.size(); ++i){
      raytraceFreespace(clearing_observations[i]);
    }
  }

  void Costmap2D::raytraceFreespace(const Observation& clearing_observation){
    //create the functor that we'll use to clear cells from the costmap
    ClearCell clearer(costmap_);

    double ox = clearing_observation.origin_.x;
    double oy = clearing_observation.origin_.y;
    pcl::PointCloud<pcl::PointXYZ> cloud = clearing_observation.cloud_;

    //get the map coordinates of the origin of the sensor 
    unsigned int x0, y0;
    if(!worldToMap(ox, oy, x0, y0)){
      ROS_WARN_THROTTLE(1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.", ox, oy);
      return;
    }

    //we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
    double map_end_x = origin_x_ + getSizeInMetersX();
    double map_end_y = origin_y_ + getSizeInMetersY();

    //for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
    for(unsigned int i = 0; i < cloud.points.size(); ++i){
      double wx = cloud.points[i].x;
      double wy = cloud.points[i].y;

      //now we also need to make sure that the enpoint we're raytracing 
      //to isn't off the costmap and scale if necessary
      double a = wx - ox;
      double b = wy - oy;

      //the minimum value to raytrace from is the origin
      if(wx < origin_x_){
        double t = (origin_x_ - ox) / a;
        wx = origin_x_;
        wy = oy + b * t;
      }
      if(wy < origin_y_){
        double t = (origin_y_ - oy) / b;
        wx = ox + a * t;
        wy = origin_y_;
      }

      //the maximum value to raytrace to is the end of the map
      if(wx > map_end_x){
        double t = (map_end_x - ox) / a;
        wx = map_end_x;
        wy = oy + b * t;
      }
      if(wy > map_end_y){
        double t = (map_end_y - oy) / b;
        wx = ox + a * t;
        wy = map_end_y;
      }

      //now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
      unsigned int x1, y1;

      //check for legality just in case
      if(!worldToMap(wx, wy, x1, y1))
        continue;

      unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);

      //and finally... we can execute our trace to clear obstacles along that line
      raytraceLine(clearer, x0, y0, x1, y1, cell_raytrace_range);
    }
  }

  void Costmap2D::clearNonLethal(double wx, double wy, double w_size_x, double w_size_y, bool clear_no_info){
    //get the cell coordinates of the center point of the window
    unsigned int mx, my;
    if(!worldToMap(wx, wy, mx, my))
      return;

    //compute the bounds of the window
    double start_x = wx - w_size_x / 2;
    double start_y = wy - w_size_y / 2;
    double end_x = start_x + w_size_x;
    double end_y = start_y + w_size_y;

    //scale the window based on the bounds of the costmap
    start_x = max(origin_x_, start_x);
    start_y = max(origin_y_, start_y);

    end_x = min(origin_x_ + getSizeInMetersX(), end_x);
    end_y = min(origin_y_ + getSizeInMetersY(), end_y);

    //get the map coordinates of the bounds of the window
    unsigned int map_sx, map_sy, map_ex, map_ey;

    //check for legality just in case
    if(!worldToMap(start_x, start_y, map_sx, map_sy) || !worldToMap(end_x, end_y, map_ex, map_ey))
      return;

    //we know that we want to clear all non-lethal obstacles in this window to get it ready for inflation
    unsigned int index = getIndex(map_sx, map_sy);
    unsigned char* current = &costmap_[index];
    for(unsigned int j = map_sy; j <= map_ey; ++j){
      for(unsigned int i = map_sx; i <= map_ex; ++i){
        //if the cell is a lethal obstacle... we'll keep it and queue it, otherwise... we'll clear it
        if(*current != LETHAL_OBSTACLE){
          if(clear_no_info || *current != NO_INFORMATION) 
            *current = FREE_SPACE;
        }
        current++;
        index++;
      }
      current += size_x_ - (map_ex - map_sx) - 1;
      index += size_x_ - (map_ex - map_sx) - 1;
    }
  }

  void Costmap2D::resetInflationWindow(double wx, double wy, double w_size_x, double w_size_y,
      priority_queue<CellData>& inflation_queue, bool clear){
    //get the cell coordinates of the center point of the window
    unsigned int mx, my;
    if(!worldToMap(wx, wy, mx, my))
      return;

    //compute the bounds of the window
    double start_x = wx - w_size_x / 2;
    double start_y = wy - w_size_y / 2;
    double end_x = start_x + w_size_x;
    double end_y = start_y + w_size_y;

    //scale the window based on the bounds of the costmap
    start_x = max(origin_x_, start_x);
    start_y = max(origin_y_, start_y);

    end_x = min(origin_x_ + getSizeInMetersX(), end_x);
    end_y = min(origin_y_ + getSizeInMetersY(), end_y);

    //get the map coordinates of the bounds of the window
    unsigned int map_sx, map_sy, map_ex, map_ey;

    //check for legality just in case
    if(!worldToMap(start_x, start_y, map_sx, map_sy) || !worldToMap(end_x, end_y, map_ex, map_ey)){
      ROS_ERROR("Bounds not legal for reset window. Doing nothing.");
      return;
    }

    //we know that we want to clear all non-lethal obstacles in this window to get it ready for inflation
    unsigned int index = getIndex(map_sx, map_sy);
    unsigned char* current = &costmap_[index];
    for(unsigned int j = map_sy; j <= map_ey; ++j){
      for(unsigned int i = map_sx; i <= map_ex; ++i){
        //if the cell is a lethal obstacle... we'll keep it and queue it, otherwise... we'll clear it
        if(*current == LETHAL_OBSTACLE)
          enqueue(index, i, j, i, j, inflation_queue);
        else if(clear && *current != NO_INFORMATION)
          *current = FREE_SPACE;
        current++;
        index++;
      }
      current += size_x_ - (map_ex - map_sx) - 1;
      index += size_x_ - (map_ex - map_sx) - 1;
    }
  }

  void Costmap2D::computeCaches() {
    //based on the inflation radius... compute distance and cost caches
    cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
    cached_distances_ = new double*[cell_inflation_radius_ + 2];
    for(unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i){
      cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
      cached_distances_[i] = new double[cell_inflation_radius_ + 2];
      for(unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j){
        cached_distances_[i][j] = sqrt(i*i + j*j);
        cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
      }
    }
  }

  void Costmap2D::updateOrigin(double new_origin_x, double new_origin_y){
    //project the new origin into the grid
    int cell_ox, cell_oy;
    cell_ox = int((new_origin_x - origin_x_) / resolution_);
    cell_oy = int((new_origin_y - origin_y_) / resolution_);

    //compute the associated world coordinates for the origin cell
    //beacuase we want to keep things grid-aligned
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

  bool Costmap2D::setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value) {
    //we assume the polygon is given in the global_frame... we need to transform it to map coordinates
    std::vector<MapLocation> map_polygon;
    for(unsigned int i = 0; i < polygon.size(); ++i){
      MapLocation loc;
      if(!worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y)){
        ROS_DEBUG("Polygon lies outside map bounds, so we can't fill it");
        return false;
      }
      map_polygon.push_back(loc);
    }

    std::vector<MapLocation> polygon_cells;

    //get the cells that fill the polygon
    convexFillCells(map_polygon, polygon_cells);

    //set the cost of those cells
    for(unsigned int i = 0; i < polygon_cells.size(); ++i){
      unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
      costmap_[index] = cost_value;
    }
    return true;
  }

  void Costmap2D::polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells){
    PolygonOutlineCells cell_gatherer(*this, costmap_, polygon_cells);
    for(unsigned int i = 0; i < polygon.size() - 1; ++i){
      raytraceLine(cell_gatherer, polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y); 
    }
    if(!polygon.empty()){
      unsigned int last_index = polygon.size() - 1;
      //we also need to close the polygon by going from the last point to the first
      raytraceLine(cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y);
    }
  }

  void Costmap2D::convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells){
    //we need a minimum polygon of a traingle
    if(polygon.size() < 3)
      return;

    //first get the cells that make up the outline of the polygon
    polygonOutlineCells(polygon, polygon_cells);

    //quick bubble sort to sort points by x
    MapLocation swap;
    unsigned int i = 0;
    while(i < polygon_cells.size() - 1){
      if(polygon_cells[i].x > polygon_cells[i + 1].x){
        swap = polygon_cells[i];
        polygon_cells[i] = polygon_cells[i + 1];
        polygon_cells[i + 1] = swap;

        if(i > 0)
          --i;
      }
      else
        ++i;
    }

    i = 0;
    MapLocation min_pt;
    MapLocation max_pt;
    unsigned int min_x = polygon_cells[0].x;
    unsigned int max_x = polygon_cells[polygon_cells.size() -1].x;

    //walk through each column and mark cells inside the polygon
    for(unsigned int x = min_x; x <= max_x; ++x){
      if(i >= polygon_cells.size() - 1)
        break;

      if(polygon_cells[i].y < polygon_cells[i + 1].y){
        min_pt = polygon_cells[i];
        max_pt = polygon_cells[i + 1];
      }
      else{
        min_pt = polygon_cells[i + 1];
        max_pt = polygon_cells[i];
      }

      i += 2;
      while(i < polygon_cells.size() && polygon_cells[i].x == x){
        if(polygon_cells[i].y < min_pt.y)
          min_pt = polygon_cells[i];
        else if(polygon_cells[i].y > max_pt.y)
          max_pt = polygon_cells[i];
        ++i;
      }

      MapLocation pt;
      //loop though cells in the column
      for(unsigned int y = min_pt.y; y < max_pt.y; ++y){
        pt.x = x;
        pt.y = y;
        polygon_cells.push_back(pt);

      }
    }
  }

  unsigned int Costmap2D::getSizeInCellsX() const{
    return size_x_;
  }

  unsigned int Costmap2D::getSizeInCellsY() const{
    return size_y_;
  }

  double Costmap2D::getSizeInMetersX() const{
    return (size_x_ - 1 + 0.5) * resolution_;
  }

  double Costmap2D::getSizeInMetersY() const{
    return (size_y_ - 1 + 0.5) * resolution_;
  }

  double Costmap2D::getOriginX() const{
    return origin_x_;
  }

  double Costmap2D::getOriginY() const{
    return origin_y_;
  }

  double Costmap2D::getResolution() const{
    return resolution_;
  }

  bool Costmap2D::isCircumscribedCell(unsigned int x, unsigned int y) const {
    unsigned char cost = getCost(x, y);
    if(cost < INSCRIBED_INFLATED_OBSTACLE && cost >= circumscribed_cost_lb_)
      return true;
    return false;
  }

  void Costmap2D::saveMap(std::string file_name){
    FILE *fp = fopen(file_name.c_str(), "w");

    if(!fp){
      ROS_WARN("Can't open file %s", file_name.c_str());
      return;
    }

    fprintf(fp, "P2\n%d\n%d\n%d\n", size_x_, size_y_, 0xff); 
    for(unsigned int iy = 0; iy < size_y_; iy++) {
      for(unsigned int ix = 0; ix < size_x_; ix++) {
        unsigned char cost = getCost(ix,iy);
        if (cost == LETHAL_OBSTACLE) {
          fprintf(fp, "255 ");
        } 
        else if (cost == NO_INFORMATION){
          fprintf(fp, "180 ");
        }
        else if (cost == INSCRIBED_INFLATED_OBSTACLE ) {
          fprintf(fp, "128 ");
        } 
        else if (cost > 0){
          fprintf(fp, "50 ");
        }
        else {
          fprintf(fp, "0 ");
        }
      }
      fprintf(fp, "\n");
    }
    fclose(fp);
  }

};
