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
#include <voxel_grid/voxel_grid.h>
#include <sys/time.h>
#include <ros/console.h>
#include <voxel_grid/basic_marker.h>
#include <voxel_grid/basic_clearer.h>

namespace voxel_grid {
  VoxelGrid::VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z)
  {
    size_x_ = size_x; 
    size_y_ = size_y; 
    size_z_ = size_z; 

    if(size_z_ > 16){
      ROS_ERROR("Error, this implementation can only support up to 16 z values (%d)", size_z_);
      size_z_ = 16;
    }

    accuracy_multiplier_ = 1;

    data_ = new uint32_t[size_x_ * size_y_];
    uint32_t unknown_col = ~((uint32_t)0)>>16;
    uint32_t* col = data_;
    for(unsigned int i = 0; i < size_x_ * size_y_; ++i){
      *col = unknown_col;
      ++col;
    }
  }

  void VoxelGrid::resize(unsigned int size_x, unsigned int size_y, unsigned int size_z)
  {
    //if we're not actually changing the size, we can just reset things
    if(size_x == size_x_ && size_y == size_y_ && size_z == size_z_){
      reset();
      return;
    }

    delete[] data_;
    size_x_ = size_x; 
    size_y_ = size_y; 
    size_z_ = size_z; 

    if(size_z_ > 16){
      ROS_INFO("Error, this implementation can only support up to 16 z values (%d)", size_z); 
      size_z_ = 16;
    }

    data_ = new uint32_t[size_x_ * size_y_];
    uint32_t unknown_col = ~((uint32_t)0)>>16;
    uint32_t* col = data_;
    for(unsigned int i = 0; i < size_x_ * size_y_; ++i){
      *col = unknown_col;
      ++col;
    }
  }

  VoxelGrid::~VoxelGrid()
  {
    delete [] data_;
  }

  void VoxelGrid::reset(){
    uint32_t unknown_col = ~((uint32_t)0)>>16;
    uint32_t* col = data_;
    for(unsigned int i = 0; i < size_x_ * size_y_; ++i){
      *col = unknown_col;
      ++col;
    }
  }

  int VoxelGrid::getRaytraceAxis(double x0, double y0, double z0,
				 double x1, double y1, double z1) {
    double abs_dx = std::abs(x1 - x0);
    double abs_dy = std::abs(y1 - y0);
    double abs_dz = std::abs(z1 - z0);
    if (abs_dy >= abs_dx && abs_dy >= abs_dz) return 1;
    if (abs_dz >= abs_dx && abs_dz >= abs_dy) return 2;
    return 0;
  }

  unsigned int VoxelGrid::getNumSteps(double x0, double y0, double z0,
				      double x1, double y1, double z1) {
    int axis = getRaytraceAxis(x0, y0, z0, x1, y1, z1);
    switch (axis) {
      case 0: return static_cast <unsigned int> (std::floor(std::abs(x1 - x0)) + 1);
      case 1: return static_cast <unsigned int> (std::floor(std::abs(y1 - y0)) + 1);
      case 2: return static_cast <unsigned int> (std::floor(std::abs(z1 - z0)) + 1);
    }
  }

  void VoxelGrid::markVoxelLine(double x0, double y0, double z0,
				double x1, double y1, double z1,
				unsigned int max_length){
    if(x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1>=size_x_ || y1>=size_y_ || z1>=size_z_){
      ROS_DEBUG("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)", x0, y0, z0, x1, y1, z1,
          size_x_, size_y_, size_z_);
      return;
    }
    BasicMarker basic_marker(data_);
    int xyz = getRaytraceAxis(x0, y0, z0, x1, y1, z1);
    unsigned int num_steps = getNumSteps(x0, y0, z0, x1, y1, z1);
    unsigned int width = std::min(max_length, size_x_);
    raytraceLine(&basic_marker, x0, y0, z0, x1, y1, z1, width, xyz, num_steps);
  }

  void VoxelGrid::clearVoxelLine(double x0, double y0, double z0,
				 double x1, double y1, double z1,
				 unsigned int max_length){
    if(x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1>=size_x_ || y1>=size_y_ || z1>=size_z_){
      ROS_DEBUG("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)", x0, y0, z0, x1, y1, z1,
          size_x_, size_y_, size_z_);
      return;
    }
    BasicClearer basic_clearer(data_);
    int xyz = getRaytraceAxis(x0, y0, z0, x1, y1, z1);
    unsigned int num_steps = getNumSteps(x0, y0, z0, x1, y1, z1);
    unsigned int width = std::min(max_length, size_x_);
    raytraceLine(&basic_clearer, x0, y0, z0, x1, y1, z1, width, xyz, num_steps);
  }

  void VoxelGrid::clearVoxelLineInMap(double x0, double y0, double z0,
				      double x1, double y1, double z1,
				      AbstractGridUpdater* clearer,
				      unsigned int max_length)
  {
    int xyz = getRaytraceAxis(x0, y0, z0, x1, y1, z1);
    unsigned int num_steps = getNumSteps(x0, y0, z0, x1, y1, z1);
    unsigned int width = std::min(max_length, size_x_*10);
    raytraceLine(clearer, x0, y0, z0, x1, y1, z1, width, xyz,
		 num_steps);
  }

  VoxelStatus VoxelGrid::getVoxel(unsigned int x, unsigned int y, unsigned int z)
  {
    if(x >= size_x_ || y >= size_y_ || z >= size_z_){
      ROS_DEBUG("Error, voxel out of bounds. (%d, %d, %d)\n", x, y, z);
      return UNKNOWN;
    }
    uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
    uint32_t result = data_[y * size_x_ + x] & full_mask;
    unsigned int bits = numBits(result);

    // known marked: 11 = 2 bits, unknown: 01 = 1 bit, known free: 00 = 0 bits
    if(bits < 2){
      if(bits < 1)
        return FREE;

      return UNKNOWN;
    }

    return MARKED;
  }

  VoxelStatus VoxelGrid::getVoxelColumn(unsigned int x, unsigned int y, unsigned int unknown_threshold, unsigned int marked_threshold)
  {
    if(x >= size_x_ || y >= size_y_){
      ROS_DEBUG("Error, voxel out of bounds. (%d, %d)\n", x, y);
      return UNKNOWN;
    }
    
    uint32_t* col = &data_[y * size_x_ + x];

    unsigned int unknown_bits = uint16_t(*col>>16) ^ uint16_t(*col);
    unsigned int marked_bits = *col>>16;

    //check if the number of marked bits qualifies the col as marked
    if(!bitsBelowThreshold(marked_bits, marked_threshold)){
      return MARKED;
    }

    //check if the number of unkown bits qualifies the col as unknown
    if(!bitsBelowThreshold(unknown_bits, unknown_threshold))
      return UNKNOWN;

    return FREE;
  }

  unsigned int VoxelGrid::sizeX(){
    return size_x_;
  }

  unsigned int VoxelGrid::sizeY(){
    return size_y_;
  }

  unsigned int VoxelGrid::sizeZ(){
    return size_z_;
  }

  void VoxelGrid::printVoxelGrid(){
    for(unsigned int z = 0; z < size_z_; z++){
      printf("Layer z = %u:\n",z);
      for(unsigned int y = 0; y < size_y_; y++){
        for(unsigned int x = 0 ; x < size_x_; x++){
          printf((getVoxel(x, y, z)) == voxel_grid::MARKED? "#" : " ");
        }
        printf("|\n");
      } 
    }
  }

  void VoxelGrid::printColumnGrid(){
    printf("Column view:\n");
    for(unsigned int y = 0; y < size_y_; y++){
      for(unsigned int x = 0 ; x < size_x_; x++){
        printf((getVoxelColumn(x, y, 16, 0) == voxel_grid::MARKED)? "#" : " ");
      }
      printf("|\n");
    } 
  }
};
