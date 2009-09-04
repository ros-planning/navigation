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

namespace voxel_grid {
  VoxelGrid::VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z)
  {
    size_x_ = size_x; 
    size_y_ = size_y; 
    size_z_ = size_z; 

    if(size_z_ > 16)
      ROS_INFO("Error, this implementation can only support up to 16 z values"); 

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

  void VoxelGrid::markVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length){
    if(x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1>=size_x_ || y1>=size_y_ || z1>=size_z_){
      ROS_INFO("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)", x0, y0, z0, x1, y1, z1, 
          size_x_, size_y_, size_z_);
      return;
    }

    MarkVoxel mv(data_);
    raytraceLine(mv, x0, y0, z0, x1, y1, z1, max_length);
  }

  void VoxelGrid::clearVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length){
    if(x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1>=size_x_ || y1>=size_y_ || z1>=size_z_){
      ROS_INFO("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)", x0, y0, z0, x1, y1, z1, 
          size_x_, size_y_, size_z_);
      return;
    }

    ClearVoxel cv(data_);
    raytraceLine(cv, x0, y0, z0, x1, y1, z1, max_length);
  }

  void VoxelGrid::clearVoxelLineInMap(double x0, double y0, double z0, double x1, double y1, double z1, unsigned char *map_2d, 
      unsigned int unknown_threshold, unsigned int mark_threshold, unsigned int max_length){
    costmap = map_2d;
    if(map_2d == NULL){
      clearVoxelLine(x0, y0, z0, x1, y1, z1, max_length);
      return;
    }

    if(x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1>=size_x_ || y1>=size_y_ || z1>=size_z_){
      ROS_INFO("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)", x0, y0, z0, x1, y1, z1, 
          size_x_, size_y_, size_z_);
      return;
    }

    ClearVoxelInMap cvm(data_, costmap, unknown_threshold, mark_threshold);
    raytraceLine(cvm, x0, y0, z0, x1, y1, z1, max_length);
  }

  VoxelStatus VoxelGrid::getVoxel(unsigned int x, unsigned int y, unsigned int z)
  {
    if(x >= size_x_ || y >= size_y_ || z >= size_z_){
      ROS_INFO("Error, voxel out of bounds. (%d, %d, %d)\n", x, y, z);
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
      ROS_INFO("Error, voxel out of bounds. (%d, %d)\n", x, y);
      return UNKNOWN;
    }
    
    uint32_t* col = &data_[y * size_x_ + x];

    unsigned int unknown_bits = uint16_t(*col>>16) ^ uint16_t(*col);
    unsigned int marked_bits = *col>>16;

    //check if the number of unkown bits qualifies the col as unknown
    if(!bitsBelowThreshold(unknown_bits, unknown_threshold))
      return UNKNOWN;

    //check if the number of marked bits qualifies the col as marked
    if(!bitsBelowThreshold(marked_bits, marked_threshold)){
      return MARKED;
    }

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
      printf("Layer z = %d:\n",z);
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

  int main(int argc, char *argv[]){
    ROS_INFO("Initializing voxel grid.\n");
    //int size_x = 50, size_y = 10, size_z = 5;
    int size_x = 1000, size_y = 1000, size_z = 5;
    voxel_grid::VoxelGrid *v = new voxel_grid::VoxelGrid(size_x, size_y, size_z);

    unsigned char *costMap = new unsigned char[size_x * size_y]; //initialize cost map
    for(int x = 0; x < size_x; x++){
      for(int y = 0; y < size_y; y++){
        costMap[y * size_x + x] = 128;
      }
    }


    //Put a "tabletop" into the scene.  A flat rectangle of set voxels at z = 12.
    int table_z = 1;
    int table_x_min = 5, table_x_max = 15;
    int table_y_min = 0, table_y_max = 3;
    for(int x = table_x_min; x <= table_x_max; x++){
      v->markVoxelLine(x, table_y_min, table_z, x, table_y_max, table_z);
    }

    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    for(unsigned int j = 0; j < 1000; ++j){
      for(unsigned int i = 0; i < 700; ++i){
        //Add a few synthetic obstacles (diagonal line on the floor) just to demonstrate line drawing
        v->markVoxelLine(size_x - 1, size_y - 1, 0, 0, 0, 0);
      }
    }
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f\n", t_diff);

    //clear a scan next to the table (will clear obstacles out)
    v->clearVoxelLineInMap(table_x_max + 1, 0, table_z, table_x_max + 1, size_y - 1, table_z, costMap, 16, 0);

    //clear a scan over the table (will not clear obstacles out until it passes the edge of the table)
    v->clearVoxelLineInMap(table_x_min + 1, 0, table_z + 1, table_x_min + 1, size_y - 1, table_z, costMap, 16, 0);

    //clear a scan through the table (will only clear obstacles where it passes through the table
    v->clearVoxelLineInMap(table_x_min, table_y_min, 0, table_x_max, table_y_max, size_z - 1, costMap, 16, 0);

    //clear a scan that clears out a whole line of the table, to show that it doesnt clear out the diagonal line underneath it
    v->clearVoxelLineInMap(table_x_max - 2, 0, table_z, table_x_max - 2, size_y - 1, table_z, costMap, 16, 0);

    //Visualize the output
    /*
    v->printVoxelGrid();
    v->printColumnGrid();

    printf("CostMap:\n===========\n");
    for(int y = 0; y < size_y; y++){
      for(int x = 0; x < size_x; x++){
        printf((costMap[y * size_x + x] > 0 ? "#" : " "));
      }printf("|\n");
    }
    */
  }
