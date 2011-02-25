/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*********************************************************************/
#include <voxel_grid/voxel_grid.h>
#include <sys/time.h>
#include <ros/console.h>

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
