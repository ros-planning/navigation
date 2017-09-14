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
#include <voxel_grid/plain_clearer.h>
#include <voxel_grid/cache_clearer.h>
#include <gtest/gtest.h>


void get_counts(voxel_grid::VoxelGrid &vg, unsigned int &mark_count,
		unsigned int &free_count, unsigned int &unknown_count)
{
  for(unsigned int i = 0; i < vg.sizeX(); ++i){
    for(unsigned int j = 0; j < vg.sizeY(); ++j){
      for(unsigned int k = 0; k < vg.sizeZ(); ++k){
        if(vg.getVoxel(i, j, k) == voxel_grid::MARKED){
          mark_count++;
        }
        else if(vg.getVoxel(i, j, k) == voxel_grid::FREE){
          free_count++;
        }
        else if(vg.getVoxel(i, j, k) == voxel_grid::UNKNOWN){
          unknown_count++;
        }
      }
    }
  }
}

TEST(voxel_grid, basicMarkingAndClearing){
  int size_x = 50, size_y = 10, size_z = 16;
  voxel_grid::VoxelGrid vg(size_x, size_y, size_z);

  //Put a "tabletop" into the scene.  A flat rectangle of set voxels at z = 12.
  int table_z = 12;
  int table_x_min = 5, table_x_max = 15;
  int table_y_min = 0, table_y_max = 3;
  for(int x = table_x_min; x <= table_x_max; x++){
    vg.markVoxelLine(x, table_y_min, table_z, x, table_y_max, table_z);
  }

  for(int i = table_x_min; i <= table_x_max; ++i){
    for(int j = table_y_min; j <= table_y_max; ++j){
      //check that each cell of the table is marked
      ASSERT_EQ(voxel_grid::MARKED, vg.getVoxel(i, j, table_z));
    }
  }
  std::cout<<"Adding square at z=12"<<std::endl;
  vg.printVoxelGrid();
  vg.printColumnGrid();

  unsigned int mark_count = 0;
  unsigned int free_count = 0;
  unsigned int unknown_count = 0;
  //go through each cell in the voxel grid and make sure that only 44 are filled in
  get_counts(vg, mark_count, free_count, unknown_count);

  ASSERT_EQ(mark_count, 44);

  //the rest of the cells should be unknown
  ASSERT_EQ(unknown_count, vg.sizeX() * vg.sizeY() * vg.sizeZ() - 44);

  //now, let's clear one of the rows of the table
  vg.clearVoxelLine(table_x_min, table_y_min, table_z, table_x_max, table_y_min, table_z);
  std::cout<<"Clear out a row"<<std::endl;
  vg.printColumnGrid();

  mark_count = 0;
  unknown_count = 0;
  free_count = 0;
  //go through each cell in the voxel grid and make sure that only 33 are now filled in
  get_counts(vg, mark_count, free_count, unknown_count);

  //we've switched 11 cells from marked to free
  ASSERT_EQ(mark_count, 33);

  //we've just explicitly seen through 11 cells
  ASSERT_EQ(free_count, 11);

  //the rest of the cells should still be unknown
  ASSERT_EQ(unknown_count, vg.sizeX() * vg.sizeY() * vg.sizeZ() - 44);

  //now let's put in a vertical column manually to test markVoxel
  for(unsigned int i = 0; i < vg.sizeZ(); ++i){
    vg.markVoxel(0, 0, i);
    ASSERT_EQ(vg.getVoxel(0, 0, i), voxel_grid::MARKED);
  }

  std::cout<<"Add column along z"<<std::endl;
  vg.printVoxelGrid();
  vg.printColumnGrid();

  //now, let's clear that line of voxels and make sure that they clear out OK
  vg.clearVoxelLine(0, 0, 0, 0, 0, vg.sizeZ() - 1);
  std::cout<<"Clear the column"<<std::endl;
  vg.printVoxelGrid();
  vg.printColumnGrid();

  for(unsigned int i = 0; i < vg.sizeZ(); ++i){
    ASSERT_EQ(vg.getVoxel(0, 0, i), voxel_grid::FREE);
  }

}

TEST(voxel_grid, plainAndCornerClearing){
  int size_x = 3, size_y = 3, size_z = 16;
  int cache_width = size_x;
  voxel_grid::VoxelGrid vg(size_x, size_y, size_z);
  voxel_grid::VoxelGrid vg_plain_cache(size_x, size_y, size_z);
  voxel_grid::VoxelGrid vg_corner_cache(size_x, size_y, size_z);
  unsigned char costmap[size_x * size_y];
  voxel_grid::PlainClearer plain(vg.getData(), costmap, 1, 1);
  voxel_grid::CacheClearer plain_cache(vg_plain_cache.getData(), costmap, size_x,
				       size_y, cache_width, 1, 1, false);
  voxel_grid::CacheClearer corner_cache(vg_corner_cache.getData(), costmap, size_x,
					size_y, cache_width, 1, 1, true);

  // Put a simple cube in the scene
  int side_len = size_x - 1;
  int x_min = 0, x_max = side_len;
  int y_min = 0, y_max = side_len;
  int z_min = 12, z_max = z_min + side_len;
  for(int x = x_min; x <= x_max; ++x){
    for(int z = z_min; z <= z_max; ++z){
      vg.markVoxelLine(x, y_min, z, x, y_max, z);
      vg_plain_cache.markVoxelLine(x, y_min, z, x, y_max, z);
      vg_corner_cache.markVoxelLine(x, y_min, z, x, y_max, z);
    }
  }
  std::cout<<"Adding cube at z=12"<<std::endl;
  vg.printVoxelGrid();
  vg.printColumnGrid();

  unsigned int mark_count = 0;
  unsigned int free_count = 0;
  unsigned int unknown_count = 0;
  //go through each cell in the voxel grid and make sure that 27 are filled in
  get_counts(vg, mark_count, free_count, unknown_count);
  ASSERT_EQ(mark_count, 27);
  ASSERT_EQ(free_count, 0);
  ASSERT_EQ(unknown_count, vg.sizeX() * vg.sizeY() * vg.sizeZ() - 27);

  // simple clearing along a diagonal 3d ray
  vg.clearVoxelLineInMap(x_min, y_min, z_min, x_max, y_max, z_max, &plain);
  std::cout<<"Clear diagonal ray through cube"<<std::endl;
  vg.printVoxelGrid();
  vg.printColumnGrid();
  mark_count = 0;
  free_count = 0;
  unknown_count = 0;
  get_counts(vg, mark_count, free_count, unknown_count);
  ASSERT_EQ(mark_count, 24);
  ASSERT_EQ(free_count, 3);
  ASSERT_EQ(unknown_count, vg.sizeX() * vg.sizeY() * vg.sizeZ() - 27);

  // cache clearing without corners should be the same
  vg_plain_cache.clearVoxelLineInMap(x_min, y_min, z_min, x_max, y_max, z_max,
				     &plain_cache);
  plain_cache.update(); // applies cached masks to the costmap
  std::cout<<"Clear diagonal ray through original cube using cache"<<std::endl;
  vg_plain_cache.printVoxelGrid();
  vg_plain_cache.printColumnGrid();
  mark_count = 0;
  free_count = 0;
  unknown_count = 0;
  get_counts(vg_plain_cache, mark_count, free_count, unknown_count);
  ASSERT_EQ(mark_count, 24);
  ASSERT_EQ(free_count, 3);
  ASSERT_EQ(unknown_count, vg_plain_cache.sizeX() *
	    vg_plain_cache.sizeY() * vg_plain_cache.sizeZ() - 27);

  vg_corner_cache.clearVoxelLineInMap(x_min, y_min, z_min, x_max, y_max, z_max,
				      &corner_cache);
  corner_cache.update();
  std::cout<<"Clear diagonal ray plus corners through original cube using cache"<<std::endl;
  vg_corner_cache.printVoxelGrid();
  vg_corner_cache.printColumnGrid();
  mark_count = 0;
  free_count = 0;
  unknown_count = 0;
  get_counts(vg_corner_cache, mark_count, free_count, unknown_count);
  ASSERT_EQ(mark_count, 12); // More aggressive clearing
  ASSERT_EQ(free_count, 15);
  ASSERT_EQ(unknown_count, vg_corner_cache.sizeX() *
	    vg_corner_cache.sizeY() * vg_corner_cache.sizeZ() - 27);

}

int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
