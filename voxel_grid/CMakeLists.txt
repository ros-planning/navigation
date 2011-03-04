cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()

rosbuild_add_library(voxel_grid src/voxel_grid.cpp)

rosbuild_add_executable(bin/voxel_grid src/voxel_grid_main.cpp)
target_link_libraries(bin/voxel_grid voxel_grid)

rosbuild_add_gtest(test/voxel_grid_tests test/voxel_grid_tests.cpp)
target_link_libraries(test/voxel_grid_tests voxel_grid)
