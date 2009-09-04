cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(voxel_grid)

rospack_add_executable(bin/voxel_grid src/voxel_grid.cpp)
rospack_add_library(voxel_grid src/voxel_grid.cpp)
