cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(footprint_costmap_plugin src/footprint_costmap_plugin.cpp)
rosbuild_add_library(inflation_costmap_plugin src/inflation_costmap_plugin.cpp)
rosbuild_add_library( obstacle_costmap_plugin  src/obstacle_costmap_plugin.cpp src/observation_buffer.cpp)
rosbuild_add_library(   static_costmap_plugin    src/static_costmap_plugin.cpp)


