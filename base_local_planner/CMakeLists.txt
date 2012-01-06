cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()
rosbuild_genmsg()

rosbuild_add_boost_directories()

#uncomment for profiling
#set(ROS_COMPILE_FLAGS "-g -pg" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g -pg" ${ROS_LINK_FLAGS})
#set(ROS_COMPILE_FLAGS "-g" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g" ${ROS_LINK_FLAGS})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

rosbuild_add_library(base_local_planner src/goal_functions.cpp src/map_cell.cpp src/map_grid.cpp src/point_grid.cpp src/costmap_model.cpp src/voxel_grid_model.cpp src/trajectory_planner.cpp 
    src/trajectory.cpp src/trajectory_planner_ros.cpp src/map_grid_visualizer.cpp)
rosbuild_link_boost(base_local_planner thread)

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_executable(point_grid src/point_grid.cpp)

rosbuild_add_gtest(test/utest test/utest.cpp)
target_link_libraries(test/utest base_local_planner)
