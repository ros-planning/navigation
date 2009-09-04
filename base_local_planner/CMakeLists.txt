cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(base_local_planner)
genmsg()

rospack_add_boost_directories()

#uncomment for profiling
#set(ROS_COMPILE_FLAGS "-g -pg" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g -pg" ${ROS_LINK_FLAGS})
#set(ROS_COMPILE_FLAGS "-g" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g" ${ROS_LINK_FLAGS})

rospack_add_library(base_local_planner src/map_cell.cpp src/map_grid.cpp src/point_grid.cpp src/costmap_model.cpp src/voxel_grid_model.cpp src/trajectory_planner.cpp 
    src/trajectory.cpp src/trajectory_planner_ros.cpp)
rospack_link_boost(base_local_planner thread)

rospack_add_executable(point_grid src/point_grid.cpp)

rospack_add_gtest(test/utest test/utest.cpp)
target_link_libraries(test/utest base_local_planner)
