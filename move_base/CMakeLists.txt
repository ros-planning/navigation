cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

find_package(Eigen)
find_package(PCL)
include_directories(${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_boost_directories()

# move_base
rosbuild_add_library(move_base src/move_base.cpp)
rosbuild_link_boost(move_base system)

rosbuild_add_executable(move_base_node src/move_base_node.cpp)
target_link_libraries(move_base_node move_base)
set_target_properties(move_base_node PROPERTIES OUTPUT_NAME bin/move_base)
