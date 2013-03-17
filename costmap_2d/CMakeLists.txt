cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()

find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS}
                           ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

#rosbuild_add_boost_directories()

rosbuild_add_library(costmap_2d src/costmap_2d.cpp src/observation_buffer.cpp src/layered_costmap.cpp src/costmap_2d_ros.cpp src/costmap_2d_publisher.cpp)
target_link_libraries(costmap_2d ${PCL_LIBRARIES})
#rosbuild_link_boost(costmap_2d thread)

rosbuild_add_library(footprint_costmap_plugin plugins/footprint_costmap_plugin.cpp)
rosbuild_add_library(inflation_costmap_plugin plugins/inflation_costmap_plugin.cpp)
rosbuild_add_library( obstacle_costmap_plugin  plugins/obstacle_costmap_plugin.cpp src/observation_buffer.cpp)
rosbuild_add_library(   static_costmap_plugin    plugins/static_costmap_plugin.cpp)


rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_executable(bin/costmap_2d_markers src/costmap_2d_markers.cpp)
rosbuild_add_executable(bin/costmap_2d_cloud src/costmap_2d_cloud.cpp)

rosbuild_add_executable(bin/costmap_2d_node src/costmap_2d_node.cpp)
target_link_libraries(bin/costmap_2d_node costmap_2d)

#rosbuild_add_executable(test/costmap_tester test/costmap_tester.cpp)
#target_link_libraries(test/costmap_tester costmap_2d gtest ${PCL_LIBRARIES})

#rosbuild_download_test_data(http://pr.willowgarage.com/data/costmap_2d/simple_driving_test_indexed.bag test/simple_driving_test_indexed.bag 61168cff9425b11e093ea3a627c81c8d)
#rosbuild_download_test_data(http://pr.willowgarage.com/data/costmap_2d/willow-full-0.025.pgm test/willow-full-0.025.pgm e66b17ee374f2d7657972efcb3e2e4f7)

#rosbuild_add_rostest(test/simple_driving_test.xml)

#rosbuild_add_gtest(test/module_tests test/module_tests.cpp)
#target_link_libraries(test/module_tests costmap_2d)
