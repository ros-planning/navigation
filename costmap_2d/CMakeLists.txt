cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()
rosbuild_genmsg()

#rosbuild_add_boost_directories()

rosbuild_add_library(costmap_2d src/costmap_2d.cpp src/observation_buffer.cpp src/costmap_2d_ros.cpp src/costmap_2d_publisher.cpp src/voxel_costmap_2d.cpp)
#rosbuild_link_boost(costmap_2d thread)

rosbuild_add_executable(bin/costmap_2d_markers src/costmap_2d_markers.cpp)
rosbuild_add_executable(bin/costmap_2d_cloud src/costmap_2d_cloud.cpp)

rosbuild_add_executable(bin/costmap_2d_node src/costmap_2d_node.cpp)
target_link_libraries(bin/costmap_2d_node costmap_2d)

rosbuild_add_executable(test/costmap_tester test/costmap_tester.cpp)
target_link_libraries(test/costmap_tester costmap_2d gtest)

rosbuild_download_test_data(http://pr.willowgarage.com/data/costmap_2d/simple_driving_test.bag test/simple_driving_test.bag 80a89edf83cd6d72f806db11121c4a2d)
rosbuild_download_test_data(http://pr.willowgarage.com/data/costmap_2d/willow-full-0.025.pgm test/willow-full-0.025.pgm e66b17ee374f2d7657972efcb3e2e4f7)

#rosbuild_add_rostest(test/simple_driving_test.xml)

rosbuild_add_gtest(test/module_tests test/module_tests.cpp)
target_link_libraries(test/module_tests costmap_2d)
