cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()
genmsg()

#rospack_add_boost_directories()

rospack_add_library(costmap_2d src/costmap_2d.cpp src/observation_buffer.cpp src/costmap_2d_ros.cpp src/costmap_2d_publisher.cpp src/voxel_costmap_2d.cpp)
#rospack_link_boost(costmap_2d thread)

rospack_add_executable(bin/costmap_2d_markers src/costmap_2d_markers.cpp)
rospack_add_executable(bin/costmap_2d_cloud src/costmap_2d_cloud.cpp)

rospack_add_executable(test/costmap_tester test/costmap_tester.cpp)
target_link_libraries(test/costmap_tester costmap_2d gtest)

rosbuild_download_test_data(http://pr.willowgarage.com/data/costmap_2d/simple_driving_test.bag test/simple_driving_test.bag)

rosbuild_add_rostest(test/simple_driving_test.xml)

rosbuild_add_gtest(test/module_tests test/module_tests.cpp)
target_link_libraries(test/module_tests costmap_2d)
