cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()
rosbuild_genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_boost_directories()

rosbuild_add_executable(robot_pose_ekf 
                       src/odom_estimation.cpp 
                       src/nonlinearanalyticconditionalgaussianodo.cpp 
                       src/odom_estimation_node.cpp)
rosbuild_link_boost(robot_pose_ekf thread)

rosbuild_add_executable(test_robot_pose_ekf 
                       test/test_robot_pose_ekf.cpp)
rosbuild_link_boost(test_robot_pose_ekf thread)
rosbuild_add_gtest_build_flags(test_robot_pose_ekf)

rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_pose_ekf.launch)

# Download needed data file
rosbuild_download_test_data(http://pr.willowgarage.com/data/robot_pose_ekf/ekf_test.bag test/ekf_test.bag)
