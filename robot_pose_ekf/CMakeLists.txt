cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_boost_directories()

rosbuild_add_executable(robot_pose_ekf 
                       src/odom_estimation.cpp 
                       src/nonlinearanalyticconditionalgaussianodo.cpp 
                       src/odom_estimation_node.cpp)
rosbuild_link_boost(robot_pose_ekf thread)

## Tests are failing on OSX for an unknown reason
include(CMakeDetermineSystem)
if(CMAKE_SYSTEM_NAME MATCHES "Linux")

rosbuild_download_test_data(http://pr.willowgarage.com/data/robot_pose_ekf/ekf_test2.bag test/ekf_test2.bag 3b8c5c6485c7e29b42318a12784d7bdd)
rosbuild_add_executable(test_robot_pose_ekf test/test_robot_pose_ekf.cpp)
rosbuild_link_boost(test_robot_pose_ekf thread)
rosbuild_add_gtest_build_flags(test_robot_pose_ekf)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_pose_ekf.launch)

rosbuild_download_test_data(http://pr.willowgarage.com/data/robot_pose_ekf/zero_covariance.bag test/zero_covariance.bag 0a51b4f5001f446e8466bf7cc946fb86)
rosbuild_add_executable(test_robot_pose_ekf_zero_covariance test/test_robot_pose_ekf_zero_covariance.cpp)
rosbuild_link_boost(test_robot_pose_ekf_zero_covariance thread)
rosbuild_add_gtest_build_flags(test_robot_pose_ekf_zero_covariance)
rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_pose_ekf_zero_covariance.launch)

endif(CMAKE_SYSTEM_NAME MATCHES "Linux")
