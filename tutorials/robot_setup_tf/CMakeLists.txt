cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(robot_setup_tf)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(tf_broadcaster src/tf_broadcaster.cpp)
rospack_add_executable(tf_listener src/tf_listener.cpp)
