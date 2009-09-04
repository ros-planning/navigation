cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(fake_localization)
rospack_add_executable(fake_localization fake_localization.cpp)
