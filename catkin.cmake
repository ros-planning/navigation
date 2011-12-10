cmake_minimum_required(VERSION 2.8)
project(navigation)

find_package(catkin REQUIRED)
find_package(ROS COMPONENTS
  rostime cpp_common roscpp_serialization roscpp_traits # serialization
  roscpp rosconsole                                     # roscpp
  nav_msgs std_msgs                                     # messages
  std_srvs tf angles message_filters dynamic_reconfigure) # other


add_subdirectory(amcl)

catkin_package(navigation)