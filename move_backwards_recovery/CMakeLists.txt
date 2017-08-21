cmake_minimum_required(VERSION 2.8.3)
project(move_backwards_recovery)
add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  tf
  costmap_2d
  nav_core
  pluginlib
  base_local_planner
)

find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES move_backwards_recovery
  CATKIN_DEPENDS
    roscpp
    pluginlib
)

add_library(move_backwards_recovery src/move_backwards_recovery.cpp)
target_link_libraries(move_backwards_recovery ${catkin_LIBRARIES})

install(TARGETS move_backwards_recovery
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES move_backwards_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

