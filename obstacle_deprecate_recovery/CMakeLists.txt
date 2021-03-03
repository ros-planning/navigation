cmake_minimum_required(VERSION 3.0.2)
project(obstacle_deprecate_recovery)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  nav_core
  pluginlib
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES obstacle_deprecate_recovery
  CATKIN_DEPENDS
    base_local_planner
    nav_core
    pluginlib
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(obstacle_deprecate_recovery src/obstacle_deprecate_recovery.cpp)
add_dependencies(obstacle_deprecate_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_deprecate_recovery ${catkin_LIBRARIES})

install(TARGETS obstacle_deprecate_recovery
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install (FILES obstacle_deprecate_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
