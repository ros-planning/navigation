cmake_minimum_required(VERSION 3.0.2)
project(go_back_recovery)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES go_back_recovery
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    geometry_msgs
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(go_back_recovery src/go_back_recovery.cpp)
add_dependencies(go_back_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(go_back_recovery ${catkin_LIBRARIES})

install(TARGETS go_back_recovery
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES go_back_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
