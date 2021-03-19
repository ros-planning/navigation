cmake_minimum_required(VERSION 3.0.2)
project(notify_surrounding_recovery)

find_package(catkin REQUIRED COMPONENTS
  lexxauto_msgs
  nav_core
  pluginlib
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES go_back_recovery
  CATKIN_DEPENDS
  lexxauto_msgs
  nav_core
  pluginlib
  roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(notify_surrounding_recovery src/notify_surrounding_recovery.cpp)
add_dependencies(notify_surrounding_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(notify_surrounding_recovery ${catkin_LIBRARIES})

install(TARGETS notify_surrounding_recovery
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES notify_surrounding_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
