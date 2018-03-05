cmake_minimum_required(VERSION 2.8.3)
project(omg_local_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  omg_ros_nav_bridge
  pluginlib
  roscpp
  std_msgs
  tf
  rosconsole
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES omg_local_planner
  CATKIN_DEPENDS geometry_msgs nav_msgs omg_ros_nav_bridge pluginlib roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(omg_local_planner src/omg_planner_ros.cpp)
add_dependencies(omg_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(omg_local_planner ${catkin_LIBRARIES})

install(TARGETS omg_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
