cmake_minimum_required(VERSION 3.0.2)
project(rotate_recovery)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    base_local_planner
    cmake_modules
    costmap_2d
    geometry_msgs
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
)

find_package(Eigen3 REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rotate_recovery
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_ros
)

add_library(rotate_recovery src/rotate_recovery.cpp)
add_dependencies(rotate_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotate_recovery ${catkin_LIBRARIES})

install(TARGETS rotate_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES rotate_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

