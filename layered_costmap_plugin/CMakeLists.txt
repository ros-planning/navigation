cmake_minimum_required(VERSION 2.8.3)
project(layered_costmap_plugin)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  costmap_2d
  pluginlib
)

catkin_package(
  LIBRARIES
  layered_costmap_plugin
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(layered_costmap_plugin
  src/layered_costmap_plugin.cpp
)
target_link_libraries(layered_costmap_plugin
  ${catkin_LIBRARIES}
)

install( TARGETS
  layered_costmap_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
