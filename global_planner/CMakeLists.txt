cmake_minimum_required(VERSION 2.8.3)
project(global_planner)

find_package(catkin REQUIRED
  COMPONENTS
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_core
  navfn
  nav_msgs
  pluginlib
  roscpp
  tf
)

generate_dynamic_reconfigure_options(
  cfg/GlobalPlanner.cfg
)

add_library(${PROJECT_NAME}
  src/quadratic_calculator.cpp
  src/dijkstra.cpp
  src/astar.cpp
  src/grid_path.cpp
  src/gradient_path.cpp
  src/planner_core.cpp
)

add_executable(planner
  src/plan_node.cpp
)
target_link_libraries(planner ${PROJECT_NAME})

install( TARGETS
  planner
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install( TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
