cmake_minimum_required(VERSION 3.0.2)
project(costmap_3d)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  actionlib_msgs
  pluginlib
  roscpp
  geometry_msgs
  octomap_msgs
  tf_conversions
  message_generation
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OCTOMAP REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(fcl REQUIRED)

# These need to be set in this way for catkin_package so that they can be
# exported to client libraries.
get_target_property(LIBFCL_INCLUDE_DIRS fcl INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(LIBFCL_LIBRARIES fcl INTERFACE_LINK_LIBRARIES)
list(APPEND LIBFCL_LIBRARIES fcl)

include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${OCTOMAP_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/Costmap3D.cfg
  cfg/CylinderClearingPlugin.cfg
  cfg/GenericPlugin.cfg
)

add_action_files(
  FILES
  GetPlanCost3D.action
)

add_service_files(
  FILES
  GetPlanCost3DService.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    costmap_3d
    layers_3d
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    pluginlib
    roscpp
    geometry_msgs
    octomap_msgs
    pcl_ros
    pcl_msgs
    pcl_conversions
    tf2_ros
    visualization_msgs
  DEPENDS
    OCTOMAP
    PCL
    LIBFCL
)

add_library(costmap_3d
  src/costmap_3d.cpp
  src/layer_3d.cpp
  src/layered_costmap_3d.cpp
  src/costmap_3d_ros.cpp
  src/costmap_3d_query.cpp
  src/costmap_3d_publisher.cpp
  src/costmap_layer_3d.cpp
)

target_link_libraries(costmap_3d
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  fcl
  ${catkin_LIBRARIES}
)
target_compile_options(costmap_3d PRIVATE ${EXTRA_COMPILE_OPTIONS})

add_library(layers_3d
  plugins/octomap_server_layer_3d.cpp
  plugins/point_cloud_layer_3d.cpp
  plugins/costmap_3d_to_2d_layer.cpp
  plugins/costmap_3d_to_2d_layer_3d.cpp
  plugins/static_2d_layer_3d.cpp
  plugins/cylinder_clearing_layer_3d.cpp
)
add_dependencies(layers_3d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(layers_3d
  costmap_3d
)
target_compile_options(layers_3d PRIVATE ${EXTRA_COMPILE_OPTIONS})

add_dependencies(costmap_3d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(layers_3d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(costmap_3d_node src/costmap_3d_node.cpp)
add_dependencies(costmap_3d_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(costmap_3d_node
    costmap_3d
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
target_compile_options(costmap_3d_node PRIVATE ${EXTRA_COMPILE_OPTIONS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_octree_solver test/test_octree_solver.cpp)
  target_link_libraries(test_octree_solver costmap_3d ${catkin_LIBRARIES})
endif()

install(TARGETS
    costmap_3d_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    costmap_3d
    layers_3d
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
  costmap_2d_plugins.xml
  costmap_3d_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
