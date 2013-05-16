cmake_minimum_required(VERSION 2.8.3)
project(move_base)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        roslib
        pluginlib
        actionlib
        dynamic_reconfigure
        message_generation
        nav_core
        tf
)
find_package(Eigen)
add_definitions(${EIGEN_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MoveBase.cfg
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        dynamic_reconfigure
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
#    ${EIGEN_INCLUDE_DIRS}
#    ${PCL_INCLUDE_DIRS}
#    ${Boost_INCLUDE_DIRS}
)

# move_base
add_library(move_base
  src/move_base.cpp
)
target_link_libraries(move_base
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(move_base move_base_gencfg)
add_dependencies(move_base geometry_msgs_gencpp)

add_executable(move_base_node
  src/move_base_node.cpp
)
target_link_libraries(move_base_node move_base)
set_target_properties(move_base_node PROPERTIES OUTPUT_NAME move_base)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )

install(
    PROGRAMS
       scripts/subtopic_forwarder.py
       scripts/subtopic_forwarder_node.py
       scripts/warner.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        move_base
        move_base_node
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
