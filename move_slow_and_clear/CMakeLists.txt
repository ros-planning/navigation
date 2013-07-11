cmake_minimum_required(VERSION 2.8.3)
project(move_slow_and_clear)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        nav_core
        costmap_2d
        actionlib_msgs
        std_msgs
        pluginlib
        )


find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    SYSTEM
        ${EIGEN_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES move_slow_and_clear
    CATKIN_DEPENDS
        roscpp
        pluginlib
        nav_core
)

add_library(${PROJECT_NAME} src/move_slow_and_clear.cpp)
add_dependencies(${PROJECT_NAME} geometry_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE)

install(TARGETS move_slow_and_clear
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES recovery_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


