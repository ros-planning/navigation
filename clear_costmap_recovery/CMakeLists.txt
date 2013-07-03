cmake_minimum_required(VERSION 2.8.3)
project(clear_costmap_recovery)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf
            costmap_2d
            nav_core
            pluginlib
        )

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
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
    LIBRARIES clear_costmap_recovery
    CATKIN_DEPENDS
        roscpp
        pluginlib
)

add_library(clear_costmap_recovery src/clear_costmap_recovery.cpp)
target_link_libraries(clear_costmap_recovery ${catkin_LIBRARIES})

install(TARGETS clear_costmap_recovery
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES ccr_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


