cmake_minimum_required(VERSION 2.8.3)
project(rotate_recovery)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            costmap_2d
            nav_core
            pluginlib
            base_local_planner
        )

find_package(Eigen REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES rotate_recovery
    CATKIN_DEPENDS
        roscpp
        pluginlib
)

add_library(rotate_recovery src/rotate_recovery.cpp)
target_link_libraries(rotate_recovery ${catkin_LIBRARIES})

install(TARGETS rotate_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES rotate_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

