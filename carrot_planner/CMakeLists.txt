cmake_minimum_required(VERSION 2.8.3)
project(carrot_planner)

find_package(catkin REQUIRED
        COMPONENTS
            roslib
            roscpp
            tf
            nav_core
            costmap_2d
            base_local_planner
            pluginlib
        )

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    )
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES carrot_planner
    CATKIN_DEPENDS
        roslib
        roscpp
        pluginlib
        costmap_2d
        base_local_planner
        nav_core
)

add_library(carrot_planner src/carrot_planner.cpp)
target_link_libraries(carrot_planner
    ${catkin_LIBRARIES}
    )

install(TARGETS carrot_planner
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


