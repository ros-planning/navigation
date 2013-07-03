cmake_minimum_required(VERSION 2.8.3)
project(dwa_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            roslib
            tf
            costmap_2d
            nav_core
            nav_msgs
            base_local_planner
            dynamic_reconfigure
            pluginlib
        )

find_package(Eigen)
find_package(PCL)
include_directories(
    include
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )
add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/DWAPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES dwa_local_planner
    CATKIN_DEPENDS
        roslib
        roscpp
        dynamic_reconfigure
        pluginlib
)

add_library(dwa_local_planner src/dwa_planner.cpp src/dwa_planner_ros.cpp)
target_link_libraries(dwa_local_planner base_local_planner ${catkin_LIBRARIES})
add_dependencies(dwa_local_planner dwa_local_planner_gencfg)
add_dependencies(dwa_local_planner nav_msgs_gencpp)

install(TARGETS dwa_local_planner
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

