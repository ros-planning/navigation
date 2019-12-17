cmake_minimum_required(VERSION 2.8.3)
project(dead_reckoning_controller)
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
        COMPONENTS
            angles
            base_local_planner
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            pcl_conversions
            pluginlib
            roscpp
            srslib_timing
            tf
        )

find_package(Boost REQUIRED
    COMPONENTS
        thread
        )

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED QUIET COMPONENTS io)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})


# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/DeadReckoningController.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES dead_reckoning_controller
    CATKIN_DEPENDS
    base_local_planner
        roscpp
        dynamic_reconfigure
        message_generation
        tf
        pluginlib
        costmap_2d
        nav_core
        angles
)

add_library(dead_reckoning_controller src/dead_reckoning_controller_ros.cpp src/dead_reckoning_controller.cpp)
target_link_libraries(dead_reckoning_controller ${catkin_LIBRARIES})
add_dependencies(dead_reckoning_controller ${catkin_EXPORTED_TARGETS})


install(TARGETS dead_reckoning_controller
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

