cmake_minimum_required(VERSION 2.8.3)
project(carrot_planner)

find_package(catkin REQUIRED
        COMPONENTS
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
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES carrot_planner
    CATKIN_DEPENDS
        roscpp
        pluginlib
        costmap_2d
        base_local_planner
        nav_core
)

add_library(carrot_planner src/carrot_planner.cpp)
add_dependencies(carrot_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(carrot_planner
    ${catkin_LIBRARIES}
    )

install(TARGETS carrot_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


