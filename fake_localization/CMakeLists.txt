cmake_minimum_required(VERSION 3.0.2)
project(fake_localization)

find_package(catkin REQUIRED
        COMPONENTS
            angles
            geometry_msgs
            message_filters
            nav_msgs
            rosconsole
            roscpp
            rospy
            tf2_geometry_msgs
            tf2_ros
        )


find_package(Boost REQUIRED)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        nav_msgs
        roscpp
        rospy
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(fake_localization fake_localization.cpp)
target_link_libraries(fake_localization
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )
add_dependencies(fake_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(
    PROGRAMS
       static_odom_broadcaster.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        fake_localization
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
