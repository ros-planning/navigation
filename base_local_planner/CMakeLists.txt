cmake_minimum_required(VERSION 2.8.3)
project(base_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
            roslib
            roscpp
            tf
            message_generation
            dynamic_reconfigure
            nav_core
            rostest
            costmap_2d
            pluginlib
        )

find_package(Boost REQUIRED
    COMPONENTS
        thread
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

catkin_python_setup()

# messages
add_message_files(
    DIRECTORY msg
    FILES
    Position2DInt.msg
)

catkin_python_setup()

generate_messages(
    DEPENDENCIES
        std_msgs
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/BaseLocalPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        base_local_planner
#       trajectory_planner_ros
    CATKIN_DEPENDS
        roslib
        roscpp
        dynamic_reconfigure
        message_generation
        tf
        pluginlib
        costmap_2d
        nav_core
)

#uncomment for profiling
#set(ROS_COMPILE_FLAGS "-g -pg" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g -pg" ${ROS_LINK_FLAGS})
#set(ROS_COMPILE_FLAGS "-g" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g" ${ROS_LINK_FLAGS})

add_library(base_local_planner 
	src/footprint_helper.cpp
	src/goal_functions.cpp
	src/map_cell.cpp
	src/map_grid.cpp
	src/map_grid_visualizer.cpp
	src/map_grid_cost_function.cpp
	src/latched_stop_rotate_controller.cpp
	src/local_planner_util.cpp
	src/odometry_helper_ros.cpp
	src/obstacle_cost_function.cpp
	src/oscillation_cost_function.cpp
	src/prefer_forward_cost_function.cpp
	src/point_grid.cpp
	src/costmap_model.cpp
	src/simple_scored_sampling_planner.cpp
	src/simple_trajectory_generator.cpp
	src/trajectory.cpp
	src/voxel_grid_model.cpp)
add_dependencies(base_local_planner base_local_planner_gencfg)
add_dependencies(base_local_planner base_local_planner_gencpp)
add_dependencies(base_local_planner nav_msgs_gencpp)
target_link_libraries(base_local_planner
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen_LIBRARIES}
    )

add_library(trajectory_planner_ros
	src/trajectory_planner.cpp
	src/trajectory_planner_ros.cpp)
add_dependencies(trajectory_planner_ros nav_msgs_gencpp)
target_link_libraries(trajectory_planner_ros
     base_local_planner)

add_executable(point_grid src/point_grid.cpp)
target_link_libraries(point_grid ${catkin_LIBRARIES})

install(TARGETS
            base_local_planner
            trajectory_planner_ros
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

catkin_add_gtest(base_local_planner_utest
	test/gtest_main.cpp
	test/utest.cpp
	test/velocity_iterator_test.cpp
	test/footprint_helper_test.cpp
	test/trajectory_generator_test.cpp
	test/map_grid_test.cpp)
target_link_libraries(base_local_planner_utest
    base_local_planner trajectory_planner_ros
    )


catkin_add_gtest(line_iterator
    test/line_iterator_test.cpp)
