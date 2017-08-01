cmake_minimum_required(VERSION 2.8.3)
project(base_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
            angles
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            geometry_msgs
            message_generation
            nav_core
            nav_msgs
            pcl_conversions
            pcl_ros
            pluginlib
            roscpp
            rospy
            std_msgs
            tf
            voxel_grid
        )

find_package(Boost REQUIRED
    COMPONENTS
        thread
        )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_python_setup()

# messages
add_message_files(
    DIRECTORY msg
    FILES
    Position2DInt.msg
)

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
        trajectory_planner_ros
    CATKIN_DEPENDS
        angles
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        message_runtime
        nav_core
        nav_msgs
        pluginlib
        roscpp
        std_msgs
        tf
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
	src/twirling_cost_function.cpp
	src/voxel_grid_model.cpp)
add_dependencies(base_local_planner base_local_planner_gencfg)
add_dependencies(base_local_planner base_local_planner_generate_messages_cpp)
add_dependencies(base_local_planner nav_msgs_generate_messages_cpp)
target_link_libraries(base_local_planner
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen_LIBRARIES}
    )

add_library(trajectory_planner_ros
	src/trajectory_planner.cpp
	src/trajectory_planner_ros.cpp)
add_dependencies(trajectory_planner_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_planner_ros
     base_local_planner)

add_executable(point_grid src/point_grid.cpp)
add_dependencies(point_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_grid ${catkin_LIBRARIES})

install(TARGETS
            base_local_planner
            trajectory_planner_ros
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
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
endif()
