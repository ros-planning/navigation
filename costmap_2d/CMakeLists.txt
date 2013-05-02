cmake_minimum_required(VERSION 2.8.3)
project(costmap_2d)

find_package(catkin REQUIRED
        COMPONENTS
            roslib
            roscpp
            std_msgs
            geometry_msgs
            tf
            laser_geometry
            dynamic_reconfigure
            message_generation
            voxel_grid
            pcl_ros
            pluginlib
        )

find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    SYSTEM
        ${EIGEN_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})

# messages
add_message_files(
    DIRECTORY msg
    FILES
    OccupancyGridUpdate.msg
    VoxelGrid.msg
)

generate_messages(
    DEPENDENCIES
        std_msgs
        geometry_msgs
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/ObstaclePlugin.cfg
  cfg/Costmap2D.cfg
  cfg/GenericPlugin.cfg
  cfg/InflationPlugin.cfg
  cfg/VoxelPlugin.cfg
)

add_library(costmap_2d
  src/costmap_2d.cpp
  src/observation_buffer.cpp
  src/layered_costmap.cpp
  src/costmap_2d_ros.cpp
  src/costmap_2d_publisher.cpp
  src/costmap_math.cpp
  src/footprint.cpp
)
target_link_libraries(costmap_2d
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(common_costmap_plugins
  plugins/footprint_costmap_plugin.cpp
  plugins/inflation_costmap_plugin.cpp
  plugins/obstacle_costmap_plugin.cpp
  plugins/static_costmap_plugin.cpp
  plugins/voxel_costmap_plugin.cpp
  plugins/voxel_with_footprint_costmap_plugin.cpp
  src/observation_buffer.cpp
)
target_link_libraries(common_costmap_plugins
  costmap_2d
)

catkin_package(
    INCLUDE_DIRS
        include
        ${EIGEN_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
    LIBRARIES costmap_2d
    CATKIN_DEPENDS
        roslib
        roscpp
        message_generation
        pcl_ros
        voxel_grid
    DEPENDS
        PCL
        Eigen

)

add_dependencies(costmap_2d costmap_2d_gencfg)
add_dependencies(common_costmap_plugins costmap_2d_gencfg)
add_dependencies(costmap_2d costmap_2d_gencpp)

add_executable(costmap_2d_markers src/costmap_2d_markers.cpp)
target_link_libraries(costmap_2d_markers
    costmap_2d
    )

add_executable(costmap_2d_cloud src/costmap_2d_cloud.cpp)
target_link_libraries(costmap_2d_cloud
    costmap_2d
    )

add_executable(costmap_2d_node src/costmap_2d_node.cpp)
target_link_libraries(costmap_2d_node
    costmap_2d
    )

# Not supported (yet?) after layered-costmap merge.  New layered API makes porting non-trivial. X
#
add_executable(costmap_tester test/costmap_tester.cpp)
target_link_libraries(costmap_tester
    gtest
    costmap_2d
    )

download_test_data(http://pr.willowgarage.com/data/costmap_2d/simple_driving_test_indexed.bag test/simple_driving_test_indexed.bag 61168cff9425b11e093ea3a627c81c8d)
download_test_data(http://pr.willowgarage.com/data/costmap_2d/willow-full-0.025.pgm test/willow-full-0.025.pgm e66b17ee374f2d7657972efcb3e2e4f7)
 
add_rostest(test/simple_driving_test.xml)

catkin_add_gtest(module_tests test/module_tests.cpp)
target_link_libraries(module_tests
    costmap_2d
    )


install( TARGETS
    costmap_2d_markers
    costmap_2d_cloud
    costmap_2d_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    costmap_2d
    common_costmap_plugins
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
