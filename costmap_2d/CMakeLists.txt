cmake_minimum_required(VERSION 2.8.3)
project(costmap_2d)

find_package(catkin REQUIRED
        COMPONENTS
            roslib
            roscpp
            rostest
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
    VoxelGrid.msg
)

generate_messages(
    DEPENDENCIES
        std_msgs
        geometry_msgs
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/Costmap2D.cfg
  cfg/ObstaclePlugin.cfg
  cfg/GenericPlugin.cfg
  cfg/InflationPlugin.cfg
  cfg/VoxelPlugin.cfg
)

add_library(costmap_2d
  src/array_parser.cpp
  src/costmap_2d.cpp
  src/observation_buffer.cpp
  src/layer.cpp
  src/layered_costmap.cpp
  src/costmap_2d_ros.cpp
  src/costmap_2d_publisher.cpp
  src/costmap_math.cpp
  src/footprint.cpp
)
add_dependencies(costmap_2d geometry_msgs_gencpp)
target_link_libraries(costmap_2d
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(layers
  plugins/footprint_layer.cpp
  plugins/inflation_layer.cpp
  plugins/obstacle_layer.cpp
  plugins/static_layer.cpp
  plugins/voxel_layer.cpp
  src/observation_buffer.cpp
)
target_link_libraries(layers
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
add_dependencies(layers costmap_2d_gencfg)
add_dependencies(costmap_2d costmap_2d_gencpp)

add_executable(costmap_2d_markers src/costmap_2d_markers.cpp)
add_dependencies(costmap_2d_markers visualization_msgs_gencpp)
target_link_libraries(costmap_2d_markers
    costmap_2d
    )

add_executable(costmap_2d_cloud src/costmap_2d_cloud.cpp)
add_dependencies(costmap_2d_cloud sensor_msgs_gencpp)
target_link_libraries(costmap_2d_cloud
    costmap_2d
    )

add_executable(costmap_2d_node src/costmap_2d_node.cpp)
target_link_libraries(costmap_2d_node
    costmap_2d
    )

add_executable(costmap_tester test/costmap_tester.cpp)
target_link_libraries(costmap_tester
    gtest
    costmap_2d
    )

include_directories(test)

add_gtest(array_parser_test test/array_parser_test.cpp)
target_link_libraries(array_parser_test costmap_2d gtest)

add_executable(footprint_tests test/footprint_tests.cpp)
target_link_libraries(footprint_tests gtest costmap_2d)
add_rostest(test/footprint_tests.launch)

add_executable(obstacle_tests test/obstacle_tests.cpp)
target_link_libraries(obstacle_tests
    costmap_2d gtest layers
    )


add_executable(static_tests test/static_tests.cpp)
target_link_libraries(static_tests
    costmap_2d gtest layers
    )

add_executable(inflation_tests test/inflation_tests.cpp)
target_link_libraries(inflation_tests
    costmap_2d gtest layers
    )

download_test_data(http://pr.willowgarage.com/data/costmap_2d/simple_driving_test_indexed.bag test/simple_driving_test_indexed.bag 61168cff9425b11e093ea3a627c81c8d)
download_test_data(http://pr.willowgarage.com/data/costmap_2d/willow-full-0.025.pgm test/willow-full-0.025.pgm e66b17ee374f2d7657972efcb3e2e4f7)
 
add_rostest(test/simple_driving_test.xml)
add_rostest(test/static_tests.launch)
add_rostest(test/obstacle_tests.launch)
add_rostest(test/inflation_tests.launch)

install( TARGETS
    costmap_2d_markers
    costmap_2d_cloud
    costmap_2d_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    costmap_2d
    layers
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
