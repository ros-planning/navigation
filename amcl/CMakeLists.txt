cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()

rosbuild_add_library(amcl_pf 
                    src/pf/pf.c 
                    src/pf/pf_kdtree.c 
                    src/pf/pf_pdf.c 
                    src/pf/pf_vector.c
                    src/pf/eig3.c
                    src/pf/pf_draw.c)

rosbuild_add_library(amcl_map
                    src/map/map.c
                    src/map/map_range.c
                    src/map/map_store.c
                    src/map/map_draw.c)

rosbuild_add_library(amcl_sensors
                    src/sensors/amcl_sensor.cpp
                    src/sensors/amcl_odom.cpp
                    src/sensors/amcl_laser.cpp)
target_link_libraries(amcl_sensors amcl_map amcl_pf)

rosbuild_add_executable(bin/amcl
                       src/amcl_node.cpp)

target_link_libraries(bin/amcl amcl_sensors amcl_map amcl_pf)

## Tests are failing on OSX for an unknown reason
#include(CMakeDetermineSystem)
#if(CMAKE_SYSTEM_NAME MATCHES "Linux")

# Bags
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/basic_localization_stage.bag test/basic_localization_stage.bag 9c1b58fe73c423feaf664d4b3d4a996a)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/global_localization_stage.bag test/global_localization_stage.bag 9b3fafc304e4474fde62ff7fd68346f2)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/small_loop_prf.bag test/small_loop_prf.bag aa7f702fe99c3154c8bfba8a127b8536)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/small_loop_crazy_driving_prg.bag test/small_loop_crazy_driving_prg.bag d76668964d009174ffc8ac3050946496)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/texas_greenroom_loop.bag test/texas_greenroom_loop.bag ef11458b47693440e878f54ad99f6a6d)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/texas_willow_hallway_loop.bag test/texas_willow_hallway_loop.bag 6599a4d44c3ffc01310571e1f486cf1a)

# Maps
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/willow-full.pgm test/willow-full.pgm b84465cdbbfe3e2fb9eb4579e0bcaf0e)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/willow-full-0.05.pgm test/willow-full-0.05.pgm b61694296e08965096c5e78611fd9765)

# Tests
rosbuild_add_rostest(test/set_initial_pose.xml)
rosbuild_add_rostest(test/basic_localization_stage.xml)
rosbuild_add_rostest(test/small_loop_prf.xml)
rosbuild_add_rostest(test/small_loop_crazy_driving_prg.xml)
rosbuild_add_rostest(test/texas_greenroom_loop.xml)
rosbuild_add_rostest(test/texas_willow_hallway_loop.xml)
# Global localization needs some work.
#rosbuild_add_rostest(test/global_localization_stage.xml)
#endif(CMAKE_SYSTEM_NAME MATCHES "Linux")
