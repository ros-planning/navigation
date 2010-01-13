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
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/basic_localization_stage.bag test/basic_localization_stage.bag 16735ad0261cbd67490dcef778e19db7)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/global_localization_stage.bag test/global_localization_stage.bag 623e80b6e7005d6e0acf3124a5d9b82e)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/small_loop_prf.bag test/small_loop_prf.bag c7f330c742f0103f0737ebc5c7714afe)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/small_loop_crazy_driving_prg.bag test/small_loop_crazy_driving_prg.bag 48fc62e04e10a35146a143451cb41f2f)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/texas_greenroom_loop.bag test/texas_greenroom_loop.bag f8677cd74f817cc0fe5a205009fa6ea4)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/texas_willow_hallway_loop.bag test/texas_willow_hallway_loop.bag e3e32f59a0c628c3dc0075ed3e235407)

# Maps
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/willow-full.pgm test/willow-full.pgm b84465cdbbfe3e2fb9eb4579e0bcaf0e)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/willow-full-0.05.pgm test/willow-full-0.05.pgm b61694296e08965096c5e78611fd9765)

# Tests
rosbuild_add_rostest(test/set_initial_pose.xml)
rosbuild_add_rostest(test/basic_localization_stage.xml)
rosbuild_add_rostest(test/small_loop_prf.xml)
rosbuild_add_rostest(test/small_loop_crazy_driving_prg.xml)
rosbuild_add_rostest(test/texas_greenroom_loop.xml)
#rosbuild_add_rostest(test/texas_willow_hallway_loop.xml)
# Global localization needs some work.
#rosbuild_add_rostest(test/global_localization_stage.xml)
#endif(CMAKE_SYSTEM_NAME MATCHES "Linux")
