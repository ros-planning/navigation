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

## Tests are failing on 64-bit for an unknown reason
#include(CMakeDetermineSystem)
#
#if(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
   #CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
   #CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")

rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/basic_localization_stage.bag test/basic_localization_stage.bag)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/global_localization_stage.bag test/global_localization_stage.bag)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/small_loop_prf.bag test/small_loop_prf.bag)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/small_loop_crazy_driving_prg.bag test/small_loop_crazy_driving_prg.bag)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/willow-full.pgm test/willow-full.pgm)

rosbuild_add_rostest(test/basic_localization_stage.xml)
rosbuild_add_rostest(test/small_loop_prf.xml)
rosbuild_add_rostest(test/small_loop_crazy_driving_prg.xml)
# Global localization needs some work.
#rosbuild_add_rostest(test/global_localization_stage.xml)
#endif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
      #CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
      #CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")
