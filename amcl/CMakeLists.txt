cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_library(amcl_pf 
                    src/pf/pf.c 
                    src/pf/pf_kdtree.c 
                    src/pf/pf_pdf.c 
                    src/pf/pf_vector.c
                    src/pf/eig3.c
                    src/pf/pf_draw.c)

rosbuild_add_library(amcl_map
                    src/map/map.c
                    src/map/map_cspace.cpp
                    src/map/map_range.c
                    src/map/map_store.c
                    src/map/map_draw.c)

rosbuild_add_library(amcl_sensors
                    src/sensors/amcl_sensor.cpp
                    src/sensors/amcl_odom.cpp
                    src/sensors/amcl_laser.cpp)
target_link_libraries(amcl_sensors amcl_map amcl_pf)

rosbuild_add_boost_directories()
rosbuild_add_executable(bin/amcl
                       src/amcl_node.cpp)
rosbuild_link_boost(bin/amcl signals)

target_link_libraries(bin/amcl amcl_sensors amcl_map amcl_pf)

## Tests are failing on OSX for an unknown reason
#include(CMakeDetermineSystem)
#if(CMAKE_SYSTEM_NAME MATCHES "Linux")

# Bags
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/basic_localization_stage_indexed.bag test/basic_localization_stage_indexed.bag 41fe43af189ec71e5e48eb9ed661a655)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/global_localization_stage_indexed.bag test/global_localization_stage_indexed.bag 752f711cf4f6e8d1d660675e2da096b0)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/small_loop_prf_indexed.bag test/small_loop_prf_indexed.bag e4ef0fc006872b43f12ed8a7ce7dcd81)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/small_loop_crazy_driving_prg_indexed.bag test/small_loop_crazy_driving_prg_indexed.bag 4a58d1a7962914009d99000d06e5939c)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/texas_greenroom_loop_indexed.bag test/texas_greenroom_loop_indexed.bag 6e3432115cccdca1247f6c807038e13d)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/texas_willow_hallway_loop_indexed.bag test/texas_willow_hallway_loop_indexed.bag 27deb742fdcd3af44cf446f39f2688a8)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/rosie_localization_stage.bag test/rosie_localization_stage.bag 3347bf3835724cfa45e958c5c1846066)

# Maps
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/willow-full.pgm test/willow-full.pgm b84465cdbbfe3e2fb9eb4579e0bcaf0e)
rosbuild_download_test_data(http://pr.willowgarage.com/data/amcl/willow-full-0.05.pgm test/willow-full-0.05.pgm b61694296e08965096c5e78611fd9765)

# Tests
rosbuild_add_rostest(test/set_initial_pose.xml)
rosbuild_add_rostest(test/basic_localization_stage.xml)
rosbuild_add_rostest(test/small_loop_prf.xml)
rosbuild_add_rostest(test/small_loop_crazy_driving_prg.xml)
rosbuild_add_rostest(test/texas_greenroom_loop.xml)
rosbuild_add_rostest(test/rosie_multilaser.xml)
#rosbuild_add_rostest(test/texas_willow_hallway_loop.xml)
# Global localization needs some work.
#rosbuild_add_rostest(test/global_localization_stage.xml)
#endif(CMAKE_SYSTEM_NAME MATCHES "Linux")
