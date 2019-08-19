cmake_minimum_required(VERSION 3.1)
project(mel_amcl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    message_filters
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
)

find_package(Boost REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MEL_AMCL.cfg
)

catkin_package(
  CATKIN_DEPENDS
    diagnostic_updater
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_msgs
    tf2_ros
  INCLUDE_DIRS include
  LIBRARIES mel_amcl_sensors mel_amcl_map mel_amcl_pf
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(src/include)

check_include_file(unistd.h HAVE_UNISTD_H)
if (HAVE_UNISTD_H)
  add_definitions(-DHAVE_UNISTD_H)
endif (HAVE_UNISTD_H)

check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)
if (HAVE_DRAND48)
  add_definitions(-DHAVE_DRAND48)
endif (HAVE_DRAND48)

add_library(mel_amcl_pf
                    src/mel_amcl/pf/pf.c
                    src/mel_amcl/pf/pf_kdtree.c
                    src/mel_amcl/pf/pf_pdf.c
                    src/mel_amcl/pf/pf_vector.c
                    src/mel_amcl/pf/eig3.c
                    src/mel_amcl/pf/pf_draw.c)

add_library(mel_amcl_map
                    src/mel_amcl/map/map.c
                    src/mel_amcl/map/map_cspace.cpp
                    src/mel_amcl/map/map_range.c
                    src/mel_amcl/map/map_store.c
                    src/mel_amcl/map/map_draw.c)

add_library(mel_amcl_sensors
                    src/mel_amcl/sensors/mel_amcl_sensor.cpp
                    src/mel_amcl/sensors/mel_amcl_odom.cpp
                    src/mel_amcl/sensors/mel_amcl_laser.cpp
                    src/mel_amcl/sensors/mel_amcl_pose.cpp
                    )
target_link_libraries(mel_amcl_sensors mel_amcl_map mel_amcl_pf)


add_executable(mel_amcl
                       src/mel_amcl_node.cpp)
add_dependencies(mel_amcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mel_amcl
    mel_amcl_sensors mel_amcl_map mel_amcl_pf
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

install( TARGETS
    mel_amcl
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install( TARGETS
    mel_amcl_sensors mel_amcl_map mel_amcl_pf
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/mel_amcl/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY examples/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
)

## Configure Tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Bags
  catkin_download_test_data(${PROJECT_NAME}_basic_localization_stage_indexed.bag
    http://download.ros.org/data/amcl/basic_localization_stage_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 41fe43af189ec71e5e48eb9ed661a655)
  catkin_download_test_data(${PROJECT_NAME}_global_localization_stage_indexed.bag
    http://download.ros.org/data/amcl/global_localization_stage_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 752f711cf4f6e8d1d660675e2da096b0)
  catkin_download_test_data(${PROJECT_NAME}_small_loop_prf_indexed.bag
    http://download.ros.org/data/amcl/small_loop_prf_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 e4ef0fc006872b43f12ed8a7ce7dcd81)
  catkin_download_test_data(${PROJECT_NAME}_small_loop_crazy_driving_prg_indexed.bag
    http://download.ros.org/data/amcl/small_loop_crazy_driving_prg_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 4a58d1a7962914009d99000d06e5939c)
  catkin_download_test_data(${PROJECT_NAME}_texas_greenroom_loop_indexed.bag
    http://download.ros.org/data/amcl/texas_greenroom_loop_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 6e3432115cccdca1247f6c807038e13d)
  catkin_download_test_data(${PROJECT_NAME}_texas_willow_hallway_loop_indexed.bag
    http://download.ros.org/data/amcl/texas_willow_hallway_loop_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 27deb742fdcd3af44cf446f39f2688a8)
  catkin_download_test_data(${PROJECT_NAME}_rosie_localization_stage.bag
    http://download.ros.org/data/amcl/rosie_localization_stage.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 3347bf3835724cfa45e958c5c1846066)

  # Maps
  catkin_download_test_data(${PROJECT_NAME}_willow-full.pgm
    http://download.ros.org/data/amcl/willow-full.pgm
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 b84465cdbbfe3e2fb9eb4579e0bcaf0e)
  catkin_download_test_data(${PROJECT_NAME}_willow-full-0.05.pgm
    http://download.ros.org/data/amcl/willow-full-0.05.pgm
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 b61694296e08965096c5e78611fd9765)

  # Tests
  add_rostest(test/set_initial_pose.xml)
  add_rostest(test/set_initial_pose_delayed.xml)
  add_rostest(test/basic_localization_stage.xml)
  add_rostest(test/small_loop_prf.xml)
  add_rostest(test/small_loop_crazy_driving_prg.xml)
  add_rostest(test/texas_greenroom_loop.xml)
  add_rostest(test/rosie_multilaser.xml)
  add_rostest(test/texas_willow_hallway_loop.xml)

# Not sure when or if this actually passed.
#
# The point of this is that you start with an even probability
# distribution over the whole map and the robot localizes itself after
# some number of iterations of sensing and motion.
#
#  add_rostest(test/global_localization_stage.xml)
endif()
