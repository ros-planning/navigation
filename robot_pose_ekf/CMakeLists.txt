cmake_minimum_required(VERSION 2.8.3)
project(robot_pose_ekf)

# bfl (Bayesian Filtering Library) is a third party package that uses pkg-config
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf
            nav_msgs
            std_msgs
            geometry_msgs
            sensor_msgs
            message_generation
        )

find_package(Boost REQUIRED COMPONENTS thread)

# services
add_service_files(
    DIRECTORY srv
    FILES
    GetStatus.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
)

include_directories(
    "include"
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )

add_executable(robot_pose_ekf 
                       src/odom_estimation.cpp 
                       src/nonlinearanalyticconditionalgaussianodo.cpp 
                       src/odom_estimation_node.cpp)
target_link_libraries(robot_pose_ekf
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${BFL_LIBRARIES}
    )
add_dependencies(robot_pose_ekf robot_pose_ekf_gencpp)

install(
    TARGETS
        robot_pose_ekf
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES robot_pose_ekf.launch example_with_gps.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Tests are failing on OSX for an unknown reason
include(CMakeDetermineSystem)
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
if(CATKIN_ENABLE_TESTING)

catkin_download_test_data(
  download_data_ekf_test2_indexed.bag
  http://download.ros.org/data/robot_pose_ekf/ekf_test2_indexed.bag
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
  MD5 71addef0ed900e05b301e0b4fdca99e2
)
add_executable(test_robot_pose_ekf test/test_robot_pose_ekf.cpp)
target_link_libraries(test_robot_pose_ekf
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${BFL_LIBRARIES}
    gtest
    )

catkin_download_test_data(
  download_data_zero_covariance.bag
  http://download.ros.org/data/robot_pose_ekf/zero_covariance_indexed.bag
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
  MD5 1f1f4e361a9e0b0f6b1379b2dd011088
)
add_executable(test_robot_pose_ekf_zero_covariance test/test_robot_pose_ekf_zero_covariance.cpp)
target_link_libraries(test_robot_pose_ekf_zero_covariance
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${BFL_LIBRARIES}
    gtest
    )

# This has to be done after we've already built targets, or catkin variables get borked
find_package(rostest)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_pose_ekf.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_pose_ekf_zero_covariance.launch)

endif(CATKIN_ENABLE_TESTING)
endif(CMAKE_SYSTEM_NAME MATCHES "Linux")
