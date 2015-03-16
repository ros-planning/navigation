cmake_minimum_required(VERSION 2.8.3)
project(move_base_msgs)

find_package(catkin REQUIRED
    COMPONENTS
        message_generation
        actionlib_msgs
        geometry_msgs
        )

#################
##   Coverage  ##
#################

set(COVERAGE "OFF" CACHE STRING "Enable coverage generation.")

message(Status "Using COVERAGE: ${COVERAGE}")
if("${COVERAGE}" STREQUAL "ON")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs -ftest-coverage")
endif()

# actions
add_action_files(
    DIRECTORY action
    FILES
        MoveBase.action
)

generate_messages(
    DEPENDENCIES
        actionlib_msgs
        geometry_msgs
)

catkin_package(
)


