cmake_minimum_required(VERSION 2.8.3)
project(map_server)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED
        COMPONENTS
            roslib
            roscpp
            tf
            nav_msgs
            rostest
        )

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        image_loader
    CATKIN_DEPENDS
        roslib
        roscpp
        tf
        nav_msgs
)
include_directories( include ${Boost_INCLUDE_DIRS} )
add_library(image_loader src/image_loader.cpp)
target_link_libraries(image_loader SDL SDL_image ${Boost_LIBRARIES})

add_executable(map_server src/main.cpp)
target_link_libraries(map_server
    image_loader
    yaml-cpp
    ${catkin_LIBRARIES}
)

# copy test data to same place as tests are run
function(copy_test_data)
    cmake_parse_arguments(PROJECT "" "" "FILES" ${ARGN})
    foreach(datafile ${PROJECT_FILES})
        file(COPY ${datafile} DESTINATION ${PROJECT_BINARY_DIR}/test)
    endforeach()
endfunction()

copy_test_data( FILES
    test/testmap.bmp
    test/testmap.png )
catkin_add_gtest(utest test/utest.cpp test/test_constants.cpp)
target_link_libraries(utest image_loader SDL SDL_image)

add_executable(rtest test/rtest.cpp test/test_constants.cpp)
target_link_libraries( rtest
    gtest
    ${catkin_LIBRARIES}
)
add_dependencies(rtest nav_msgs_gencpp)

add_executable(map_saver src/map_saver.cpp)
target_link_libraries( map_saver
    ${catkin_LIBRARIES}
    )

add_rostest(test/rtest.xml)

install(
    TARGETS
        map_saver
        map_server
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
    include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
    image_loader
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
