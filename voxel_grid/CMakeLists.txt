cmake_minimum_required(VERSION 2.8.3)
project(voxel_grid)

find_package(catkin REQUIRED
        COMPONENTS
            roslib
            roscpp
        )

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        lib${PROJECT_NAME}
    CATKIN_DEPENDS
        roslib
        roscpp
)

include_directories( include )

add_library(libvoxel_grid src/voxel_grid.cpp)
set_target_properties(libvoxel_grid PROPERTIES OUTPUT_NAME voxel_grid)

add_executable(voxel_grid src/voxel_grid_main.cpp)
target_link_libraries(voxel_grid
    libvoxel_grid
    ${catkin_LIBRARIES}
    )

install(TARGETS libvoxel_grid
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )


install(
    TARGETS
        voxel_grid
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_add_gtest(voxel_grid_tests test/voxel_grid_tests.cpp)
target_link_libraries(voxel_grid_tests
    libvoxel_grid
    ${catkin_LIBRARIES}
    )
