cmake_minimum_required(VERSION 2.8.3)
project(voxel_grid)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
        )

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        voxel_grid
    CATKIN_DEPENDS
        roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(voxel_grid src/voxel_grid.cpp)
target_link_libraries(voxel_grid ${catkin_LIBRARIES})

add_executable(voxel_grid_sample src/voxel_grid_main.cpp)
target_link_libraries(voxel_grid_sample
    voxel_grid
    ${catkin_LIBRARIES}
    )

install(TARGETS voxel_grid
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

catkin_add_gtest(voxel_grid_tests test/voxel_grid_tests.cpp)
target_link_libraries(voxel_grid_tests
    voxel_grid
    ${catkin_LIBRARIES}
    )
