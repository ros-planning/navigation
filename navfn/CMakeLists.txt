cmake_minimum_required(VERSION 2.8.3)
project(navfn)

include(CheckIncludeFile)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        costmap_2d
        geometry_msgs
        message_generation
        nav_core
        nav_msgs
        pluginlib
        rosconsole
        roscpp
        tf2_ros
        sensor_msgs
        visualization_msgs
        )

find_package(Eigen3 REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})


# services
add_service_files(
    DIRECTORY srv
    FILES
    MakeNavPlan.srv
    SetCostmap.srv
)

generate_messages(
    DEPENDENCIES
        geometry_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        navfn
    CATKIN_DEPENDS
        geometry_msgs
        message_runtime
        nav_core
        nav_msgs
        pluginlib
        roscpp
        sensor_msgs
        visualization_msgs
)

check_include_file(sys/time.h HAVE_SYS_TIME_H)
if (HAVE_SYS_TIME_H)
  add_definitions(-DHAVE_SYS_TIME_H)
endif (HAVE_SYS_TIME_H)

add_library (navfn src/navfn.cpp src/navfn_ros.cpp)
target_link_libraries(navfn
    ${catkin_LIBRARIES}
    )
add_dependencies(navfn ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(navfn_node src/navfn_node.cpp)
target_link_libraries(navfn_node
    navfn
    )

install(TARGETS navfn_node
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(TARGETS navfn
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/navfn/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

### The problem with FindFLTK is that it only reports success if *all*
### fltk components are installed, but we only need the core library.
# include (FindFLTK)
include (CheckIncludeFileCXX)
check_include_file_cxx (FL/Fl.H NAVFN_HAVE_FLTK)
check_include_file_cxx (pgm.h NAVFN_HAVE_NETPBM)
message (STATUS "NAVFN_HAVE_FLTK: ${NAVFN_HAVE_FLTK}, NETPBM: ${NAVFN_HAVE_NETPBM}")
# Just linking -lfltk is not sufficient on OS X
if (NAVFN_HAVE_FLTK AND NAVFN_HAVE_NETPBM AND NOT APPLE)
  message (STATUS "FLTK found: adding navtest to build")
  add_executable (navtest src/navtest/navtest.cpp src/navtest/navwin.cpp)
  set (FLTK_SKIP_FLUID 1)
  set (FLTK_SKIP_FORMS 1)
  set (FLTK_SKIP_IMAGES 1)
  find_package(FLTK)
  if(FLTK_FOUND)
    target_link_libraries (navtest navfn netpbm ${FLTK_LIBRARIES})
  else (FLTK_FOUND)
    target_link_libraries (navtest navfn netpbm fltk)
  endif (FLTK_FOUND)
else (NAVFN_HAVE_FLTK)
  message (STATUS "FLTK orf NETPBM not found: cannot build navtest")
endif (NAVFN_HAVE_FLTK AND NAVFN_HAVE_NETPBM AND NOT APPLE)

### For some reason (on cmake-2.4.7 at least) the "check" for pgm.h
### always succeeds, even if pgm.h is not installed. It seems to be
### caused by a bug in the rule that attempts to build the C source:
### instead of directly calling e.g. 'gcc -c
### /CMakeFiles/CMakeTmp/CheckIncludeFile.c' it goes through some make
### infrastructure, which reports "Nothing to be done for
### `CMakeFiles/cmTryCompileExec.dir/build'" and calls that a success.
###
### As a workaround we simply force everyone to install libnetpbm
#
# include (CheckIncludeFile)
# check_include_file (pgm.h NAVFN_HAVE_NETPBM)
# message (STATUS "NAVFN_HAVE_NETPBM: ${NAVFN_HAVE_NETPBM}")
# if (NAVFN_HAVE_NETPBM)
#   message (STATUS "found pgm.h")
#   add_definitions (-DNAVFN_HAVE_NETPBM)
#target_link_libraries (navfn -lnetpbm)
# else (NAVFN_HAVE_NETPBM)
#   message (STATUS "pgm.h not found (optional)")
# endif (NAVFN_HAVE_NETPBM)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
