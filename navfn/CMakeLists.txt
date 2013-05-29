cmake_minimum_required(VERSION 2.8.3)
project(navfn)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        geometry_msgs
        costmap_2d
        nav_core
        pluginlib
        nav_msgs
        tf
        visualization_msgs
        pcl_ros
        message_generation
        )

# <rosdep name="fltk"/>
# <rosdep name="netpbm"/>
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    SYSTEM
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})


# services
add_service_files(
    DIRECTORY srv
    FILES
    MakeNavPlan.srv
    SetCostmap.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        navfn
    CATKIN_DEPENDS
        nav_core
        roscpp
        pluginlib
)

add_library (navfn src/navfn.cpp src/navfn_ros.cpp)
target_link_libraries(navfn
    ${catkin_LIBRARIES}
    )

add_executable(navfn_node src/navfn_node.cpp)
target_link_libraries(navfn_node
    navfn
    )

install(TARGETS navfn
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/
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
  add_executable (navtest src/navtest.cpp src/navwin.cpp)
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

add_subdirectory(test)
