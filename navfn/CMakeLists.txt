cmake_minimum_required (VERSION 2.4.6)

### If you're trying to debug include directives or so, this here
### might help:
#set (CMAKE_VERBOSE_MAKEFILE ON)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

### Debug mode implies slower runtime, "of course"
#set (ROS_BUILD_TYPE Debug)
set (ROS_BUILD_TYPE RelWithDebInfo)

add_definitions (-Wall)
rospack (navfn)
gensrv()
genmsg()

rospack_add_library (navfn src/navfn.cpp src/navfn_ros.cpp)

rospack_add_executable(bin/navfn_node src/navfn_node.cpp)
target_link_libraries(bin/navfn_node navfn)




### The problem with FindFLTK is that it only reports success if *all*
### fltk components are installed, but we only need the core library.
# include (FindFLTK)
include (CheckIncludeFileCXX)
check_include_file_cxx (FL/Fl.H NAVFN_HAVE_FLTK)
message (STATUS "NAVFN_HAVE_FLTK: ${NAVFN_HAVE_FLTK}")
# Just linking -lfltk is not sufficient on OS X
if (NAVFN_HAVE_FLTK AND NOT APPLE)
  message (STATUS "FLTK found: adding navtest to build")
  rospack_add_executable (navtest src/navtest.cpp src/navwin.cpp)
  target_link_libraries (navtest navfn fltk netpbm)
else (NAVFN_HAVE_FLTK)
  message (STATUS "FLTK not found: cannot build navtest")
endif (NAVFN_HAVE_FLTK AND NOT APPLE)

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
target_link_libraries (navfn -lnetpbm)
# else (NAVFN_HAVE_NETPBM)
#   message (STATUS "pgm.h not found (optional)")
# endif (NAVFN_HAVE_NETPBM)
