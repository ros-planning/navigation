cmake_minimum_required(VERSION 2.8)
project(amcl)

include_directories(include)
include_directories(${ROS_INCLUDE_DIRS})
include_directories(/usr/local/include/bullet) # my local install of bullet

add_executable(amcl src/amcl_node.cpp)

target_link_libraries(amcl ${PYTHON_LIBRARIES})

install_cmake_infrastructure(amcl
  VERSION 0.0.0
  )
