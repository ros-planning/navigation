cmake_minimum_required(VERSION 3.0.2)
project(costmap_3d_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
)

add_action_files(
  FILES
  GetPlanCost3D.action
  RayQuery3D.action
)

add_service_files(
  FILES
  GetPlanCost3DService.srv
  RayQuery3DService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
