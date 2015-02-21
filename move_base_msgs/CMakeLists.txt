cmake_minimum_required(VERSION 2.8.3)
project(move_base_msgs)

find_package(catkin REQUIRED
    COMPONENTS
        message_generation
        actionlib_msgs
        geometry_msgs
        )


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


