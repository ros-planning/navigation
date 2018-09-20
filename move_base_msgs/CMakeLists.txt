cmake_minimum_required(VERSION 2.8.3)
project(move_base_msgs)

find_package(catkin REQUIRED
    COMPONENTS
        message_generation
        actionlib_msgs
        geometry_msgs
)

# services
add_service_files(
    DIRECTORY srv
    FILES
        ClearCostmap.srv
)

# actions
add_action_files(
    DIRECTORY action
    FILES
        MoveBase.action
)

add_message_files(
    DIRECTORY msg
    FILES
        PoseWithCovarianceStampedArray.msg
        LoopMiss.msg
        amcl_analytics.msg
        amcl_data.msg
        cluster.msg
        flatCov.msg
)

generate_messages(
    DEPENDENCIES
        actionlib_msgs
        geometry_msgs
)

catkin_package(
)


