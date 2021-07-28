#!/usr/bin/env python3
"""
Test ray query 3D
"""

import sys
import rospy
import math
import tf.transformations
import costmap_3d.srv
import geometry_msgs.msg
import copy


if __name__ == "__main__":
    x = 0
    if len(sys.argv) > 1:
        x = float(sys.argv[1])
    y = 0
    if len(sys.argv) > 2:
        y = float(sys.argv[2])
    z = 0
    if len(sys.argv) > 3:
        z = float(sys.argv[3])
    roll = 0
    if len(sys.argv) > 4:
        roll = float(sys.argv[4])
    pitch = 0
    if len(sys.argv) > 5:
        pitch = float(sys.argv[5])
    yaw = 0
    if len(sys.argv) > 6:
        yaw = float(sys.argv[6])

    rospy.init_node("ray_query_3d", anonymous=True)

    ray_query_srv = rospy.ServiceProxy("/move_base/local_costmap/ray_query_3d",
                                      costmap_3d.srv.RayQuery3DService)
    req = costmap_3d.srv.RayQuery3DServiceRequest()
    req.header.frame_id = "base_footprint"
    req.header.stamp = rospy.Time.now()
    req.width = .1
    req.height = .2
    req.transform_wait_time_limit = 0.1
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    for yaw_delta in range(-180, 180, 5):
        for pitch_delta in range(-50, 50, 5):
            pose.orientation = geometry_msgs.msg.Quaternion(
                    *tf.transformations.quaternion_from_euler(
                        roll,
                        pitch + pitch_delta * math.pi / 180.00,
                        yaw + yaw_delta * math.pi / 180.0))
            req.ray_poses.append(copy.deepcopy(pose))
    rospy.loginfo("Request: " + str(req))
    res = ray_query_srv(req)
    rospy.loginfo("Result: " + str(res))
