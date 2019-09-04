#!/usr/bin/env python
"""
Test the performance of 3D costmap by sending a veyr large get plan cost service requests.
"""

import sys
import math
import random
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
import costmap_3d.srv
import geometry_msgs.msg
import copy


if __name__ == "__main__":

    rospy.init_node("test_costmap3d_get_plan_cost_performance", anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    try:
        xform = tfBuffer.lookup_transform('odom', 'base_footprint',
                rospy.Time(0.0), rospy.Duration(10.0))

    except tf2_ros.TransformException as e:
        rospy.logerr("Cannot determine base footprint transform")
        sys.exit(1)

    get_cost_srv = rospy.ServiceProxy("/move_base/local_costmap/get_plan_cost_3d",
            costmap_3d.srv.GetPlanCost3DService)
    pose_array = geometry_msgs.msg.PoseArray()
    pose_array.header.frame_id = "odom"
    pose_array.header.stamp = rospy.Time.now()
    req = costmap_3d.srv.GetPlanCost3DServiceRequest()
    req.lazy = False
    req.buffered = True
    req.header.frame_id = "odom"
    req.header.stamp = pose_array.header.stamp
    req.cost_query_mode = costmap_3d.srv.GetPlanCost3DServiceRequest.COST_QUERY_MODE_DISTANCE
    req.footprint_mesh_resource = ""
#    req.footprint_mesh_resource = "package://ant_description/meshes/robot-get-close-to-obstacle-backward.stl"
#    req.padding = 0.0
    req.padding = float('NaN')
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = "base_footprint"
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    req.poses.append(tf2_geometry_msgs.do_transform_pose(pose, xform))
#    rospy.loginfo("Request: " + str(req))
    res = get_cost_srv(req)
    rospy.loginfo("Result: " + str(res))
    rospy.loginfo("Number of lethals: " + str(len(res.lethal_indices)))
