#!/usr/bin/env python
"""
Test the performance of the 2D costmap_monitor layer by sending a very large
get plan cost service request. This is a useful baseline for comparing its
performance with the 3D version.
"""

import sys
import math
import random
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations
import costmap_monitor_msgs.srv
import geometry_msgs.msg


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

    get_cost_srv = rospy.ServiceProxy("/move_base/local_costmap/costmap_monitor/get_plan_cost",
            costmap_monitor_msgs.srv.GetPlanCostService)
    req = costmap_monitor_msgs.srv.GetPlanCostServiceRequest()
    req.lazy = False
    req.header.frame_id = "odom"
    req.padding = 0.0
    pt = geometry_msgs.msg.Point()
    pt.z = 0.0
    pt.x = .372
    pt.y = 0.0
    req.footprint.append(pt)
    pt.x = .368
    pt.y = -.178
    req.footprint.append(pt)
    pt.x = .345
    pt.y = -.235
    req.footprint.append(pt)
    pt.x = .293
    pt.y = -.272
    req.footprint.append(pt)
    pt.x = .0114
    pt.y = -.285
    req.footprint.append(pt)
    pt.x = -0.262
    pt.y = -0.272
    req.footprint.append(pt)
    pt.x = -0.312
    pt.y = -0.242
    req.footprint.append(pt)
    pt.x = -0.343
    pt.y = -0.192
    req.footprint.append(pt)
    pt.x = -0.350
    pt.y = 0.0
    req.footprint.append(pt)
    pt.x = -0.343
    pt.y = 0.192
    req.footprint.append(pt)
    pt.x = -0.312
    pt.y = 0.242
    req.footprint.append(pt)
    pt.x = -0.262
    pt.y = 0.272
    req.footprint.append(pt)
    pt.x = 0.0114
    pt.y = 0.285
    req.footprint.append(pt)
    pt.x = 0.293
    pt.y = 0.272
    req.footprint.append(pt)
    pt.x = 0.345
    pt.y = 0.235
    req.footprint.append(pt)
    pt.x = 0.368
    pt.y = 0.178
    req.footprint.append(pt)
    for i in range(0,100000):
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "base_footprint"
        pose.pose.position.x = random.uniform(-4.0, 4.0)
        pose.pose.position.y = random.uniform(-4.0, 4.0)
        pose.pose.position.z = 0.0
        theta = random.uniform(-math.pi, math.pi)
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        req.poses.append(tf2_geometry_msgs.do_transform_pose(pose, xform))

    res = get_cost_srv(req)
    rospy.loginfo("Result: " + str(res))
