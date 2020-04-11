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
import sensor_msgs.msg
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

    pose_array_pub = rospy.Publisher("/test_costmap_3d/pose_array",
            geometry_msgs.msg.PoseArray, queue_size=1, latch=True)
    point_cloud_pub = rospy.Publisher("/test_costmap_3d/distance_cloud",
            sensor_msgs.msg.PointCloud, queue_size=1, latch=True)
    test_ns = "/move_base/local_costmap/"
    get_cost_srv = rospy.ServiceProxy(test_ns + "get_plan_cost_3d",
            costmap_3d.srv.GetPlanCost3DService)
    frame = "odom"
    pose_array = geometry_msgs.msg.PoseArray()
    pose_array.header.frame_id = frame
    pose_array.header.stamp = rospy.Time.now()
    req = costmap_3d.srv.GetPlanCost3DServiceRequest()
    req.lazy = False
    req.buffered = False
    req.header.frame_id = frame
    req.header.stamp = pose_array.header.stamp
    req.cost_query_mode = costmap_3d.srv.GetPlanCost3DServiceRequest.COST_QUERY_MODE_SIGNED_DISTANCE
#    req.cost_query_mode = costmap_3d.srv.GetPlanCost3DServiceRequest.COST_QUERY_MODE_DISTANCE
#    req.cost_query_mode = costmap_3d.srv.GetPlanCost3DServiceRequest.COST_QUERY_MODE_COLLISION_ONLY
    req.footprint_mesh_resource = ""
#    req.footprint_mesh_resource = "package://ant_description/meshes/robot-get-close-to-obstacle-backward.stl"
    req.padding = 0.0
#    req.padding = 0.25
    x_start = 0.0
    y_start = 0.0
    x_res = 0.01
    y_res = 0.01
    x_0 = -200
    y_0 = -200
    x_n = 200
    y_n = 200
    regions = []
    for i in range(x_0, x_n):
        for j in range(y_0, y_n):
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = frame
            pose.pose.position.x = x_start + i * x_res
            pose.pose.position.y = y_start + j * y_res
            pose.pose.position.z = 0.0
            theta = 0.5
            q = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            req.poses.append(tf2_geometry_msgs.do_transform_pose(pose, xform))
            regions.append(costmap_3d.srv.GetPlanCost3DServiceRequest.COST_QUERY_REGION_ALL)
            pose_array.poses.append(tf2_geometry_msgs.do_transform_pose(pose, xform).pose)
    req.cost_query_regions = regions

    pose_array_pub.publish(pose_array)

#    rospy.loginfo("Request: " + str(req))
    res = get_cost_srv(req)

    point_cloud = sensor_msgs.msg.PointCloud()
    point_cloud.header = pose_array.header
    for p, cost in zip(pose_array.poses, res.pose_costs):
        z = -cost
        if (z > 0):
            z += 0.5;
        if (z > 5.0):
            z = 5.0;
        if (z < -5.0):
            z = -5.0;
        point_cloud.points.append(geometry_msgs.msg.Point32(p.position.x, p.position.y, z))
    point_cloud_pub.publish(point_cloud)

#    rospy.loginfo("Result: " + str(res))
#    rospy.loginfo("Number of lethals: " + str(len(res.lethal_indices)))
    rospy.spin()
