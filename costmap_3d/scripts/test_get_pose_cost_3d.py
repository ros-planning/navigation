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

    pose_array_pub = rospy.Publisher("/test_costmap_3d/pose_array", geometry_msgs.msg.PoseArray, queue_size=1)
    test_ns = "/move_base/local_costmap/"
    get_cost_srv = rospy.ServiceProxy(test_ns + "get_plan_cost_3d",
            costmap_3d.srv.GetPlanCost3DService)
    pose_array = geometry_msgs.msg.PoseArray()
    pose_array.header.frame_id = "odom"
    pose_array.header.stamp = rospy.Time.now()
    req = costmap_3d.srv.GetPlanCost3DServiceRequest()
    req.lazy = False
    req.buffered = False
    req.header.frame_id = "odom"
    req.header.stamp = pose_array.header.stamp
#    req.cost_query_mode = costmap_3d.srv.GetPlanCost3DServiceRequest.COST_QUERY_MODE_SIGNED_DISTANCE
    req.cost_query_mode = costmap_3d.srv.GetPlanCost3DServiceRequest.COST_QUERY_MODE_DISTANCE
#    req.cost_query_mode = costmap_3d.srv.GetPlanCost3DServiceRequest.COST_QUERY_MODE_COLLISION_ONLY
    req.footprint_mesh_resource = ""
#    req.footprint_mesh_resource = "package://ant_description/meshes/robot-get-close-to-obstacle-backward.stl"
    req.padding = 0.0
#    req.padding = 0.25
#    for i in range(0,35*4*5*8):
    for teb in range(0,4):
#    for teb in range(0,60*4):
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "base_footprint"
#        pose.pose.position.x = random.uniform(-4.0, 4.0)
#        pose.pose.position.y = random.uniform(-4.0, 4.0)
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        # pick a random direction to travel in
        path_theta = random.uniform(-math.pi, math.pi)
        path_poses = []
        while math.hypot(pose.pose.position.x, pose.pose.position.y) < 4.0:
            q = tf.transformations.quaternion_from_euler(0.0, 0.0, path_theta)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path_poses.append(copy.deepcopy(pose))
            # Simulate motion in path theta
            d = random.gauss(.075, .01)
            pose.pose.position.x += d*math.cos(path_theta)
            pose.pose.position.y += d*math.sin(path_theta)
            # Pick a new path theta
            path_theta = random.gauss(path_theta, math.pi/36)

        # Simulate 15Hz planning loop and 5Hz map update dumping distance cache
        for planning_loop in range(0,3):
            # Simulate optimization step
            for optimization_step in range(0,20):
                for i in range(0,len(path_poses)):
                    path_pose = path_poses[i]
                    path_pose.pose.position.x += random.gauss(0, .25/(2.7**(optimization_step+planning_loop)))
                    path_pose.pose.position.y += random.gauss(0, .25/(2.7**(optimization_step+planning_loop)))
                    if path_pose.pose.position.x > 4.0:
                        path_pose.pose.position.x = 4.0
                    if path_pose.pose.position.x < -4.0:
                        path_pose.pose.position.x = -4.0
                    if path_pose.pose.position.y > 4.0:
                        path_pose.pose.position.y = 4.0
                    if path_pose.pose.position.y < -4.0:
                        path_pose.pose.position.y = -4.0
                    q = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
                    q[0] = path_pose.pose.orientation.x
                    q[1] = path_pose.pose.orientation.y
                    q[2] = path_pose.pose.orientation.z
                    q[3] = path_pose.pose.orientation.w
                    theta = tf.transformations.euler_from_quaternion(q)[2]
                    theta += random.gauss(0, (math.pi/36)/(2**(optimization_step+planning_loop)))
                    q = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
                    path_pose.pose.orientation.x = q[0]
                    path_pose.pose.orientation.y = q[1]
                    path_pose.pose.orientation.z = q[2]
                    path_pose.pose.orientation.w = q[3]
                    # necessary?
                    path_poses[i] = copy.deepcopy(path_pose)
                    orig_pose = path_pose
                    req.poses.append(tf2_geometry_msgs.do_transform_pose(orig_pose, xform))
                    pose_array.poses.append(tf2_geometry_msgs.do_transform_pose(orig_pose, xform).pose)
                    # simulate calls for jacobian
                    epsilon = 1e-9
                    path_pose.pose.position.x += epsilon
                    req.poses.append(tf2_geometry_msgs.do_transform_pose(path_pose, xform))
                    pose_array.poses.append(tf2_geometry_msgs.do_transform_pose(path_pose, xform).pose)
                    path_pose.pose.position.x -= epsilon
                    path_pose.pose.position.y += epsilon
        #            req.poses.append(tf2_geometry_msgs.do_transform_pose(orig_pose, xform))
                    req.poses.append(tf2_geometry_msgs.do_transform_pose(path_pose, xform))
                    pose_array.poses.append(tf2_geometry_msgs.do_transform_pose(path_pose, xform).pose)
                    path_pose.pose.position.y -= epsilon
                    theta += epsilon
                    q = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
                    path_pose.pose.orientation.x = q[0]
                    path_pose.pose.orientation.y = q[1]
                    path_pose.pose.orientation.z = q[2]
                    path_pose.pose.orientation.w = q[3]
        #            req.poses.append(tf2_geometry_msgs.do_transform_pose(orig_pose, xform))
                    req.poses.append(tf2_geometry_msgs.do_transform_pose(path_pose, xform))
                    pose_array.poses.append(tf2_geometry_msgs.do_transform_pose(path_pose, xform).pose)

    pose_array_pub.publish(pose_array)
#    rospy.loginfo("Request: " + str(req))
    res = get_cost_srv(req)
    rospy.loginfo("Result: " + str(res))
    rospy.loginfo("Number of lethals: " + str(len(res.lethal_indices)))
    rospy.spin()
