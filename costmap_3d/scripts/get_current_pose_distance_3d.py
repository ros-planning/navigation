#!/usr/bin/env python3
"""
Test the performance of 3D costmap by sending a veyr large get plan cost service requests.
"""

import sys
#import math
#import random
import rospy
#import tf2_ros
#import tf2_geometry_msgs
#import tf.transformations
import costmap_3d_msgs.srv
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

    rospy.init_node("test_costmap3d_get_plan_cost_performance", anonymous=True)

    get_cost_srv = rospy.ServiceProxy("/move_base/local_costmap/get_plan_cost_3d",
                                      costmap_3d_msgs.srv.GetPlanCost3DService)
    req = costmap_3d_msgs.srv.GetPlanCost3DServiceRequest()
    req.lazy = False
    req.buffered = False
    req.header.stamp = rospy.Time.now()
    req.cost_query_mode = costmap_3d_msgs.srv.GetPlanCost3DServiceRequest.COST_QUERY_MODE_SIGNED_DISTANCE
    # req.footprint_mesh_resource = "package://ant_description/meshes/robot-get-close-to-obstacle-backward.stl"
    # req.padding = float('NaN')
    # req.footprint_mesh_resource = ""
    # req.padding = 0.0
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = "base_footprint"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    req.poses.append(copy.deepcopy(pose))
    pose.pose.position.x = x + 1e-5
    req.poses.append(copy.deepcopy(pose))
    pose.pose.position.x = x
    pose.pose.position.y = y + 1e-5
    req.poses.append(copy.deepcopy(pose))
    pose.pose.position.y = y
    pose.pose.orientation.z += 1e-5
    pose.pose.orientation.w -= 1e-5
    req.poses.append(copy.deepcopy(pose))
#    req.cost_query_regions = [costmap_3d_msgs.srv.GetPlanCost3DServiceRequest.COST_QUERY_REGION_ALL,
#                              costmap_3d_msgs.srv.GetPlanCost3DServiceRequest.COST_QUERY_REGION_LEFT,
#                              costmap_3d_msgs.srv.GetPlanCost3DServiceRequest.COST_QUERY_REGION_RIGHT]
    rospy.loginfo("Request: " + str(req))
    res = get_cost_srv(req)
    rospy.loginfo("Result: " + str(res))
    rospy.loginfo("Number of lethals: " + str(len(res.lethal_indices)))
    rospy.loginfo(" dx: " + str(1e5 * (res.pose_costs[1]-res.pose_costs[0])))
    rospy.loginfo(" dy: " + str(1e5 * (res.pose_costs[2]-res.pose_costs[0])))
    rospy.loginfo(" dt: " + str(1e5 * (res.pose_costs[3]-res.pose_costs[0])))
