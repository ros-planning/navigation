#!/usr/bin/env python

import rospy

import math
from tf import transformations
from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseSetter(rospy.SubscribeListener):
    def __init__(self, pose):
        self.pose = pose

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        p = PoseWithCovarianceStamped()
        p.header.frame_id = "map"
        p.pose.pose.position.x = self.pose[0]
        p.pose.pose.position.y = self.pose[1]
        (p.pose.pose.orientation.x,
         p.pose.pose.orientation.y,
         p.pose.pose.orientation.z,
         p.pose.pose.orientation.w) = transformations.quaternion_from_euler(0, 0, self.pose[2])
        p.pose.covariance[6*0+0] = 0.5 * 0.5
        p.pose.covariance[6*1+1] = 0.5 * 0.5
        p.pose.covariance[6*3+3] = math.pi/12.0 * math.pi/12.0
        peer_publish(p)


if __name__ == '__main__':
    pose = map(float, rospy.myargv()[1:])
    rospy.init_node('pose_setter', anonymous=True)
    pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, PoseSetter(pose))
    rospy.spin()
