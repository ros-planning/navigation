#!/usr/bin/env python

import rospy

import math
from tf import transformations
from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseSetter(rospy.SubscribeListener):
    def __init__(self, pose, stamp, publish_time):
        self.pose = pose
        self.stamp = stamp
        self.publish_time = publish_time

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        p = PoseWithCovarianceStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.stamp
        p.pose.pose.position.x = self.pose[0]
        p.pose.pose.position.y = self.pose[1]
        (p.pose.pose.orientation.x,
         p.pose.pose.orientation.y,
         p.pose.pose.orientation.z,
         p.pose.pose.orientation.w) = transformations.quaternion_from_euler(0, 0, self.pose[2])
        p.pose.covariance[6*0+0] = 0.5 * 0.5
        p.pose.covariance[6*1+1] = 0.5 * 0.5
        p.pose.covariance[6*3+3] = math.pi/12.0 * math.pi/12.0
        # wait for the desired publish time
        while rospy.get_rostime() < self.publish_time:
            rospy.sleep(0.01)
        peer_publish(p)


if __name__ == '__main__':
    pose = map(float, rospy.myargv()[1:4])
    t_stamp = rospy.Time()
    t_publish = rospy.Time()
    if len(rospy.myargv()) > 4:
        t_stamp = rospy.Time.from_sec(float(rospy.myargv()[4]))
    if len(rospy.myargv()) > 5:
        t_publish = rospy.Time.from_sec(float(rospy.myargv()[5]))
    rospy.init_node('pose_setter', anonymous=True)
    rospy.loginfo("Going to publish pose {} with stamp {} at {}".format(pose, t_stamp.to_sec(), t_publish.to_sec()))
    pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, PoseSetter(pose, stamp=t_stamp, publish_time=t_publish), queue_size=1)
    rospy.spin()
