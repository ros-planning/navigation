#!/usr/bin/env python
import rospy
from amcl.msg import HypothesisSet
from geometry_msgs.msg import PoseWithCovariance
import math

if __name__ == '__main__':
    pub = rospy.Publisher('/initialpose_cloud', HypothesisSet, queue_size=10)
    rospy.init_node("amcl_multi_init_test")

    rate = rospy.Rate(.1)
    rate.sleep()
    set = HypothesisSet()

    #Construct a pose with covariance
    pc0 = PoseWithCovariance()

    pc0.pose.position.x = 0.0
    pc0.pose.position.y = 0.0
    pc0.pose.position.z = 0.0

    pc0.pose.orientation.x = 0.0
    pc0.pose.orientation.y = 0.0
    pc0.pose.orientation.z = 0.0
    pc0.pose.orientation.w = 1.0

    pc0.covariance[6*0 + 0] = 0.5 * 0.5
    pc0.covariance[6*1 + 1] = 0.5 * 0.5
    pc0.covariance[6*5 + 5] = math.pi/12.0 * math.pi/12.0

    pc1 = PoseWithCovariance()

    pc1.pose.position.x = 6.0
    pc1.pose.position.y = 1.0
    pc1.pose.position.z = 0.0

    pc1.pose.orientation.x = 0.0
    pc1.pose.orientation.y = 0.0
    pc1.pose.orientation.z = 0.0
    pc1.pose.orientation.w = 1.0

    pc1.covariance[6*0 + 0] = 0.5 * 0.5
    pc1.covariance[6*1 + 1] = 0.5 * 0.5
    pc1.covariance[6*5 + 5] = math.pi/12.0 * math.pi/12.0

    set.hypotheses.append(pc0)
    set.hypotheses.append(pc1)

    pub.publish(set)

    rate.sleep()



