#!/usr/bin/env python

import rospy
from robot_pose_ekf.srv import GetStatus, GetStatusRequest

if __name__ == '__main__':
    rospy.init_node('spawner', anonymous=True)
    print 'looking for node robot_pose_ekf...'
    rospy.wait_for_service('robot_pose_ekf/get_status')

    s = rospy.ServiceProxy('robot_pose_ekf/get_status', GetStatus)
    resp = s.call(GetStatusRequest())
    print resp.status
