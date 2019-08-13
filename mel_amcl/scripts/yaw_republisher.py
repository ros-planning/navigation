#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 12 15:24:44 2019

@author: michael
"""


from sensor_msgs.msg import Imu
import tf
from geometry_msgs.msg import Quaternion
import rospy
from math import pi

class YawRepulisher:
    
    def __init__(self):
        

        self.yaw_repub = rospy.Publisher('/yaw/enu', Imu, queue_size=10)
        self.yaw_msg = Imu()
        self.yaw_msg.header.frame_id = 'base_link'
        
        rospy.Subscriber('/yaw', Imu, self.yaw_callback, queue_size = 10)

        
    def yaw_callback(self, data):
        input_quat = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        orientation_input_data = tf.transformations.euler_from_quaternion(input_quat) # roll,pttch,yaw

        yaw_enu = orientation_input_data[2] + pi/2


        trans_quat = tf.transformations.quaternion_from_euler(orientation_input_data[0], orientation_input_data[1], yaw_enu) # roll,pttch,yaw

        self.yaw_msg.header.stamp= data.header.stamp
        self.yaw_msg.orientation = Quaternion(*trans_quat)
        self.yaw_msg.orientation_covariance = data.orientation_covariance
        self.yaw_repub.publish(self.yaw_msg) # need to add orientation covariances to gps parser!!
            

    

def main():
    
    rospy.init_node('republish_yaw_as_enu', anonymous=True)
    YawRepulisher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "End yaw republishing module"
    



if __name__ == '__main__':
    main()
