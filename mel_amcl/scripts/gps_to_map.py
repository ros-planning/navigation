#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 10 09:38:47 2019

@author: michael
"""
# This script takes latitude and longitude data from a gps message (sensor_msgs/NavSatFix) and a world referenced heading from an Imu message (sensor_msgs/Imu).
# The script uses GGA_to_utm.py to transform the incoming lat/lon to utm coords (eastings northings [ENU system]) and procutes_analysis.py to to find the rotation matrix between utm and the robots map frame.
# The output of the script is the robots pose (geometry_msgs/PoseWithCovarianceStamped) in the map frame approximated from measurements by the GPS and the rotation matrix which uses reference GPS points in the map. The pose frame_id should match the GPS' to take into accound the transform from the GPS position and the base_link.

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from math import sin
from math import cos
from math import pi
import tf
import utm
import parse_gps_logs



class GPS_to_map_pose_publish:
    
    def __init__(self):

        self.got_tf = False

#        Get ROS Parameters
        gps_data_directory = rospy.get_param('~gps_log_directory', 'riseholme')
        self.gps_yaw_offset_rad = rospy.get_param('~gps_yaw_offset_rad', 0.0) # use this if it is not in the GPS tf
        self.added_gps_position_covariance = rospy.get_param('~added_gps_position_covariance', 0.0)
        self.added_gps_orientation_covariance = rospy.get_param('~added_gps_orientation_covariance', 0.0)
#        self.gps_tf_frame_id = rospy.get_param('~gps_frame_id', 'septentrio') # use this if data did not arrive with header usually use with static_tf_publisher for post processing


#        Need to keep track of the robot orientation in the map for the GPS tf
        self.map_tf_listener = tf.TransformListener()


#        Get transformation data from gps logs
        transformation_from_logs = parse_gps_logs.GGA_to_utm_to_map_transformation(gps_data_directory)
        self.transformation_matrix, self.theta_trans_rad = transformation_from_logs.transformation_matrix, transformation_from_logs.theta_trans_rad

        
#        Setup publishers
        self.pose_pub = rospy.Publisher('gps/map_pose', PoseWithCovarianceStamped, queue_size=10)
        self.yaw_pub = rospy.Publisher('gps/map_yaw', Imu, queue_size=10)
        self.pose_yaw_pub = rospy.Publisher('gps/map_pose_yaw', PoseWithCovarianceStamped, queue_size=10)
        
        self.pose_msg = PoseWithCovarianceStamped()
        self.yaw_msg = Imu()
        self.pose_yaw_msg = PoseWithCovarianceStamped()

        self.pose_msg.header.frame_id = 'map'
        self.yaw_msg.header.frame_id = 'base_link'
        self.pose_yaw_msg.header.frame_id = 'map'

        
#        Setup subscribers
        self.pose_subscriber = rospy.Subscriber('/gps/fix', NavSatFix, self.pose_callback, queue_size = 10)
        self.yaw_subscriber = rospy.Subscriber('/yaw', Imu, self.yaw_callback, queue_size = 10)



    def pose_callback(self,data):

        if not self.got_tf:        
            self.get_gps_antenna_tf(data.header.frame_id)
            
            
        try:
            now = rospy.Time.now()
            self.map_tf_listener.waitForTransform('map', 'base_link', now, rospy.Duration(0.5))
            (map_position, map_orientation_quat) = self.map_tf_listener.lookupTransform('map', 'base_link', now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        
        map_orientation = tf.transformations.euler_from_quaternion(map_orientation_quat)
        self.map_orientation_rad = map_orientation[2]


        lat = data.latitude
        lon = data.longitude
        location_cov = data.position_covariance

        utm_coord = utm.from_latlon(lat, lon)

        x = self.transformation_matrix[0,0]*utm_coord[0] + self.transformation_matrix[0,1]*utm_coord[1] + self.transformation_matrix[0,2]
        y = self.transformation_matrix[1,0]*utm_coord[0] + self.transformation_matrix[1,1]*utm_coord[1] + self.transformation_matrix[1,2]
        
    
        xpos = x - ( (self.trans[0]*cos(self.map_orientation_rad)) - (self.trans[1]*sin(self.map_orientation_rad)) )
        ypos = y - ( (self.trans[0]*sin(self.map_orientation_rad)) + (self.trans[1]*cos(self.map_orientation_rad)) )
        

        self.pose_msg.header.stamp = data.header.stamp
        self.pose_msg.pose.pose.position.x = xpos
        self.pose_msg.pose.pose.position.y = ypos
        self.pose_msg.pose.covariance[0] = location_cov[0] + self.added_gps_position_covariance
        self.pose_msg.pose.covariance[7] = location_cov[4] + self.added_gps_position_covariance
        self.pose_msg.pose.covariance[14] = location_cov[8] + self.added_gps_position_covariance
        self.pose_pub.publish(self.pose_msg) # need to add orientation covariances!!
        
        

    def yaw_callback(self,data):
    
        if not self.got_tf:        
            self.get_gps_antenna_tf(data.header.frame_id)
            
        try:
            now = rospy.Time.now()
            self.map_tf_listener.waitForTransform('map', 'base_link', now, rospy.Duration(0.5))
            (map_position, map_orientation_quat) = self.map_tf_listener.lookupTransform('map', 'base_link', now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
            
        input_quat = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        orientation_input_data = tf.transformations.euler_from_quaternion(input_quat) # roll,pttch,yaw


#        This one works for old riseholme
        yaw_map_rad = orientation_input_data[2] + self.theta_trans_rad + pi/2 + self.rot[2] + self.gps_yaw_offset_rad


        trans_quat = tf.transformations.quaternion_from_euler(orientation_input_data[0], orientation_input_data[1], yaw_map_rad) # roll,pttch,yaw

        self.pose_yaw_msg.header.stamp = data.header.stamp
        self.pose_yaw_msg.pose.pose.position = self.pose_msg.pose.pose.position
        self.pose_yaw_msg.pose.covariance = self.pose_msg.pose.covariance 
        self.pose_yaw_msg.pose.pose.orientation = Quaternion(*trans_quat)
        self.pose_yaw_msg.pose.covariance[21] = 0.1 + self.added_gps_orientation_covariance
        self.pose_yaw_msg.pose.covariance[28] = 0.1 + self.added_gps_orientation_covariance
        self.pose_yaw_msg.pose.covariance[35] = 0.1 + self.added_gps_orientation_covariance
        self.pose_yaw_pub.publish(self.pose_yaw_msg) # need to add orientation covariances to gps parser!!
    
        self.yaw_msg.header.stamp= data.header.stamp
        self.yaw_msg.orientation = Quaternion(*trans_quat)
        self.yaw_msg.orientation_covariance[0] = 0.1 + self.added_gps_orientation_covariance
        self.yaw_msg.orientation_covariance[4] = 0.1 + self.added_gps_orientation_covariance
        self.yaw_msg.orientation_covariance[8] = 0.1 + self.added_gps_orientation_covariance
        self.yaw_pub.publish(self.yaw_msg) # need to add orientation covariances to gps parser!!
        
    
    
    def get_gps_antenna_tf(self, frame_id):
        
#        Get GPS antenna tf
        rospy.loginfo( "Waiting for GPS tf.")
        GPS_tf_listener = tf.TransformListener()
        while not self.got_tf:
            try:
                (self.trans, self.rot_q) = GPS_tf_listener.lookupTransform('base_link', frame_id, rospy.Time(0))
                self.got_tf = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.got_tf = False
                continue
        self.rot = tf.transformations.euler_from_quaternion(self.rot_q) # roll,pttch,yaw
        rospy.loginfo( "Got gps tf")
        


def main():
    
    rospy.init_node('gps_to_map_transform', anonymous=True)
    
    GPS_to_map_pose_publish()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down GPS to map pose transformation module"
    
    
if __name__ == '__main__':
    main()

