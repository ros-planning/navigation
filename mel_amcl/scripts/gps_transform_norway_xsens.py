#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 10 09:38:47 2019

@author: michael
"""
# This script takes latitude and longitude data from a gps message (sensor_msgs/NavSatFix) and a world referenced heading from an Imu message (sensor_msgs/Imu).
# The script uses GGA_to_utm.py to transform the incoming lat/lon to utm coords (eastings northings [ENU system]) and procutes_analysis.py to to find the rotation matrix between utm and the robots map frame.
# The output of the script is the robots pose (geometry_msgs/PoseWithCovarianceStamped) in the map frame approximated from measurements by the GPS and the rotation matrix which uses reference GPS points in the map. The pose frame_id should match the GPS' to take into accound the transform from the GPS position and the base_link.

from procrutes_analysis import PA
from pyproj import Proj
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from geometry_msgs.msg import PoseWithCovariance
from math import atan
from math import sin
from math import cos
from math import pi
import PyKDL
import tf
import utm

#Q, ref_utm, ref_2D = PA()

#myProj = Proj("+proj=utm +zone=30, +east +ellps=WGS84 +datum=WGS84 +units=m")
#x, y = myProj(longitude_degrees, latitude_degrees) # x and y in utm coordinates
#loc = np.matrix((x,y))
#location = Q * (loc - ref_utm) +ref_2d
        



#scaling_data = PA()                 # Obtain returned values from procrutes_analysis's function PA
#Q = scaling_data[0]      # Q is the rotation matrix
#ref_utm = scaling_data[1] 
#ref_2D = scaling_data[2]
#ref_theta_deg = scaling_data[3]*180/pi


def euler_to_quaternion(yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw] 

def quaternion_to_euler(x, y, z, w):

        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        yaw = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        roll = math.atan2(t3, t4)


        return yaw, pitch, roll


class GPS_to_pose_message:
    
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.GPS_tf_listener = tf.TransformListener()
        self.msg = PoseWithCovarianceStamped()
        #self.msg.header.frame_id='map'
        self.gps_tf_frame_id = rospy.get_param('~gps_frame_id')
        got_tf = False
        rospy.loginfo( "Waiting for GPS tf.")
        while not got_tf:
            try:
                (self.trans,self.rot_q) = self.GPS_tf_listener.lookupTransform('base_link', self.gps_tf_frame_id, rospy.Time(0))
                #(self.trans,self.rot) = self.GPS_tf_listener.lookupTransform('base_link', 'septentrio', rospy.Time(0))

                got_tf = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                got_tf = False
                continue
        
            self.rot = tf.transformations.euler_from_quaternion(self.rot_q)# roll,pttch,yaw

        rospy.loginfo( "Got gps tf")
        
        self.msg.header.frame_id='map'
        self.pose_pub = rospy.Publisher('gps/map_pose', PoseWithCovarianceStamped, queue_size=10)

        self.yaw_trans_rad = 0.0 # initialise variable
        
        
        self.pose_subscriber = rospy.Subscriber('/gps/fix', NavSatFix, self.pose_callback, queue_size = 10)
        self.yaw_subscriber = rospy.Subscriber('/yaw', Imu, self.yaw_callback, queue_size = 10)


    def pose_callback(self,data):
#        print pub
#         now = rospy.get_rostime()
#        pose_stamped.header.stamp=now
#        print posewithcovariancestamped    
        lat =data.latitude
        lon =data.longitude
        location_cov = data.position_covariance
#        print cov
        myProj = Proj("+proj=utm +zone=30, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")# +a=6378140.5")    #Covert Lat/Lon to utm format
        utm_map = utm.from_latlon(lat, lon)
        x_map = utm_map[0]
        y_map = utm_map[1]
        #x_map, y_map = myProj(lon, lat)                     #northing and easting in utm frame
        loc = np.matrix((x_map,y_map))
        loc = np.transpose(loc) 
        location = Q * (loc - ref_utm) + ref_2D #Covert from utm frame to 2Dmap frame
        # Apply transformation caused by GPS offset from base_link
        # Note, cannot just use tf package as GPS data is not relative in the same way as, for example, LIDAR.
        # i.e sensor measurement or not  in the sensor frame (i.e. w.r.t. the sensor itself)
        
        xpos = location[0]# - ((self.trans[0]*cos(self.yaw_trans_rad)) - (self.trans[1]*sin(self.yaw_trans_rad)))
        ypos = location[1]# - ((self.trans[0]*sin(self.yaw_trans_rad)) + (self.trans[1]*cos(self.yaw_trans_rad)))
        
        
        #print 'tf', ((self.trans[0]*cos(self.yaw_trans_rad)) - (self.trans[1]*sin(self.yaw_trans_rad))), ((self.trans[1]*sin(self.yaw_trans_rad)) + (self.trans[1]*cos(self.yaw_trans_rad)))
        
        self.msg.header.stamp= data.header.stamp
        self.msg.pose.pose.position.x= xpos
        self.msg.pose.pose.position.y= ypos
        self.msg.pose.covariance[0] = location_cov[0] 
        self.msg.pose.covariance[7] = location_cov[4] 
        self.msg.pose.covariance[14] = location_cov[8]
        #self.pose_pub.publish(self.msg) # need to add orientation covariances!!

    def yaw_callback(self,data):
        #rospy.loginfo( "In yaw callback")

        yaw_x = data.orientation.x
        yaw_y = data.orientation.y
        yaw_z = data.orientation.z
        yaw_w = data.orientation.w
        input_quat = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        #orientation_input_data = quaternion_to_euler(yaw_x, yaw_y, yaw_z, yaw_w)
        orientation_input_data = tf.transformations.euler_from_quaternion(input_quat)# roll,pttch,yaw

#        print orientation_input_data[2]
        #gps_yaw_offset_rad = rospy.get_param('~gps_yaw_offset_rad')
        #self.yaw_trans_rad = orientation_input_data[2] + ref_theta_rad + (pi) + (self.rot[2]) - 12*pi/180 #((pi/2)+gps_yaw_offset_rad) # 
        self.yaw_trans_rad = orientation_input_data[2] - ref_theta_rad + pi + (self.rot[2]) #+ 12*pi/180 #((pi/2)+gps_yaw_offset_rad) # 
#        self.yaw_trans_rad = yaw_trans_deg*pi/180
#        print yaw_trans_deg
        #trans_quat = euler_to_quaternion(orientation_input_data[0], orientation_input_data[1], self.yaw_trans_rad)
        trans_quat = tf.transformations.quaternion_from_euler(orientation_input_data[0], orientation_input_data[1], self.yaw_trans_rad)# roll,pttch,yaw

        self.msg.pose.pose.orientation.z = trans_quat[2]
        self.msg.pose.pose.orientation.y = trans_quat[1]
        self.msg.pose.pose.orientation.x = trans_quat[0]
        self.msg.pose.pose.orientation.w = trans_quat[3]
        self.msg.pose.covariance[21] = 0.1
        self.msg.pose.covariance[28] = 0.1
        self.msg.pose.covariance[35] = 0.1
        self.pose_pub.publish(self.msg) # need to add orientation covariances!!
#        yaw_transform_rad = 
    
   

def main():
    rospy.init_node('gps_coord_transform', anonymous=True)
    gps_data_directory = rospy.get_param('~gps_log_directory')
    scaling_data = PA(gps_data_directory)                 # Obtain returned values from procrutes_analysis's function PA
    global Q      # Q is the rotation matrix
    Q = scaling_data[0]
    global ref_utm 
    ref_utm = scaling_data[1] 
    global ref_2D
    ref_2D = scaling_data[2]
    global ref_theta_rad
    ref_theta_rad = scaling_data[3]
    
    pm = GPS_to_pose_message()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down GPS to map pose conversion module"
    
if __name__ == '__main__':
    main()

