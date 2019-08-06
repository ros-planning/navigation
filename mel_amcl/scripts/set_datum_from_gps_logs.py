#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Aug  5 11:03:47 2019

@author: michael
"""

import parse_gps_logs
import rospy
import tf
from robot_localization.srv import SetDatum
from geographic_msgs.msg import GeoPose
from math import pi


def get_datum_msg(lat, lon, thata_rad):
    
#    theta_quat = tf.transformations.quaternion_from_euler(0, 0, -thata_rad-(1.1*pi/180))# roll,pitch,yaw
    theta_quat = tf.transformations.quaternion_from_euler(0, 0, -thata_rad-(2*pi/180))# roll,pitch,yaw

    datum = GeoPose()
    datum.position.latitude = lat
    datum.position.longitude = lon
    datum.position.altitude = 0
    datum.orientation.x = theta_quat[0]
    datum.orientation.y = theta_quat[1]
    datum.orientation.z = theta_quat[2]
    datum.orientation.w = theta_quat[3]
    return datum

def main():
    
    rospy.init_node('set_datum_from_gps_logs', anonymous=True)
    
#    gps_data_directory = rospy.get_param('~gps_log_directory')
    gps_data_directory = 'riseholme'

    transformation_from_logs = parse_gps_logs.GGA_to_utm_to_map_transformation(gps_data_directory)
    
    
    datum_msg = get_datum_msg(transformation_from_logs.datum_lat, transformation_from_logs.datum_lon, transformation_from_logs.theta_trans_rad)
    
    print 'Setting datum as: \n', datum_msg
    
    rospy.wait_for_service('datum')
    try:
        geo_pose = rospy.ServiceProxy('datum', SetDatum)
        response = geo_pose(datum_msg)
        print response
    except rospy.ServiceException, e:
        rospy.loginfo ("Service call failed: %s"%e)


if __name__ == '__main__':
    main()

