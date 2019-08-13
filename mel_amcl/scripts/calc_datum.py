#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 12 2019

@author: michael


This script subsribes to a navsatfix message and outputs the latitude and longitude accounting for the gps tf offset from base_link.

"""

import rospy
import tf
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import utm
from math import sin
from math import cos




class GetDatum:
    
    def __init__(self):

        rospy.Subscriber('/gps/fix', NavSatFix, self.calc_datum, queue_size = 10)

        self.got_tf = False
        self.map_tf_listener = tf.TransformListener()



    def calc_datum(self, data):
    
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

        utm_coord = utm.from_latlon(lat, lon)

    
        xpos = utm_coord[0]  - ( (self.trans[0]) )
        ypos = utm_coord[1]  - ( self.trans[1])
            
        lat, lon =  utm.to_latlon(xpos, ypos, utm_coord[2], utm_coord[3])
        print 'Datum is: ', lat, lon




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
    
    rospy.init_node('calc_datum_gps_navsatfix', anonymous=True)
    GetDatum()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Calculate datum module"
    



if __name__ == '__main__':
    main()

