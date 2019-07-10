#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 10:18:55 2019

@author: michael
"""

# This script monitors the discrepencies between the GPS data and AMCL data.
# To avoid AMCL diverging, when the discrepency is large, the GPS data is used
# to reinitialise AMCL.
# NOTE: This is only suitable when we can trust GPS data, but want AMCL to
# refine pose estimate.
# The script should be extended to do, somewhat, the reverse i.e.:
# If AMCL is confident, i.e. the scan match is high (The evidence term in
# Bayes Equ - needs modification of AMCL to get published) then GPS data can
# be ignored as the robot is probably indoors and GPS poor. This may cause problems 
# in long polytunnels where AMCL wont actually be that useful (despite maybe a
# good scan match) if lasers dont see the other end of the tunnel.
# The script should also be extended to consider the covariances from AMCL and 
# GPS and consider other sources of information.

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovariance
import numpy as np
#from scipy.spatial import distance

#
#class monitor:
#    
#    def __init__(self):
#        '''Initialize ros publisher, ros subscriber'''
#        # topic where we publish
#        self.msg = PoseWithCovarianceStamped()
#        self.msg.header.frame_id='map'
#        self.pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
#        # subscribed Topics
#        self.amcl_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback, queue_size = 10)
#        self.gps_subscriber = rospy.Subscriber("/gps_transform",PoseWithCovarianceStamped, self.gps_callback, queue_size = 10)
#        self.fused_pose_subscriber = rospy.Subscriber("/rl/map_pose", Odometry, self.fused_pose_callback, queue_size = 10)
#        
#        
#        self.data_analysis()
#        self.pose_pub.publish(self.msg)
#
#
#
#        
#    def amcl_callback(self,data):
#        self.amcl_2d_pose = [data.pose.pose.position.x, data.pose.pose.position.y ]
#        print self.amcl_2d_pose
#        
#        
#        
#    def gps_callback(self,data):
#        self.gps_2d_pose = [data.pose.pose.position.x, data.pose.pose.position.y ]
#        print self.gps_2d_pose
#        
#        
#    def fused_pose_callback(self,data):
#        self.fused_2d_pose = [data.pose.pose.position.x, data.pose.pose.position.y ]
#        print self.fused_2d_pose
#        
#        
#    def data_analysis(self):
#        print "hi"
#        rospy.spin()
#
#def main():
#    rospy.init_node('localisation_monitor', anonymous=True)
#    pm = monitor()
#    try:
#        rospy.spin()
#    except KeyboardInterrupt:
#        print "Shutting down GPS to map pose conversion module"
#    
#if __name__ == '__main__':
#    main()
#    

discrepency_threshold = 0.5
    
class Monitor:
    
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.msg = PoseWithCovarianceStamped()
        self.msg.header.frame_id='map'
        self.pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)

        self.amcl_2d_pose = None
        self.gps_2d_pose = None
        self.fused_2d_pose = None
        self.amcl_data = None
        self.gps_data = None 
        
    def amcl_callback(self,data):
        self.amcl_2d_pose = np.array([data.pose.pose.position.x, data.pose.pose.position.y ])
        print self.amcl_2d_pose
        self.amcl_data = data
        self.data_analysis()
        
        
    def gps_callback(self,data):
        self.gps_2d_pose =  np.array([data.pose.pose.position.x, data.pose.pose.position.y ])
        self.gps_data = data
#        print self.gps_2d_pose
        
        
    def fused_pose_callback(self,data):
        self.fused_2d_pose =  np.array([data.pose.pose.position.x, data.pose.pose.position.y ])
#        print self.fused_2d_pose
        
        
    def data_analysis(self):
        dist = np.linalg.norm(np.array([self.gps_data.pose.pose.position.x, self.gps_data.pose.pose.position.y])-np.array([self.amcl_data.pose.pose.position.x, self.amcl_data.pose.pose.position.y]))
        if dist > discrepency_threshold:
            print "such far away"
            self.msg.header.stamp= self.gps_data.header.stamp
            self.msg.pose = self.gps_data.pose
            print self.gps_data.pose.covariance[35]
            self.msg.pose.covariance = list(self.msg.pose.covariance)
            self.msg.pose.covariance[35] = 0.07
            self.msg.pose.covariance = tuple(self.msg.pose.covariance)
#            self.msg.pose.covariance[35] = 0.07 # approximated covariance from GPS heading, should be subscribed to
            self.pose_pub.publish(self.msg)
            
        print dist


    
if __name__ == '__main__':
    rospy.init_node('localisation_monitor', anonymous=True)
    monitor = Monitor()
    
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, monitor.amcl_callback, queue_size = 10)
    rospy.Subscriber("/gps_transform",PoseWithCovarianceStamped, monitor.gps_callback, queue_size = 10)
    rospy.Subscriber("/rl/map_pose", Odometry, monitor.fused_pose_callback, queue_size = 10)
    rospy.spin()


        


    