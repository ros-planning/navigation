#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""

@author: michael
"""
# Subscibe to map tf and amcl_quality and generate a heatmap of localisation quality


import matplotlib.pyplot as plt
import matplotlib.tri as tri
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import numpy as np
import tf
from scipy.interpolate import griddata
from mpl_toolkits.axes_grid1.axes_divider import make_axes_locatable



plt.rcParams['interactive'] 
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_title('Localisation quality')
ax.set(xlim=(-40, -5), ylim=(-60, -30))
cax = make_axes_locatable(ax).append_axes("right", size="5%", pad="2%")

plt.subplots_adjust(hspace=0.5)

class localisation_quality:
    
    def __init__(self):
        
        self.map_tf_listener = tf.TransformListener()
        
        self.data_array = np.empty((0,3), float)

        self.quality_subscriber = rospy.Subscriber('amcl_quality', Float64, self.amcl_quality_callback, queue_size = 10)

    
        
        
        
    def amcl_quality_callback(self, data):
        
        
        try:
            now = rospy.Time.now()
            self.map_tf_listener.waitForTransform('map', 'base_link', now, rospy.Duration(0.5))
            (map_position, map_orientation_quat) = self.map_tf_listener.lookupTransform('map', 'base_link', now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        
        
        
#        self.data_array.append([map_position[0], map_position[1], data.data])
        self.data_array = np.append(self.data_array, np.array([[map_position[0], map_position[1], data.data]]), axis=0)        
#        print "Array:", self.data_array  
        
        
        if len(self.data_array)%10 == 0:
            self.plot_quality()

    
    def plot_quality(self):
        ngridx = 100
        ngridy = 200
#        plot = ax.tricontour(self.data_array[:,0], self.data_array[:,1], self.data_array[:,2], linewidths=0.5, colors='k')
        cntr1 = ax.tricontourf(self.data_array[:,0], self.data_array[:,1], self.data_array[:,2], cmap="RdBu_r")
        fig.colorbar(cntr1, cax=cax)
        ax.plot(self.data_array[:,0], self.data_array[:,1], 'ko', ms=3)

        plt.show()
            
        
        

def main():
    rospy.init_node('localisation_quality_plotter', anonymous=True)

    quality = localisation_quality()
    
    
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        quality.plot_quality()
        print "Plotting data then shutting down localisation quality plotting module"
    
if __name__ == '__main__':
    main()

# -*- coding: utf-8 -*-

