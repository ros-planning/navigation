#!/usr/bin/env python

#----------------------------------
# @author: Vicky
# @email: engr.electron@gmail.com
# ----------------------------------

# This script takes in the northing and easting from GGA_to_utm.py, performs Procrustes Analysis, i.e., calculates the rotation matrix Q,
# and assign a reference point (ref_utm and ref_2D). Note: any point can be taken as refernce point, here the first one is taken as reference hence,
# ref_utm = utm_data[0,:] and ref_2D=twod_data[0,:]

import numpy as np
from numpy import *
from GGA_to_utm_norway import get_coord_utm
from math import atan
from math import asin
from math import acos
from math import pi
#from peolple_location import test
#############################################
########  rotation_matrix = Q_t/Q_r  ########
#############################################
def PA(gps_data_directory):
    data = get_coord_utm(gps_data_directory)
    utm_data = data[0]
    twod_data= data[1]
#    sprint data
    ref_utm=utm_data[0,:]            #Choose ref point on on utm_map (choose first one for simplicity)
    ref_2D=twod_data[0,:]      #Choose reference point on 2D map (Choose first one for simplicity)
    Q_r = utm_data - ref_utm         #Denomenator  for calculating rotation matrix
    Q_t = twod_data - ref_2D   #Numerator for calculating rotation matrix
    Q_t= np.delete(Q_t,np.where(~Q_t.any(axis=1))[0], axis=0)      #Remove zero row from Numerator
    Q_r= np.delete(Q_r,np.where(~Q_r.any(axis=1))[0], axis=0)      #Remove zero column from denominator
    Q_t=np.transpose(Q_t)                                          #
    Q_r=np.transpose(Q_r)                                          #
    Q_=np.linalg.pinv(Q_r)                                         
    rotation_matrix=np.matmul(Q_t,Q_)         
    ref_2D = ref_2D[:,None]      #To convert 1x2 array into 2x1 array. Note: np.transpose does not work on 1 dimensional array
    ref_utm = ref_utm[:,None]    #To convert 1x2 array into 2x1 array. Note: np.transpose does not work on 1 dimensional array    
    
    other_ref_utm = utm_data[1,:] # choose another ref point on utm to find angle difference with 2d map
    other_ref_2D = twod_data[1,:] # choose another ref point on 2d map to find angle difference with utm
    dx = ref_2D[0] - other_ref_2D[0]
    dy = ref_2D[1] - other_ref_2D[1]
    de = ref_utm[0] - other_ref_utm[0]
    dn = ref_utm[1] - other_ref_utm[1]
    a=de**2
    b=dn**2
    nedist = (a+b)**(0.5)
    c=dx**2
    d=dy**2
    xydist = (c+d)**(0.5)
    error = ((nedist-xydist)**2)**0.5
    print 'projection error', error
    theta = atan(dx/dy)-atan(de/dn)
    ans = theta*180/pi
    print 'rotation_matrix', rotation_matrix
    theta = np.mean([acos(rotation_matrix[0,0]),acos(rotation_matrix[1,1]),-asin(rotation_matrix[0,1]),asin(rotation_matrix[1,0])])
    ans = theta*180/pi
    print ans
   # ref_heading
    
    return (rotation_matrix, ref_utm, ref_2D, theta)

    
