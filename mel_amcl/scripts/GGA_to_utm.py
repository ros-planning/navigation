#!/usr/bin/env python
#import rospy

#----------------------------------
# @author: Vicky
# @email: engr.electron@gmail.com
# ----------------------------------

# This script takes readings from gps log files in the gps_log folder, finds out the weighted average lat/long for each file. These are then converted to degree format from hour/mins/secs format
# before feeding it to pyproj which converts it into northing and easting (coordinates in utm format). These northings and eatings are in a world frame (will work with Google Earth).

import numpy as np
import os
import math
from pyproj import Proj
import rospkg
import utm


def get_coord_utm(gps_data_directory):
    rospack = rospkg.RosPack()
    location = rospack.get_path("mel_amcl") + "/gps_logs/" + gps_data_directory + "/" #location = rospack.get_path("PACKAGE_NAME") + "/FOLDER_NAME/" 
    print "gps loc dir: ", location
#    location = "gps_logs/"
    counter = 0
    txtfiles = []
    twod_coordinates = [] #Coordinates on 2D map
    X=np.array([])
    Y=np.array([])
    utm_x=np.array([])
    utm_y=np.array([])
    for file in os.listdir(location):
        try:
            if file.endswith(".txt"):
                txtfiles.append(location+str(file))
                st=file.find('(')+1
                md=file.find(" ")
                en=file.find(')')
                X=np.append(file[st:md],X)
                Y=np.append(file[md+1:en],Y)
                counter=counter+1
        except Exception as e:
            raise e
            print('No files found') 
    X=np.array(X).astype(np.float)
    Y=np.array(Y).astype(np.float)
    XXX = X[::-1]
    YYY = Y[::-1]
    twod_coordinates=np.column_stack((XXX,YYY))
    
    for i in range(0,counter):
        latitude = np.array([])
        longitude = np.array([]) 
        lat_err = np.array([])
        lon_err = np.array([])
        test = open(txtfiles[i],'r')
        line =  test.readline()

        while line:
            a = line.find('$GPGGA')
            b = line.find('$GPGST')
            if a == 0:               #True only if line starts with $GPGGA
                latitude=np.append(line[17:30],latitude) # had to change from 30 as precision changed
                longitude=np.append(line[33:46],longitude) # had to change from 46 as precision changed
                if line[47] == 'W':
                    west_hem=1    #flag to identify west hemisphere (longitude must be multiplied with -1 after converting to degrees)
                    ##   Weights ##
                else:
                    west_hem=0

            if b == 0:
                lat_err = np.append(line[-22:-17],lat_err)
                lon_err = np.append(line[-16:-11],lon_err) 
            line = test.readline()  #Read next line  
        latitude=np.array(latitude).astype(np.float)   #convert string to float
        longitude=np.array(longitude).astype(np.float) #convert string to float
        # print 'lat_err1', lat_err
        # lat_err = np.array(lat_err).astype(np.float)
        # lon_err = np.array(lon_err).astype(np.float)
        # print 'lat_err2', lat_err
        # w_lat = np.multiply(latitude,lat_err)
        # w_lon = np.multiply(longitude,lon_err)
        # latitude = np.sum(w_lat)/np.sum(lat_err)
        # longitude = np.sum(w_lon)/np.sum(lon_err)
        latitude = np.mean(latitude)
        longitude = np.mean(longitude)
        if west_hem == 1:
                    longitude = -1*longitude
    ####### Convert GGA into mdegrees  ##########
        gps_lat=latitude/100     
        gps_long=longitude/100
        lat_frac, lat_whole = math.modf(gps_lat)  # separate fraction portion from whole portion. whole will be the  degrees
        long_frac, long_whole = math.modf(gps_long)  # separate fraction portion from whole portion. whole will be the degrees
        lat_frac_deg=(lat_frac/60)*100         #calculating the decimal portion of degrees
        long_frac_deg=(long_frac/60)*100       #calculating the decimal portion of degrees
        print ' lat and lon = ', gps_lat, gps_long
        latitude_degrees = lat_whole+lat_frac_deg
        longitude_degrees = long_whole+long_frac_deg 
        print ' lat and lon = ', latitude_degrees, longitude_degrees
#        myProj = Proj("+proj=utm +zone=30, +east +ellps=WGS72 +datum=WGS84 +units=m +no_defs +a=6378134.0")
        myProj = Proj("+proj=utm +zone=30, +east +ellps=WGS84 +datum=WGS84 +units=m")
#        myProj = Proj("+proj=utm +zone=32, +east +ellps=WGS84 +datum=WGS84 +units=m")
        utmvals = utm.from_latlon(latitude_degrees, longitude_degrees)
        print 'utm', utmvals[0], utmvals[1]
        #x, y = myProj(longitude_degrees, latitude_degrees)
        #print 'utm from proj', x, y
        x = utmvals[0]
        y = utmvals[1]
        utm_x=np.append(x,utm_x)
        utm_y=np.append(y,utm_y)
        
        utm_xx = utm_x[::-1]
        utm_yy = utm_y[::-1]
        coord_utm=np.column_stack((utm_xx,utm_yy))
        test.close
        print 'utm coords: ', coord_utm
        print 'two d coord: ', twod_coordinates
    return (coord_utm, twod_coordinates)
    
