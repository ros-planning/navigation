#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
@author: michael
@email: mhutchinson@sagarobotics.com
"""

"""
This script takes ground truth nmea strings from a gps log drectory and generates a
transformation from gps data into the map frame. 
"""

import rospy
import numpy as np
import os
import math
import rospkg
import utm
import libnmea_navsat_driver.parser
from cv2 import estimateAffine2D

class GGA_to_utm_to_map_transformation:
    

    def __init__(self, gps_data_directory):
        
        rospack = rospkg.RosPack()
        self.location = rospack.get_path("mel_amcl") + "/gps_logs/" + gps_data_directory + "/"
        rospy.loginfo("GPS log dir: %s", self.location)
        
        self.coords_map, self.txtfiles = self.read_files()
        self.coords_utm, self.utm_zone_num, self.utm_zone_let = self.get_coord_utm() #Corresponding utm coordinates
        self.ground_truth_error = self.get_ground_truth_error()
        
        if self.ground_truth_error < 0.1:
            rospy.loginfo("Average GPS ground truth error:  %f m", self.ground_truth_error)
        else:
            rospy.logwarn("Average GPS ground truth error is:  %f m, consider collecting new ground truth points!", self.ground_truth_error)

        self.transformation_matrix, self.theta_trans_rad = self.get_utm_transformation_matrix()

        self.datum_lat, self.datum_lon = self.get_datum()


    def read_files(self):
        
        if os.path.exists(self.location):        
            
            X = np.array([])
            Y = np.array([])
            txtfiles = []
            
            for file in os.listdir(self.location):
                try:
                    if file.endswith(".txt"):
                        txtfiles.append(self.location+str(file))
                        st = file.find('(')+1
                        md = file.find(" ")
                        en = file.find(')')
                        if st == -1 | md == -1 | en == -1:
                            rospy.logerr("Incorrect gps log file name - use format (x y) - remember a space!.")
                        X = np.append(X, file[st:md])
                        Y = np.append(Y, file[md+1:en])
                except Exception as e:
                    raise e
                    
            if not txtfiles:
                rospy.logerr("No GPS ground truth logs found in %s.", self.location)
#            if len(txtfiles) < 3:
#                rospy.logerr("Not enough GPS ground truth logs found in %s. Only found %i.", self.location, len(txtfiles))
                
            rospy.loginfo("Processing %i GPS ground truth logs found in %s.",len(txtfiles), self.location)
            
            X = np.array(X).astype(np.float)
            Y = np.array(Y).astype(np.float)

            coords_map = np.column_stack((X,Y))
            
            return coords_map, txtfiles
        
        else:
            rospy.logerr("Folder does not exist:  %s ", self.location)
        
        
        
    def get_coord_utm(self):

        utm_x_=np.array([])
        utm_y_=np.array([])
        
        for i in range(len(self.txtfiles)):
            latitudes = np.array([])
            longitudes = np.array([]) 
            gps_log_file = open(self.txtfiles[i],'r')
            line =  gps_log_file.readline()
        
            while line:
                parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(line)
                if not parsed_sentence:
                    line =  gps_log_file.readline()
                    continue
                
                if 'GGA' in parsed_sentence:
                    data = parsed_sentence['GGA']
                
                    latitude = data['latitude']
                    if data['latitude_direction'] == 'S':
                        latitude = -latitude
    
                    longitude = data['longitude']
                    if data['longitude_direction'] == 'W':
                        longitude = -longitude
                    
                    latitudes = np.append(latitude, latitudes)
                    longitudes = np.append(longitude, longitudes)
                
                line =  gps_log_file.readline()
        
            
            latitude_mean = np.mean(latitude)
            longitude_mean = np.mean(longitude)
        
            gps_log_file.close
        
            utmvals = utm.from_latlon(latitude_mean, longitude_mean)
    
            utm_x_ = np.append(utm_x_, utmvals[0])
            utm_y_ = np.append(utm_y_, utmvals[1])
            
            utm_zone_num = utmvals[2]
            utm_zone_let = utmvals[3]

            coords_utm = np.column_stack((utm_x_,utm_y_))
#            print 'utm coords_new: ', coords_utm
        return coords_utm, utm_zone_num, utm_zone_let
            
    
    
    def get_ground_truth_error(self):
        
        errors_utm_map = np.array([])
        errors_files = np.array([])
        num_files = len(self.txtfiles)
        
        for i in range(num_files):
            for j in range(num_files):
                
                if i == j: continue
                    
                dist_map = np.sqrt((self.coords_map[i,0]-self.coords_map[j,0])**2 + (self.coords_map[i,1]-self.coords_map[j,1])**2)
                dist_utm = np.sqrt((self.coords_utm[i,0]-self.coords_utm[j,0])**2 + (self.coords_utm[i,1]-self.coords_utm[j,1])**2)
                errors_utm_map = np.append(errors_utm_map, np.sqrt((dist_utm - dist_map)**2))
            errors_files = np.append(errors_files, np.mean(errors_utm_map[(num_files-1)*i:(num_files-1)*(i+1)]))
            print "Error log for file ",'{:35}'.format(os.path.relpath(self.txtfiles[i], self.location)), '{:>5}'.format('='), errors_files[i]
                
        average_gps_ground_truth_error =  np.mean(errors_utm_map)
            
        return average_gps_ground_truth_error
        
        
        
    def get_utm_transformation_matrix(self):
        
        transformation_matrix, inliers = estimateAffine2D(self.coords_utm, self.coords_map, confidence=0.99)
    
    
        theta_trans_rad = ( math.atan2(transformation_matrix[1,0],transformation_matrix[1,1]) + math.atan2(-transformation_matrix[0,1],transformation_matrix[0,0]) ) / 2
        theta_1 = math.atan2(transformation_matrix[1,0],transformation_matrix[1,1])
        theta_2 = math.atan2(-transformation_matrix[0,1],transformation_matrix[0,0])
        print "Theta 1 : ", theta_1
        print "Theta 2 : ", theta_2
        
        
        theta_inconsitency = abs(theta_2 - theta_1)
        if theta_inconsitency > 0.05:
            rospy.logwarn("Error between yaw estimates is %f radians, are gps logs accurate?", theta_inconsitency )
            
#        theta_trans_rad = np.mean([math.acos(transformation_matrix[0,0]),math.acos(transformation_matrix[1,1]),-math.asin(transformation_matrix[0,1]),math.asin(transformation_matrix[1,0])])

        return transformation_matrix, theta_trans_rad
        

    def get_datum(self):

        a = np.array([ [self.transformation_matrix[0,0], self.transformation_matrix[0,1]], [self.transformation_matrix[1,0], self.transformation_matrix[1,1]] ])
        b = np.array([-self.transformation_matrix[0,2], -self.transformation_matrix[1,2]])
        datum_utm_x, datum_utm_y = np.linalg.solve(a, b)
        
        datum_lat, datum_lon = utm.to_latlon(datum_utm_x, datum_utm_y, self.utm_zone_num, self.utm_zone_let)
        
        return datum_lat, datum_lon
        
            
        

    
def main():
    
    getUtm = GGA_to_utm_to_map_transformation("norway")
    print "Transformation matrix : \n", getUtm.transformation_matrix
    print "Theta deg: ", math.degrees(getUtm.theta_trans_rad)
    getUtm = GGA_to_utm_to_map_transformation("riseholme")

    
    print "Transformation matrix : \n", getUtm.transformation_matrix
    print "Theta deg: ", math.degrees(getUtm.theta_trans_rad)
    


if __name__ == '__main__':
    main()











