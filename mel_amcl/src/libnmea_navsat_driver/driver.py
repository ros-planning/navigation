# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

import rospy

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
import libnmea_navsat_driver.parser

from tf.transformations import quaternion_about_axis


class RosNMEADriver(object):
    def __init__(self):
        self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)
        self.vel_pub = rospy.Publisher('vel', TwistStamped, queue_size=1)
        self.yaw_pub = rospy.Publisher('yaw', Imu, queue_size=1)
        self.time_ref_pub = rospy.Publisher('time_reference', TimeReference, queue_size=1)

        self.time_ref_source = rospy.get_param('~time_ref_source', None)
        self.use_RMC = rospy.get_param('~useRMC', False)

        # dual antenna stuff
        # dx and dy is as seen from master antenna that is at the rear of the "heading arrow"
        dx_antennas = rospy.get_param('~dx_antennas', 1.033463)
        dy_antennas = rospy.get_param('~dy_antennas', 0)
        self.heading_offset = math.atan2(dy_antennas,dx_antennas)
        print "Heading offset is"
        print self.heading_offset

    # Returns True if we successfully did something with the passed in
    # nmea_string
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        if not check_nmea_checksum(nmea_string):
            rospy.logwarn("Received a sentence with an invalid checksum. " +
                          "Sentence was: %s" % repr(nmea_string))
            return False

        parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            rospy.logdebug("Failed to parse NMEA sentence. Sentece was: %s" % nmea_string)
            return False

        if timestamp:
            current_time = timestamp
        else:
            current_time = rospy.get_rostime()
        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        current_time_ref = TimeReference()
        current_time_ref.header.stamp = current_time
        current_time_ref.header.frame_id = frame_id
        if self.time_ref_source:
            current_time_ref.source = self.time_ref_source
        else:
            current_time_ref.source = frame_id

        if not self.use_RMC and 'GGA' in parsed_sentence:
            print "GGA!"
            data = parsed_sentence['GGA']
            gps_qual = data['fix_type']
            if gps_qual == 0:
                current_fix.status.status = NavSatStatus.STATUS_NO_FIX
            elif gps_qual == 1:
                current_fix.status.status = NavSatStatus.STATUS_FIX
            elif gps_qual == 2:
                current_fix.status.status = NavSatStatus.STATUS_SBAS_FIX
            elif gps_qual in (4, 5):
                current_fix.status.status = NavSatStatus.STATUS_GBAS_FIX
            else:
                current_fix.status.status = NavSatStatus.STATUS_NO_FIX

            current_fix.status.service = NavSatStatus.SERVICE_GPS

            current_fix.header.stamp = current_time

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            hdop = data['hdop']
            current_fix.position_covariance[0] = hdop ** 2
            current_fix.position_covariance[4] = hdop ** 2
            current_fix.position_covariance[8] = (2 * hdop) ** 2  # FIXME
            current_fix.position_covariance_type = \
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            self.fix_pub.publish(current_fix)

            if not math.isnan(data['utc_time']):
                current_time_ref.time_ref = rospy.Time.from_sec(data['utc_time'])
                self.time_ref_pub.publish(current_time_ref)

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(current_fix)

                if not math.isnan(data['utc_time']):
                    current_time_ref.time_ref = rospy.Time.from_sec(data['utc_time'])
                    self.time_ref_pub.publish(current_time_ref)

            # Publish velocity from RMC regardless, since GGA doesn't provide it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * \
                    math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * \
                    math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)

        elif 'HDT' in parsed_sentence:
            data = parsed_sentence['HDT']
            print "HDT!"
            #print "data is: "
            #print data['heading_deg']
            #print data['T']
            #print 'DATA: '
            #print data['heading_deg']/math.pi*180

            #NED#
            heading_ned = self.normalize_angle(data['heading_deg'] + self.heading_offset)
            print "NED"
            print heading_ned/math.pi*180
            #ENU#
            heading_enu = 2*math.pi - self.normalize_angle(heading_ned + 3*math.pi/2)
            print "ENU"
            print heading_enu/math.pi*180
            
            current_yaw = Imu()
            current_yaw.header.stamp = current_time
            current_yaw.header.frame_id = frame_id
            orientation_quat = quaternion_about_axis(heading_enu, (0,0,1))
            current_yaw.orientation.x = orientation_quat[0]
            current_yaw.orientation.y = orientation_quat[1]
            current_yaw.orientation.z = orientation_quat[2]
            current_yaw.orientation.w = orientation_quat[3]
            current_yaw.orientation_covariance[0] = 0.01
            current_yaw.orientation_covariance[4] = 0.01
            current_yaw.orientation_covariance[8] = 0.01  # FIXME
            if not math.isnan(current_yaw.orientation.x) :
                self.yaw_pub.publish(current_yaw)
        else:
            return False

    """Helper method for getting the frame_id with the correct TF prefix"""

    #MODIFIED 
    @staticmethod
    def get_frame_id():
        frame_id = rospy.get_param('~frame_id', 'gps')
#L        if frame_id[0] != "/":
#A            """Add the TF prefix"""
#R            prefix = ""
#S            prefix_param = rospy.search_param('tf_prefix')
#L            if prefix_param:
#A                prefix = rospy.get_param(prefix_param)
#R                if prefix[0] != "/":
#S                    prefix = "/%s" % prefix
#            return "%s/%s" % (prefix, frame_id)
#        else:
        return frame_id
    
    @staticmethod
    def normalize_angle(angle_in):
        while angle_in > 2*math.pi:
            angle_in = angle_in - 2*math.pi
        while angle_in < 0:
            angle_in = angle_in + 2*math.pi
        return angle_in
