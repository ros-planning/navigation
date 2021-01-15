#!/usr/bin/env python

from __future__ import print_function

import sys
import time
from math import fmod, pi

import unittest
import rospy
import rostest

import tf2_py as tf2
import tf2_ros
import PyKDL
from std_srvs.srv import Empty


class TestBasicLocalization(unittest.TestCase):
    def setUp(self):
        self.tf = None
        self.target_x = None
        self.target_y = None
        self.target_a = None
        self.tfBuffer = None

    def print_pose_diff(self):
        a_curr = self.compute_angle()
        a_diff = self.wrap_angle(a_curr - self.target_a)
        print('Curr:\t %16.6f %16.6f %16.6f' % (self.tf.translation.x, self.tf.translation.y, a_curr))
        print('Target:\t %16.6f %16.6f %16.6f' % (self.target_x, self.target_y, self.target_a))
        print('Diff:\t %16.6f %16.6f %16.6f' % (
            abs(self.tf.translation.x - self.target_x), abs(self.tf.translation.y - self.target_y), a_diff))

    def get_pose(self):
        try:
            tf_stamped = self.tfBuffer.lookup_transform("map", "base_link", rospy.Time())
            self.tf = tf_stamped.transform
            self.print_pose_diff()
        except (tf2.LookupException, tf2.ExtrapolationException):
            pass

    @staticmethod
    def wrap_angle(angle):
        """
        Wrap angle to [-pi, pi)
        :param angle: Angle to be wrapped
        :return: wrapped angle
        """
        angle += pi
        while angle < 0:
            angle += 2*pi
        return fmod(angle, 2*pi) - pi

    def compute_angle(self):
        rot = self.tf.rotation
        a_curr = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w).GetRPY()[2]
        a_diff = self.wrap_angle(a_curr - self.target_a)
        return self.target_a + a_diff

    def test_basic_localization(self):
        global_localization = int(sys.argv[1])
        self.target_x = float(sys.argv[2])
        self.target_y = float(sys.argv[3])
        self.target_a = self.wrap_angle(float(sys.argv[4]))
        tolerance_d = float(sys.argv[5])
        tolerance_a = float(sys.argv[6])
        target_time = float(sys.argv[7])

        rospy.init_node('test', anonymous=True)
        while rospy.rostime.get_time() == 0.0:
            print('Waiting for initial time publication')
            time.sleep(0.1)

        if global_localization == 1:
            print('Waiting for service global_localization')
            rospy.wait_for_service('global_localization')
            global_localization = rospy.ServiceProxy('global_localization', Empty)
            global_localization()

        start_time = rospy.rostime.get_time()
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        while (rospy.rostime.get_time() - start_time) < target_time:
            print('Waiting for end time %.6f (current: %.6f)' % (target_time, (rospy.rostime.get_time() - start_time)))
            self.get_pose()
            time.sleep(0.1)

        print("Final pose:")
        self.get_pose()
        a_curr = self.compute_angle()

        self.assertNotEqual(self.tf, None)
        self.assertAlmostEqual(self.tf.translation.x, self.target_x, delta=tolerance_d)
        self.assertAlmostEqual(self.tf.translation.y, self.target_y, delta=tolerance_d)
        self.assertAlmostEqual(a_curr, self.target_a, delta=tolerance_a)


if __name__ == '__main__':
    rostest.run('amcl', 'amcl_localization', TestBasicLocalization, sys.argv)