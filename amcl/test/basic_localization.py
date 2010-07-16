#!/usr/bin/env python

import roslib
roslib.load_manifest('amcl')

import sys
import time
import math
from math import fmod,pi

import unittest
import rospy
import rostest

from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty

class TestBasicLocalization(unittest.TestCase):

  def setUp(self):
    self.tf = None
    self.target_x = None
    self.target_y = None
    self.target_a = None

  def tf_cb(self, msg):
    for t in msg.transforms:
      if t.header.frame_id == '/map':
        self.tf = t.transform
        (a_curr, a_diff) = self.compute_angle_diff()
        print 'Curr:\t %16.6f %16.6f %16.6f' % (self.tf.translation.x, self.tf.translation.y, a_curr)
        print 'Target:\t %16.6f %16.6f %16.6f' % (self.target_x, self.target_y, self.target_a)
        print 'Diff:\t %16.6f %16.6f %16.6f' % (abs(self.tf.translation.x-self.target_x),abs(self.tf.translation.y - self.target_y), a_diff)

  def compute_angle_diff(self):
    rot = self.tf.rotation
    a = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])[2]
    d_a = self.target_a

    return ( a, abs(fmod(a - d_a + 5*pi, 2*pi) - pi) )
    

  def test_basic_localization(self):
    global_localization = int(sys.argv[1])
    self.target_x = float(sys.argv[2])
    self.target_y = float(sys.argv[3])
    self.target_a = float(sys.argv[4])
    tolerance_d = float(sys.argv[5])
    tolerance_a = float(sys.argv[6])
    target_time = float(sys.argv[7])

    if global_localization == 1:
      #print 'Waiting for service global_localization'
      rospy.wait_for_service('global_localization')
      global_localization = rospy.ServiceProxy('global_localization', Empty)
      resp = global_localization()

    rospy.init_node('test', anonymous=True)
    while(rospy.rostime.get_time() == 0.0):
      #print 'Waiting for initial time publication'
      time.sleep(0.1)
    start_time = rospy.rostime.get_time()
    # TODO: This should be replace by a pytf listener
    rospy.Subscriber('/tf', tfMessage, self.tf_cb)

    while (rospy.rostime.get_time() - start_time) < target_time:
      #print 'Waiting for end time %.6f (current: %.6f)'%(target_time,(rospy.rostime.get_time() - start_time))
      time.sleep(0.1)

    (a_curr, a_diff) = self.compute_angle_diff()
    print 'Curr:\t %16.6f %16.6f %16.6f' % (self.tf.translation.x, self.tf.translation.y, a_curr)
    print 'Target:\t %16.6f %16.6f %16.6f' % (self.target_x, self.target_y, self.target_a)
    print 'Diff:\t %16.6f %16.6f %16.6f' % (abs(self.tf.translation.x-self.target_x),abs(self.tf.translation.y - self.target_y), a_diff)
    self.assertNotEquals(self.tf, None)
    self.assertTrue(abs(self.tf.translation.x - self.target_x) <= tolerance_d)
    self.assertTrue(abs(self.tf.translation.y - self.target_y) <= tolerance_d)
    self.assertTrue(a_diff <= tolerance_a)

if __name__ == '__main__':
  rostest.run('amcl', 'amcl_localization', 
              TestBasicLocalization, sys.argv)
