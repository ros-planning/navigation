#!/usr/bin/env python

import roslib
roslib.load_manifest('amcl')

import sys
import time
import math

import unittest
import rospy
import rostest

from tf.msg import tfMessage
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
        print 'Curr:\t %16.6f %16.6f' % (self.tf.translation.x, self.tf.translation.y)
        print 'Target:\t %16.6f %16.6f' % (self.target_x, self.target_y)
        print 'Diff:\t %16.6f %16.6f' % (abs(self.tf.translation.x-self.target_x),abs(self.tf.translation.y - self.target_y))

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
    rospy.Subscriber('tf_message', tfMessage, self.tf_cb)

    while (rospy.rostime.get_time() - start_time) < target_time:
      #print 'Waiting for end time %.6f (current: %.6f)'%(target_time,(rospy.rostime.get_time() - start_time))
      time.sleep(0.1)
    print 'Curr:\t %16.6f %16.6f' % (self.tf.translation.x, self.tf.translation.y)
    print 'Target:\t %16.6f %16.6f' % (self.target_x, self.target_y)
    print 'Diff:\t %16.6f %16.6f' % (abs(self.tf.translation.x-self.target_x),abs(self.tf.translation.y - self.target_y))
    self.assertNotEquals(self.tf, None)
    self.assertTrue(abs(self.tf.translation.x - self.target_x) <= tolerance_d)
    self.assertTrue(abs(self.tf.translation.y - self.target_y) <= tolerance_d)
    #TODO: Check orientation

if __name__ == '__main__':
  rostest.run('amcl', 'amcl_localization', 
              TestBasicLocalization, sys.argv)
