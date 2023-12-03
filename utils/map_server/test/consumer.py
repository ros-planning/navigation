#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
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
#

from __future__ import print_function

PKG = 'static_map_server'
NAME = 'consumer'

import sys
import unittest
import time

import rospy
import rostest
from nav_msgs.srv import GetMap


class TestConsumer(unittest.TestCase):
    def __init__(self, *args):
        super(TestConsumer, self).__init__(*args)
        self.success = False

    def callback(self, data):
        print(rospy.get_caller_id(), "I heard %s" % data.data)
        self.success = data.data and data.data.startswith('hello world')
        rospy.signal_shutdown('test done')

    def test_consumer(self):
        rospy.wait_for_service('static_map')
        mapsrv = rospy.ServiceProxy('static_map', GetMap)
        resp = mapsrv()
        self.success = True
        print(resp)
        while not rospy.is_shutdown() and not self.success:  # and time.time() < timeout_t: <== timeout_t doesn't exists??
            time.sleep(0.1)
        self.assert_(self.success)
        rospy.signal_shutdown('test done')

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestConsumer, sys.argv)
