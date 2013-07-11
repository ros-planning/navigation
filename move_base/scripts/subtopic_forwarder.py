#!/usr/bin/env python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of Willow Garage, Inc. nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#*
#* Author: Eitan Marder-Eppstein
#***********************************************************
PKG = 'move_base'
NAME = 'subtopic_forwarder'

import rospy
import sys
import rostopic
from roslib.message import Message


class SubtopicForwarder(object):
    def __init__(self, topic, remapped_topic):
        topic_class, real_topic, msg_eval = rostopic.get_topic_class(topic)
        try:
            rospy.Subscriber(real_topic, topic_class, self.callback, (remapped_topic, msg_eval))
        except ValueError:
            pass

    def callback(self, data, args):
        remapped_topic, msg_eval = args

        #make sure to check if we have a subtopic or just a topic
        remapped_msg = msg_eval is not None and msg_eval(data) or data

        if not issubclass(type(remapped_msg), Message):
            rospy.logerr("Sorry, forwarding primitive types, in this case a %s, is not supported by this tool :(", type(remapped_msg))
        return

        remap_pub = rospy.Publisher(remapped_topic, type(remapped_msg))
        remap_pub.publish(remapped_msg)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print "You are uinsg this tool wrong, usage is:"
        print "subtopic_forwarder [source_topic] [destination_topic]"
        sys.exit(-1)

    rospy.init_node(NAME, anonymous=True)

    topic = rospy.resolve_name(sys.argv[1])
    remapped_topic = rospy.resolve_name(sys.argv[2])
    sf = SubtopicForwarder(topic, remapped_topic)

    rospy.spin()
