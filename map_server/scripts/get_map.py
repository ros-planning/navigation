#!/usr/bin/env python
import rospy
from nav_msgs.msg import GetMapAction, GetMapGoal
from actionlib import SimpleActionClient

rospy.init_node('get_map', anonymous=True)

rospy.loginfo('Connecting to server')
ac = SimpleActionClient('get_static_map', GetMapAction)
ac.wait_for_server()

rospy.loginfo('Calling action to get map')
ac.send_goal(GetMapGoal())

rospy.loginfo('Waiting for result')
ac.wait_for_result()

result = ac.get_result()
print result.map.info
