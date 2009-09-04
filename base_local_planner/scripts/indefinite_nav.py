#! /usr/bin/python

import roslib
roslib.load_manifest('base_local_planner')

import sys, time, traceback, logging, rospy, random
from test_msgs.msg import Planner2DGoal
from test_msgs.msg import Planner2DState

NAME = 'indefinite_nav'

#goals = [
#[19.25, 26.37, 3.00]
#[27.760, 33.166, -0.036]
#  ]

goals = [
 [50.250, 6.863, 3.083], 
 [18.550, 11.762, 2.554],
 [53.550, 20.163, 0.00],
 [18.850, 28.862, 0.00],
 [47.250, 39.162, 1.571],
 [11.450, 39.662, 0.00]
 ]

chrg_stations = [
 [33.844, 36.379, -1.571]
]

first = True

def indefinite_nav():
  def callback(state):
    send_goal(0)

  def send_goal(done):
    global goals
    global first
    if first or done == 1:
      goal_pts = goals[random.randint(0, len(goals) - 1)]
      goal = Planner2DGoal()
      goal.header.frame_id = "map"
      goal.goal.x = goal_pts[0]
      goal.goal.y = goal_pts[1]
      goal.goal.th = goal_pts[2]
      goal.enable = 1
      goal.timeout = 1000.0
      first = False
      print "New Goal: x: %.2f, y: %.2f, th: %.2f" % (goal.goal.x, goal.goal.y, goal.goal.th)
      pub.publish(goal)

  rospy.Subscriber("state", Planner2DState, callback)
  pub = rospy.Publisher("goal", Planner2DGoal)
  rospy.init_node(NAME, anonymous=True)
  rospy.spin()

if __name__ == '__main__':
  try:
    indefinite_nav()
  except KeyboardInterrupt, e:
    pass
  print "exiting"

