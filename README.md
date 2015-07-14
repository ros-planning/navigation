ROS Navigation Stack (Mayfield version)
====================

Version: 0.1
Maintainer: Kaijen Hsiao (kaijen@mayfieldrobotics.com)

A 2D navigation stack that takes in information from odometry, sensor
streams, and a goal pose and outputs safe velocity commands that are sent
to a mobile base.

To test pull requests before merging, run the following:

```
roslaunch turtlebot_gazebo turtlebot_playground.launch
```

In a second terminal, run our custom nav stack (Mayfield folks only):
```
roslaunch gizmo_ros_navigation navigation_demo.launch sim:=true
```

The equivalent inside this stack, which unfortunately doesn't have very good params (not needed if you did the last launch), is: 
```roslaunch turtlebot_gazebo amcl_demo.launch```

Finally, run rviz with an appropriate config file:
```
sudo apt-get install ros-indigo-turtlebot-interactions
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

Use the 2D Nav Goal button in rviz to give the robot navigation goals.

