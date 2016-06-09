^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package base_local_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.12.11 (2016-06-08)
--------------------

1.12.10 (2016-05-27)
--------------------

1.12.9 (2016-05-26)
-------------------

1.12.8 (2016-05-16)
-------------------

1.12.7 (2016-01-05)
-------------------

1.12.6 (2016-01-02)
-------------------

1.12.5 (2015-10-29)
-------------------
* base_local_planner: some fixes in goal_functions
* Contributors: Gael Ecorchard

1.12.4 (2015-06-03)
-------------------
* fix stuck_left/right calculation
* Contributors: Michael Ferguson

1.12.3 (2015-04-30)
-------------------

1.12.2 (2015-03-31)
-------------------

1.12.1 (2015-03-14)
-------------------

1.12.0 (2015-02-04)
-------------------
* update maintainer email
* Contributors: Michael Ferguson

1.11.15 (2015-02-03)
--------------------
* Add ARCHIVE_DESTINATION for static builds
* Contributors: Gary Servin

1.11.14 (2014-12-05)
--------------------
* Fixed setting child_frame_id in base_local_planner::OdometryHelperRos
* Contributors: Mani Monajjemi

1.11.13 (2014-10-02)
--------------------

1.11.12 (2014-10-01)
--------------------
* Bugfix uninitialised occ_cost variable usage
  This fixes `#256 <https://github.com/ros-planning/navigation/issues/256>`_.
* base_local_planner: adds waitForTransform
* Fixed issue causing trajectory planner returning false to isGoalReach ed even when it's control thread finishes executing
* Contributors: Daniel Stonier, Marcus Liebhardt, hes3pal

1.11.11 (2014-07-23)
--------------------
* Minor code cleanup
* Contributors: Enrique Fernández Perdomo

1.11.10 (2014-06-25)
--------------------
* Remove unnecessary colons
* renames acc_lim_th to acc_lim_theta, add warning if using acc_lim_th
* uses odom child_frame_id to set robot_vel frame_id
* Contributors: David Lu!!, Michael Ferguson, Enrique Fernández Perdomo

1.11.9 (2014-06-10)
-------------------
* uses ::hypot(x, y) instead of sqrt(x*x, y*y)
* No need to use `limits->`
* Contributors: Enrique Fernández Perdomo

1.11.8 (2014-05-21)
-------------------

1.11.7 (2014-05-21)
-------------------
* fixes latch_xy_goal_tolerance param not taken
* update build to find eigen using cmake_modules
* Trajectory: fix constness of getter methods
* Use hypot() instead of sqrt(x*x, y*y)
* Fix bug in distance calculation for trajectory rollout
* Some documentation fixes in SimpleTrajectoryGenerator
* Contributors: Michael Ferguson, Siegfried-A. Gevatter Pujals, enriquefernandez

1.11.5 (2014-01-30)
-------------------
* Merge pull request `#152 <https://github.com/ros-planning/navigation/issues/152>`_ from KaijenHsiao/hydro-devel
  uncommented trajectory_planner_ros from catkin_package LIBRARIES so other packages can find it
* Fix negative score bug, add ability to sum scores
* Ignore pyc files from running in devel
* Correct type of prefer_forward penalty member variable
* uncommented trajectory_planner_ros from catkin_package LIBRARIES so other packages can find it
* Better handling of frame param in MapGridVisualizer
* check for CATKIN_ENABLE_TESTING
* Change maintainer from Hersh to Lu

1.11.4 (2013-09-27)
-------------------
* Package URL Updates
* Changed new Odom-Helper::initialize() function to setOdomTopic().
* Converted to a pointcloud pointer in Observation in more places.
