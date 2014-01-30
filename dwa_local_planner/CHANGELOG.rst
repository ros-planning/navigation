^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dwa_local_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.5 (2014-01-30)
-------------------
* Fix for `#5 <https://github.com/ros-planning/navigation/issues/5>`_
* Parameter to sum scores in DWA Planner
* Keep consistent allignment_cost scale
  Use the configured alignment cost instead of resetting to 1.0. This
  condition (farther from goal than forward_point_distance) is probably
  met at the start of navigation, it seems this cost should be obeyed
  until the robot gets close, then ignored completely.
* Cheat factor for end of DWA Plans
* Change maintainer from Hersh to Lu

1.11.4 (2013-09-27)
-------------------
* Package URL Updates
* Changed new Odom-Helper::initialize() function to setOdomTopic().
* some more corrections for the pointcloud object bug
