^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dwa_local_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.14.7 (2020-03-10)
-------------------

1.14.6 (2020-03-04)
-------------------

1.14.5 (2019-11-15)
-------------------
* ROS_DEBUG prints incorrect gen_id & incorrect namespace for /latch_xy_goal_tolerance (`#862 <https://github.com/ros-planning/navigation/issues/862>`_)
  * gen_id also increments when the critic's scale is set to 0
  * Moved the latch_xy_goal_tolerance parameter from global namespace to the planner's namespace.
  * `latch_xy_goal_tolerance`  parameter is searched in node_handle's namespace as well as in global namespace, for people who relied on the old configuration
* Set footprint before in place rotation continuation (`#829 <https://github.com/ros-planning/navigation/issues/829>`_)
  * Make sure to call setFootprint() before an in-place rotation
  * Change to const reference
  * Remove footprint from findBestPath
* Contributors: Marcel Soler, Veera Ragav

1.14.4 (2018-06-19)
-------------------

1.14.3 (2018-03-16)
-------------------
* Merge pull request `#672 <https://github.com/ros-planning/navigation/issues/672>`_ from ros-planning/email_update_kinetic
  update maintainer email (kinetic)
* Merge pull request `#648 <https://github.com/ros-planning/navigation/issues/648>`_ from aaronhoy/kinetic_add_ahoy
  Add myself as a maintainer.
* Contributors: Aaron Hoy, Michael Ferguson

1.14.2 (2017-08-14)
-------------------

1.14.1 (2017-08-07)
-------------------
* Fix CMakeLists + package.xmls (`#548 <https://github.com/ros-planning/navigation/issues/548>`_)
* Add cost function to prevent unnecessary spinning
* remove GCC warnings
* Contributors: Martin Günther, Morgan Quigley, Vincent Rabaud

1.14.0 (2016-05-20)
-------------------

1.13.1 (2015-10-29)
-------------------

1.13.0 (2015-03-17)
-------------------
* link only libraries found with find_package
* Contributors: Lukas Bulwahn

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

1.11.13 (2014-10-02)
--------------------

1.11.12 (2014-10-01)
--------------------

1.11.11 (2014-07-23)
--------------------

1.11.10 (2014-06-25)
--------------------

1.11.9 (2014-06-10)
-------------------

1.11.8 (2014-05-21)
-------------------

1.11.7 (2014-05-21)
-------------------
* update build to find eigen using cmake_modules
* Contributors: Michael Ferguson

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
