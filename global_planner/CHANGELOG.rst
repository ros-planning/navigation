^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package global_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.16.2 (2018-07-31)
-------------------

1.16.1 (2018-07-28)
-------------------

1.16.0 (2018-07-25)
-------------------
* Switch to TF2 `#755 <https://github.com/ros-planning/navigation/issues/755>`_
* feat(orientation_filter): Added additional orientation filter options (`#739 <https://github.com/ros-planning/navigation/issues/739>`_)
  * feat(orientation_filter): Added additional orientation filter options
  Enables plan references with different orientations for omni-base
  controllers. The following options are added:
  - Backward (backward path traversal, pose points to previous point)
  - Leftward (lateral path traversal in the positive y direction)
  - Rightward (lateral path traversal in the negative y direction)
  * Updated orientation filter option description
  * Added window size parameter to orientation filter
  Previously, the orientation was calculated using the current and the
  next point. However, when the path is somewhat jumpy, this results in
  poor orientations. By adding this parameter and altering the orientation
  calculation, the calculated orientation can be smoothened along the path
  by taking into account a larger window. The orientation of index point i
  will be calculated using the positions of i - window_size and i +
  window_size.
* Contributors: Michael Ferguson, Rein Appeldoorn, Vincent Rabaud

1.15.2 (2018-03-22)
-------------------
* Merge pull request `#673 <https://github.com/ros-planning/navigation/issues/673>`_ from ros-planning/email_update_lunar
  update maintainer email (lunar)
* Merge pull request `#649 <https://github.com/ros-planning/navigation/issues/649>`_ from aaronhoy/lunar_add_ahoy
  Add myself as a maintainer.
* Rebase PRs from Indigo/Kinetic (`#637 <https://github.com/ros-planning/navigation/issues/637>`_)
  * Respect planner_frequency intended behavior (`#622 <https://github.com/ros-planning/navigation/issues/622>`_)
  * Only do a getRobotPose when no start pose is given (`#628 <https://github.com/ros-planning/navigation/issues/628>`_)
  Omit the unnecessary call to getRobotPose when the start pose was
  already given, so that move_base can also generate a path in
  situations where getRobotPose would fail.
  This is actually to work around an issue of getRobotPose randomly
  failing.
  * Update gradient_path.cpp (`#576 <https://github.com/ros-planning/navigation/issues/576>`_)
  * Update gradient_path.cpp
  * Update navfn.cpp
  * update to use non deprecated pluginlib macro (`#630 <https://github.com/ros-planning/navigation/issues/630>`_)
  * update to use non deprecated pluginlib macro
  * multiline version as well
  * Print SDL error on IMG_Load failure in server_map (`#631 <https://github.com/ros-planning/navigation/issues/631>`_)
* Contributors: Aaron Hoy, David V. Lu!!, Michael Ferguson

1.15.1 (2017-08-14)
-------------------

1.15.0 (2017-08-07)
-------------------
* convert packages to format2
* Fix CMakeLists + package.xmls (`#548 <https://github.com/ros-planning/navigation/issues/548>`_)
* Fix to increment 'cycle' in while loop (`#546 <https://github.com/ros-planning/navigation/issues/546>`_)
* Fix to check index value before accessing to element of potential array (`#547 <https://github.com/ros-planning/navigation/issues/547>`_)
* Set frame_id and stamp on Path message even if path is not found. (`#533 <https://github.com/ros-planning/navigation/issues/533>`_)
* Contributors: Junya Hayashi, Martin Günther, Mikael Arguedas

1.14.0 (2016-05-20)
-------------------

1.13.1 (2015-10-29)
-------------------
* Add missing angles dependecy
* Fix for `#337 <https://github.com/ros-planning/navigation/issues/337>`_
* Contributors: David V. Lu!!, Gary Servin

1.13.0 (2015-03-17)
-------------------
* Fixing various memory freeing operations
* Add Orientation Filter to Global Planner
* Contributors: Alex Bencz, David V. Lu!!, James Servos, Michael Ferguson

1.12.0 (2015-02-04)
-------------------
* update maintainer email
* Contributors: Michael Ferguson

1.11.15 (2015-02-03)
--------------------

1.11.14 (2014-12-05)
--------------------

1.11.13 (2014-10-02)
--------------------

1.11.12 (2014-10-01)
--------------------
* Add Gradient Path's cycle limit to GridPath
* When clearing endpoint, do not overwrite potentials
* Consolidate usage of POT_HIGH
* Contributors: David V. Lu!!

1.11.11 (2014-07-23)
--------------------
* Minor code cleanup
* Update package.xml
* Contributors: David Lu!!, Enrique Fernández Perdomo

1.11.10 (2014-06-25)
--------------------
* Remove unnecessary colons
* global_planner now pushes the goal onto the end of the global path
* move_base planService now searches out from desired goal
* Contributors: David Lu!!, Kaijen Hsiao

1.11.9 (2014-06-10)
-------------------
* uses ::hypot(x, y) instead of sqrt(x*x, y*y)
* Contributors: Enrique Fernández Perdomo

1.11.8 (2014-05-21)
-------------------

1.11.7 (2014-05-21)
-------------------

1.11.5 (2014-01-30)
-------------------
* Global Planner Cleanup
* Create the vector reversed instead of reverse it after
* Reversed the plan vector
* global_planner: Fix bgp_plugin.xml file and install it
* Improved Global Planner (from Groovy branch)

1.11.4 (2013-09-27)
-------------------
* Package URL Updates
