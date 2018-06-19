^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package move_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.14.4 (2018-06-19)
-------------------
* Merge pull request `#711 <https://github.com/ros-planning/navigation/issues/711>`_ from SteveMacenski/costmap_clearing_thread_lock
  adding mutex locks to costmap clearing service
* Contributors: Michael Ferguson, stevemacenski

1.14.3 (2018-03-16)
-------------------
* Merge pull request `#672 <https://github.com/ros-planning/navigation/issues/672>`_ from ros-planning/email_update_kinetic
  update maintainer email (kinetic)
* Merge pull request `#648 <https://github.com/ros-planning/navigation/issues/648>`_ from aaronhoy/kinetic_add_ahoy
  Add myself as a maintainer. 
* Rebase PRs from Indigo (`#636 
<https://github.com/ros-planning/navigation/issues/636>`_)
  * Update gradient_path.cpp (`#576 <https://github.com/ros-planning/navigation/issues/576>`_)
  * Update gradient_path.cpp
  * Update navfn.cpp
  * Only do a getRobotPose when no start pose is given (`#628 <https://github.com/ros-planning/navigation/issues/628>`_)
  Omit the unnecessary call to getRobotPose when the start pose was
  already given, so that move_base can also generate a path in
  situations where getRobotPose would fail.
  This is actually to work around an issue of getRobotPose randomly
  failing.
* Respect planner_frequency intended behavior (`#622 <https://github.com/ros-planning/navigation/issues/622>`_)
* Contributors: Aaron Hoy, David V. Lu!!, Jorge Santos Simón, Michael Ferguson

1.14.2 (2017-08-14)
-------------------

1.14.1 (2017-08-07)
-------------------
* Add a max_planning_retries parameter to move_base [kinetic] (`#539 <https://github.com/ros-planning/navigation/issues/539>`_)
* Fixed deadlock when changing global planner
* Fix CMakeLists + package.xmls (`#548 <https://github.com/ros-planning/navigation/issues/548>`_)
* Added deps to amcl costmap_2d move_base (`#512 <https://github.com/ros-planning/navigation/issues/512>`_)
* move_base: Add move_base_msgs to find_package.
* Contributors: Jorge Santos Simón, Maarten de Vries, Martin Günther, Vincent Rabaud, mryellow, ne0

1.14.0 (2016-05-20)
-------------------

1.13.1 (2015-10-29)
-------------------
* Removes installation of nonexistent directories
* use correct size for clearing window
* full name has been required for eons, this code just adds unneeded complexity
* remove ancient conversion scripts from v0.2 to v0.3
* proper locking during costmap update
* Contributors: Michael Ferguson, Thiago de Freitas Oliveira Araujo

1.13.0 (2015-03-17)
-------------------
* Fixing various memory freeing operations
* Contributors: Alex Bencz

1.12.0 (2015-02-04)
-------------------
* update maintainer email
* Contributors: Michael Ferguson

1.11.15 (2015-02-03)
--------------------
* Disable global planner when resetting state.
* Mark move_base headers for installation
* Add ARCHIVE DESTINATION for move_base
* Break infinite loop when tolerance 0 is used
* remove partial usage of <tab> in the code
* Contributors: Gary Servin, Michael Ferguson, ohendriks, v4hn

1.11.14 (2014-12-05)
--------------------
* use timer rather than rate for immediate wakeup
* adding lock to planner makePlan fail case
* Contributors: Michael Ferguson, phil0stine

1.11.13 (2014-10-02)
--------------------

1.11.12 (2014-10-01)
--------------------

1.11.11 (2014-07-23)
--------------------
* removes trailing spaces and empty lines
* Contributors: Enrique Fernández Perdomo

1.11.10 (2014-06-25)
--------------------
* Remove unnecessary colons
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
* update build to find eigen using cmake_modules
* Fix classloader warnings on exit of move_base
* Contributors: Michael Ferguson

1.11.4 (2013-09-27)
-------------------
* Package URL Updates
* Reintroduce ClearCostmaps Service
* Add dependencies to recovery behaviors. 
