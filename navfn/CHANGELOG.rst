^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package navfn
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.14.3 (2018-03-16)
-------------------
* Merge pull request `#672 <https://github.com/ros-planning/navigation/issues/672>`_ from ros-planning/email_update_kinetic
  update maintainer email (kinetic)
* Merge pull request `#648 <https://github.com/ros-planning/navigation/issues/648>`_ from aaronhoy/kinetic_add_ahoy
  Add myself as a maintainer.
* added message_generation to build deps to prevent failing generation of GetStatus, MakeNavPlan and SetCostmap (`#640 <https://github.com/ros-planning/navigation/issues/640>`_)
* Rebase PRs from Indigo (`#636 <https://github.com/ros-planning/navigation/issues/636>`_)
  * Update gradient_path.cpp (`#576 <https://github.com/ros-planning/navigation/issues/576>`_)
  * Update gradient_path.cpp
  * Update navfn.cpp
  * Only do a getRobotPose when no start pose is given (`#628 <https://github.com/ros-planning/navigation/issues/628>`_)
  Omit the unnecessary call to getRobotPose when the start pose was
  already given, so that move_base can also generate a path in
  situations where getRobotPose would fail.
  This is actually to work around an issue of getRobotPose randomly
  failing.
* update to use non deprecated pluginlib macro (`#630 <https://github.com/ros-planning/navigation/issues/630>`_)
  * update to use non deprecated pluginlib macro
  * multiline version as well
* Contributors: Aaron Hoy, David V. Lu!!, Leroy Rügemer, Michael Ferguson, Mikael Arguedas

1.14.2 (2017-08-14)
-------------------

1.14.1 (2017-08-07)
-------------------
* Fix CMakeLists + package.xmls (`#548 <https://github.com/ros-planning/navigation/issues/548>`_)
* port `#549 <https://github.com/ros-planning/navigation/issues/549>`_ (in alphabetical order)
* address gcc6 build error
* remove GCC warnings
* Contributors: Lukas Bulwahn, Martin Günther, Michael Ferguson, Vincent Rabaud

1.14.0 (2016-05-20)
-------------------
* navfn: make independent on costmap implementation
  navfn::NavfnROS:
  * remove direct dependency on costmap_2d::Costmap2DROS
  * add constructor for barebone costmap_2d::Costmap2D (user must provide also global_frame)
  * NavfnROS::initialize() follows constructor semantics
  nav_core::BaseGlobalPlanner interface unchanged
* Contributors: Jiri Horner

1.13.1 (2015-10-29)
-------------------
* Fix for `#337 <https://github.com/ros-planning/navigation/issues/337>`_
* Contributors: David V. Lu!!

1.13.0 (2015-03-17)
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

1.11.13 (2014-10-02)
--------------------

1.11.12 (2014-10-01)
--------------------

1.11.11 (2014-07-23)
--------------------
* removes unused param planner_costmap_publish_frequency
* Contributors: Enrique Fernández Perdomo

1.11.10 (2014-06-25)
--------------------
* Remove unnecessary colons
* Contributors: David Lu!!

1.11.9 (2014-06-10)
-------------------
* uses ::hypot(x, y) instead of sqrt(x*x, y*y)
* Contributors: Enrique Fernández Perdomo

1.11.8 (2014-05-21)
-------------------

1.11.7 (2014-05-21)
-------------------
* update build to find eigen using cmake_modules
* Contributors: Michael Ferguson

1.11.5 (2014-01-30)
-------------------
* navfn: fix parallel build error from missing dep
* fixed header installation directory
* check for CATKIN_ENABLE_TESTING
* Change maintainer from Hersh to Lu

1.11.4 (2013-09-27)
-------------------
* Package URL Updates
* fixed `#103 <https://github.com/ros-planning/navigation/issues/103>`_, navfn_node not installed
* Potential missing dependency
