^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package navfn
^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.16.7 (2020-08-27)
-------------------

1.16.6 (2020-03-18)
-------------------
* Fix Unknown CMake command check_include_file (navfn & base_local_planner) (`#975 <https://github.com/ros-planning/navigation/issues/975>`_)
* Contributors: Sam Pfeiffer

1.16.5 (2020-03-15)
-------------------
* [melodic] updated install for better portability. (`#973 <https://github.com/ros-planning/navigation/issues/973>`_)
* Contributors: Sean Yen

1.16.4 (2020-03-04)
-------------------
* [Windows][melodic] Navigation (except for map_server and amcl) Windows build bring up (`#851 <https://github.com/cobalt-robotics/navigation/issues/851>`_)
* Add frame ID to empty NavFn paths (`#964 <https://github.com/cobalt-robotics/navigation/issues/964>`_)
  * Don't publish empty paths
  RViz will complain about not being able to transform the path if it doesn't have a frame ID, so we'll just drop these
  Closes `#963 <https://github.com/cobalt-robotics/navigation/issues/963>`_
  * Publish empty paths with a valid frame
  * Fix indexing into empty plan for timestamp
* Contributors: Nick Walker, Sean Yen

1.16.3 (2019-11-15)
-------------------
* Merge pull request `#831 <https://github.com/ros-planning/navigation/issues/831>`_ from ros-planning/feature/remove_slashes
  [melodic] Remove leading slashes from default frame_id parameters
* Remove leading slashes from default frame_id parameters
* Merge pull request `#789 <https://github.com/ros-planning/navigation/issues/789>`_ from ipa-fez/fix/astar_const_melodic
  Remove const from create_nav_plan_astar
* remove const from create_nav_plan_astar
* Contributors: David V. Lu, Felix, Michael Ferguson

1.16.2 (2018-07-31)
-------------------

1.16.1 (2018-07-28)
-------------------

1.16.0 (2018-07-25)
-------------------
* Remove dependency on PCL
* Switch to TF2 `#755 <https://github.com/ros-planning/navigation/issues/755>`_
* Merge pull request `#737 <https://github.com/ros-planning/navigation/issues/737>`_ from marting87/minor_comment_fixes
  Minor comment corrections
* Contributors: David V. Lu, Martin Ganeff, Michael Ferguson, Vincent Rabaud

1.15.2 (2018-03-22)
-------------------
* Merge pull request `#673 <https://github.com/ros-planning/navigation/issues/673>`_ from ros-planning/email_update_lunar
  update maintainer email (lunar)
* Remove Dead Code [Lunar] (`#646 <https://github.com/ros-planning/navigation/issues/646>`_)
  * Clean up navfn
  * Cleanup amcl
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
* import only PCL common
* port `#549 <https://github.com/ros-planning/navigation/issues/549>`_ (in alphabetical order)
* address gcc6 build error
* remove GCC warnings
* Contributors: Lukas Bulwahn, Martin Günther, Michael Ferguson, Mikael Arguedas, Vincent Rabaud

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
