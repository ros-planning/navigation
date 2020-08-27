^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clear_costmap_recovery
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.16.7 (2020-08-27)
-------------------

1.16.6 (2020-03-18)
-------------------

1.16.5 (2020-03-15)
-------------------
* [melodic] updated install for better portability. (`#973 <https://github.com/ros-planning/navigation/issues/973>`_)
* Contributors: Sean Yen

1.16.4 (2020-03-04)
-------------------
* [Windows][melodic] Navigation (except for map_server and amcl) Windows build bring up (`#851 <https://github.com/cobalt-robotics/navigation/issues/851>`_)
* Contributors: Sean Yen

1.16.3 (2019-11-15)
-------------------
* Merge pull request `#877 <https://github.com/ros-planning/navigation/issues/877>`_ from SteveMacenski/layer_clear_area-melodic
  [melodic] moving clearing area method to costmap_layer so other applications can clear other types.
* Merge branch 'melodic-devel' into layer_clear_area-melodic
* Add force_updating and affected_maps parameters to tailor clear costmaps recovey (`#838 <https://github.com/ros-planning/navigation/issues/838>`_)
  behavior
* clear area in layer for melodic
* Contributors: Jorge Santos Simón, Michael Ferguson, Steven Macenski, stevemacenski

1.16.2 (2018-07-31)
-------------------

1.16.1 (2018-07-28)
-------------------

1.16.0 (2018-07-25)
-------------------
* Switch to TF2 `#755 <https://github.com/ros-planning/navigation/issues/755>`_
* Contributors: Michael Ferguson, Vincent Rabaud

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
* import only PCL common
* remove GCC warnings
* Contributors: Martin Günther, Mikael Arguedas, Vincent Rabaud

1.14.0 (2016-05-20)
-------------------

1.13.1 (2015-10-29)
-------------------
* proper locking during costmap update
* Contributors: Michael Ferguson

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
* Fix the build (layers library is exported by costmap_2d)
* Contributors: Michael Ferguson

1.11.12 (2014-10-01)
--------------------
* Clarify debug messages
* Initial Clearing Costmap parameter change
* Contributors: David Lu!!, Michael Ferguson

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
* Misc. other commits from Groovy Branch
* Change maintainer from Hersh to Lu

1.11.4 (2013-09-27)
-------------------
* Package URL Updates
* Fix bounds setting 
