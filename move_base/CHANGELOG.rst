^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package move_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.16.7 (2020-08-27)
-------------------
* move_base: Add options for make_plan service (`#981 <https://github.com/ros-planning/navigation/issues/981>`_)
  Adds the following two parameters for the ~make_plan service:
  1. make_plan_clear_costmap
  Whether or not to clear the global costmap on make_plan service call.
  2. make_plan_add_unreachable_goal
  Whether or not to add the original goal to the path if it is unreachable in the make_plan service call.
* Contributors: nxdefiant

1.16.6 (2020-03-18)
-------------------

1.16.5 (2020-03-15)
-------------------

1.16.4 (2020-03-04)
-------------------
* [Windows][melodic] Navigation (except for map_server and amcl) Windows build bring up (`#851 <https://github.com/cobalt-robotics/navigation/issues/851>`_)
* Contributors: Sean Yen

1.16.3 (2019-11-15)
-------------------
* Merge branch 'melodic-devel' into layer_clear_area-melodic
* Added publishZeroVelocity() before starting planner (`#751 <https://github.com/ros-planning/navigation/issues/751>`_)
  Edit for Issue `#750 <https://github.com/ros-planning/navigation/issues/750>`_
* Merge pull request `#831 <https://github.com/ros-planning/navigation/issues/831>`_ from ros-planning/feature/remove_slashes
  [melodic] Remove leading slashes from default frame_id parameters
* Remove leading slashes from default frame_id parameters
* Contributors: David V. Lu, Michael Ferguson, SUNIL SULANIA, Steven Macenski

1.16.2 (2018-07-31)
-------------------

1.16.1 (2018-07-28)
-------------------

1.16.0 (2018-07-25)
-------------------
* Switch to TF2 `#755 <https://github.com/ros-planning/navigation/issues/755>`_
* Merge pull request `#723 <https://github.com/ros-planning/navigation/issues/723>`_ from moriarty/melodic-buildfarm-errors
  Melodic buildfarm errors
* Merge pull request `#719 <https://github.com/ros-planning/navigation/issues/719>`_ from ros-planning/lunar_711
  adding mutex locks to costmap clearing service
* Contributors: Alexander Moriarty, Michael Ferguson, Vincent Rabaud, stevemacenski

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
* Add a max_planning_retries parameter to move_base [kinetic] (`#539 <https://github.com/ros-planning/navigation/issues/539>`_)
* Fix for `#517 <https://github.com/ros-planning/navigation/issues/517>`_: create a getRobotPose method on move_base instead of using that on the costmaps
* Fixed deadlock when changing global planner
* rebase fixups
* convert packages to format2
* Fix CMakeLists + package.xmls (`#548 <https://github.com/ros-planning/navigation/issues/548>`_)
* Added deps to amcl costmap_2d move_base (`#512 <https://github.com/ros-planning/navigation/issues/512>`_)
* Fix CMake warnings
* move_base: Add move_base_msgs to find_package.
* Contributors: Jorge Santos, Jorge Santos Simón, Maarten de Vries, Martin Günther, Mikael Arguedas, Vincent Rabaud, mryellow, ne0

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
