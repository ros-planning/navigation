^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dwa_local_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.17.2 (2022-06-20)
-------------------

1.17.1 (2020-08-27)
-------------------

1.17.0 (2020-04-02)
-------------------
* Merge pull request `#982 <https://github.com/ros-planning/navigation/issues/982>`_ from ros-planning/noetic_prep
  Noetic Migration
* increase required cmake version
* Contributors: Michael Ferguson

1.16.6 (2020-03-18)
-------------------

1.16.5 (2020-03-15)
-------------------
* [melodic] updated install for better portability. (`#973 <https://github.com/ros-planning/navigation/issues/973>`_)
* Contributors: Sean Yen

1.16.4 (2020-03-04)
-------------------
* Fixes gdist- and pdist_scale node paramter names (`#936 <https://github.com/cobalt-robotics/navigation/issues/936>`_)
  Renames goal and path distance dynamic reconfigure parameter
  names in the cfg file in order to actually make the parameters
  used by the trajectory planner changeable.
  Fixes `#935 <https://github.com/cobalt-robotics/navigation/issues/935>`_
* Contributors: David Leins

1.16.3 (2019-11-15)
-------------------
* Set footprint before in place rotation continuation (`#829 <https://github.com/ros-planning/navigation/issues/829>`_) (`#861 <https://github.com/ros-planning/navigation/issues/861>`_)
  * Make sure to call setFootprint() before an in-place rotation
  * Change to const reference
  * Remove footprint from findBestPath
* Contributors: David V. Lu!!

1.16.2 (2018-07-31)
-------------------
* Merge pull request `#773 <https://github.com/ros-planning/navigation/issues/773>`_ from ros-planning/packaging_fixes
  packaging fixes
* fix depends for dwa_local_planner
  * add tf2_geometry_msgs (due to https://github.com/ros/geometry2/issues/275)
  * add missing depends on angles, sensor_msgs, tf2
* Contributors: Michael Ferguson

1.16.1 (2018-07-28)
-------------------

1.16.0 (2018-07-25)
-------------------
* Merge pull request `#765 <https://github.com/ros-planning/navigation/issues/765>`_ from ros-planning/remove_pcl
  remove left over PCL depends in dwa_local_planner
* Remove PCL from local planners
* Switch to TF2 `#755 <https://github.com/ros-planning/navigation/issues/755>`_
* Make trajectory scoring scales consistent.
* unify parameter names between base_local_planner and dwa_local_planner
  addresses parts of `#90 <https://github.com/ros-planning/navigation/issues/90>`_
* Contributors: David V. Lu, Michael Ferguson, Pavlo Kolomiiets, Vincent Rabaud, moriarty

1.15.2 (2018-03-22)
-------------------
* Merge pull request `#673 <https://github.com/ros-planning/navigation/issues/673>`_ from ros-planning/email_update_lunar
  update maintainer email (lunar)
* Merge pull request `#649 <https://github.com/ros-planning/navigation/issues/649>`_ from aaronhoy/lunar_add_ahoy
  Add myself as a maintainer.
* Contributors: Aaron Hoy, Michael Ferguson

1.15.1 (2017-08-14)
-------------------

1.15.0 (2017-08-07)
-------------------
* convert packages to format2
* Add cost function to prevent unnecessary spinning
* Fix CMakeLists + package.xmls (`#548 <https://github.com/ros-planning/navigation/issues/548>`_)
* import only PCL common
* remove GCC warnings
* Fix CMake warnings
* Contributors: Martin Günther, Mikael Arguedas, Morgan Quigley, Vincent Rabaud

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
