^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package map_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.14.7 (2020-03-10)
-------------------

1.14.6 (2020-03-04)
-------------------

1.14.5 (2019-11-15)
-------------------

1.14.4 (2018-06-19)
-------------------
* Allow specification of free/occupied thresholds for map_saver (`#708 <https://github.com/ros-planning/navigation/issues/708>`_)
  * add occupied threshold command line parameter to map_saver (--occ)
  * add free threshold command line parameter to map_saver (--free)
* Map server wait for a valid time, fix `#573 <https://github.com/ros-planning/navigation/issues/573>`_ (`#700 <https://github.com/ros-planning/navigation/issues/700>`_)
  When launching the map_server with Gazebo, the current time is picked
  before the simulation is started and then has a value of 0.
  Later when querying the stamp of the map, a value of has a special
  signification on tf transform for example.
* Contributors: Romain Reignier, ipa-fez

1.14.3 (2018-03-16)
-------------------
* Merge pull request `#672 <https://github.com/ros-planning/navigation/issues/672>`_ from ros-planning/email_update_kinetic
  update maintainer email (kinetic)
* Merge pull request `#648 <https://github.com/ros-planning/navigation/issues/648>`_ from aaronhoy/kinetic_add_ahoy
  Add myself as a maintainer.
* Print SDL error on IMG_Load failure in server_map (`#631 <https://github.com/ros-planning/navigation/issues/631>`_)
* Contributors: Aaron Hoy, Aurélien Labate, Michael Ferguson

1.14.2 (2017-08-14)
-------------------
* remove offending library export (fixes `#612 <https://github.com/ros-planning/navigation/issues/612>`_)
* Contributors: Michael Ferguson

1.14.1 (2017-08-07)
-------------------
* refactor to not use tf version 1 (`#561 <https://github.com/ros-planning/navigation/issues/561>`_)
* Fix CMakeLists + package.xmls (`#548 <https://github.com/ros-planning/navigation/issues/548>`_)
* Merge pull request `#560 <https://github.com/ros-planning/navigation/issues/560>`_ from wjwwood/map_server_fixup_cmake
* update to support Python 2 and 3 (`#559 <https://github.com/ros-planning/navigation/issues/559>`_)
* remove duplicate and unreferenced file (`#558 <https://github.com/ros-planning/navigation/issues/558>`_)
* remove trailing whitespace from map_server package (`#557 <https://github.com/ros-planning/navigation/issues/557>`_)
* fix cmake use of yaml-cpp and sdl / sdl-image
* Contributors: Martin Günther, Michael Ferguson, Vincent Rabaud, William Woodall

1.14.0 (2016-05-20)
-------------------
* Corrections to alpha channel detection and usage.
  Changing to actually detect whether the image has an alpha channel instead of
  inferring based on the number of channels.
  Also reverting to legacy behavior of trinary mode overriding alpha removal.
  This will cause the alpha channel to be averaged in with the others in trinary
  mode, which is the current behavior before this PR.
* Removing some trailing whitespace.
* Use enum to control map interpretation
* Contributors: Aaron Hoy, David Lu

1.13.1 (2015-10-29)
-------------------

1.13.0 (2015-03-17)
-------------------
* rename image_loader library, fixes `#208 <https://github.com/ros-planning/navigation/issues/208>`_
* Contributors: Michael Ferguson

1.12.0 (2015-02-04)
-------------------
* update maintainer email
* Contributors: Michael Ferguson

1.11.15 (2015-02-03)
--------------------

1.11.14 (2014-12-05)
--------------------
* prevent inf loop
* Contributors: Jeremie Deray

1.11.13 (2014-10-02)
--------------------

1.11.12 (2014-10-01)
--------------------
* map_server: [style] alphabetize dependencies
* map_server: remove vestigial export line
  the removed line does not do anything in catkin
* Contributors: William Woodall

1.11.11 (2014-07-23)
--------------------

1.11.10 (2014-06-25)
--------------------

1.11.9 (2014-06-10)
-------------------

1.11.8 (2014-05-21)
-------------------
* fix build, was broken by `#175 <https://github.com/ros-planning/navigation/issues/175>`_
* Contributors: Michael Ferguson

1.11.7 (2014-05-21)
-------------------
* make rostest in CMakeLists optional
* Contributors: Lukas Bulwahn

1.11.5 (2014-01-30)
-------------------
* install crop map
* removing .py from executable script
* Map Server can serve maps with non-lethal values
* Added support for YAML-CPP 0.5+.
  The new yaml-cpp API removes the "node >> outputvar;" operator, and
  it has a new way of loading documents. There's no version hint in the
  library's headers, so I'm getting the version number from pkg-config.
* check for CATKIN_ENABLE_TESTING
* Change maintainer from Hersh to Lu

1.11.4 (2013-09-27)
-------------------
* prefix utest target to not collide with other targets
* Package URL Updates
* unique target names to avoid conflicts (e.g. with map-store)
