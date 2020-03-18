^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package amcl
^^^^^^^^^^^^^^^^^^^^^^^^^^

1.16.6 (2020-03-18)
-------------------

1.16.5 (2020-03-15)
-------------------
* [melodic] updated install for better portability. (`#973 <https://github.com/ros-planning/navigation/issues/973>`_)
* Contributors: Sean Yen

1.16.4 (2020-03-04)
-------------------
* Implement selective resampling (`#921 <https://github.com/cobalt-robotics/navigation/issues/921>`_) (`#971 <https://github.com/cobalt-robotics/navigation/issues/971>`_)
  Co-authored-by: Adi Vardi <adidasv111@gmail.com>
* Add CLI option to trigger global localization before processing a bagfile (`#816 <https://github.com/cobalt-robotics/navigation/issues/816>`_) (`#970 <https://github.com/cobalt-robotics/navigation/issues/970>`_)
  Co-authored-by: alain-m <alain@savioke.com>
* Fix some reconfigure parameters not being applied [amcl]. (`#952 <https://github.com/cobalt-robotics/navigation/issues/952>`_)
* amcl: include missing CMake functions to fix build (`#946 <https://github.com/cobalt-robotics/navigation/issues/946>`_)
* Set proper limits for the z-weights [amcl]. (`#953 <https://github.com/cobalt-robotics/navigation/issues/953>`_)
* Merge pull request `#965 <https://github.com/cobalt-robotics/navigation/issues/965>`_ from nlimpert/nlimpert/fix_missing_cmake_include
  Add missing CMake include(CheckSymbolExists) for CMake >= 3.15
* amcl: add missing CMake include(CheckSymbolExists)
  Starting with CMake 3.15 an explicit include(CheckSymbolExists)
  is required to use the check_symbol_exists macro.
* Contributors: Ben Wolsieffer, Michael Ferguson, Nicolas Limpert, Patrick Chin

1.16.3 (2019-11-15)
-------------------
* Merge branch 'melodic-devel' into layer_clear_area-melodic
* Fix typo in amcl_laser model header (`#918 <https://github.com/ros-planning/navigation/issues/918>`_)
* Merge pull request `#849 <https://github.com/ros-planning/navigation/issues/849>`_ from seanyen/amcl_windows_fix
  [Windows][melodic] AMCL Windows build bring up.
* revert unrelated changes.
* AMCL windows build bring up.
  * Add HAVE_UNISTD and HAVE_DRAND48 and portable_utils.hpp for better cross compiling.
  * Variable length array is not supported in MSVC, conditionally disable it.
  * Fix install location for shared lib and executables on Windows.
  * Use isfinite for better cross compiling.
* feat: AMCL Diagnostics (`#807 <https://github.com/ros-planning/navigation/issues/807>`_)
  Diagnostic task that monitors the estimated standard deviation of the filter.
  By: reinzor <reinzor@gmail.com>
* fix typo for parameter beam_skip_error_threshold but bandaged for other users in AMCL (`#790 <https://github.com/ros-planning/navigation/issues/790>`_)
  * fix typo but bandage for other users
* Merge pull request `#785 <https://github.com/ros-planning/navigation/issues/785>`_ from mintar/amcl_c++11
  amcl: Add compile option C++11
* amcl: Set C++ standard 11 if not set
  This is required to build the melodic-devel branch of the navigation
  stack on kinetic. Melodic sets CMAKE_CXX_STANDARD=14, but kinetic
  doesn't set that variable at all.
* Contributors: Hadi Tabatabaee, Martin Günther, Michael Ferguson, Rein Appeldoorn, Sean Yen, Steven Macenski

1.16.2 (2018-07-31)
-------------------
* Merge pull request `#773 <https://github.com/ros-planning/navigation/issues/773>`_ from ros-planning/packaging_fixes
  packaging fixes
* update amcl to have proper depends
  * add geometry_msgs
  * add tf2_msgs
  * fix alphabetical order
* Contributors: Michael Ferguson

1.16.1 (2018-07-28)
-------------------
* Merge pull request `#770 <https://github.com/ros-planning/navigation/issues/770>`_ from ros-planning/fix_debians
  Fix debian builds (closes `#769 <https://github.com/ros-planning/navigation/issues/769>`_)
* make AMCL depend on sensor_msgs
  previously, amcl depended on TF, which depended on
  sensor_msgs.
* Contributors: Michael Ferguson

1.16.0 (2018-07-25)
-------------------
* Switch to TF2 `#755 <https://github.com/ros-planning/navigation/issues/755>`_
* Merge pull request `#734 <https://github.com/ros-planning/navigation/issues/734>`_ from ros-planning/melodic_731
  AMCL dynamic reconfigure: Extend parameter range (Forward port `#731 <https://github.com/ros-planning/navigation/issues/731>`_)
* Merge pull request `#728 <https://github.com/ros-planning/navigation/issues/728>`_ from ros-planning/melodic_tf2_conversion
  switch AMCL to use TF2
* fix swapped odom1/4 in omni model, fixes `#499 <https://github.com/ros-planning/navigation/issues/499>`_
* Merge pull request `#730 <https://github.com/ros-planning/navigation/issues/730>`_ from Glowcloud/melodic-devel
  Fix for Potential Memory Leak  in AmclNode::reconfigureCB `#729 <https://github.com/ros-planning/navigation/issues/729>`_
* Fix for Potential Memory Leak  in AmclNode::reconfigureCB
* switch AMCL to use TF2
* Merge pull request `#727 <https://github.com/ros-planning/navigation/issues/727>`_ from ros-planning/melodic_668
  Update laser_model_type enum on AMCL.cfg (Melodic port of `#668 <https://github.com/ros-planning/navigation/issues/668>`_)
* Update laser_model_type enum on AMCL.cfg
  Adding likelihood_field_prob laser model option on AMCL.cfg to be able to control dynamic parameters with this laser sensor model.
* Merge pull request `#723 <https://github.com/ros-planning/navigation/issues/723>`_ from moriarty/melodic-buildfarm-errors
  Melodic buildfarm errors
* include <memory> for std::shared_ptr
* Merge pull request `#718 <https://github.com/ros-planning/navigation/issues/718>`_ from moriarty/tf2-buffer-ptr
  [melodic] tf2_buffer\_ -> tf2_buffer_ptr\_
* [melodic] tf2_buffer\_ -> tf2_buffer_ptr\_
  Change required due to changes in upstream dependencies:
  `ros/geometry#163 <https://github.com/ros/geometry/issues/163>`_: "Maintain & expose tf2 Buffer in shared_ptr for tf"
  fixes `ros-planning/navigation#717 <https://github.com/ros-planning/navigation/issues/717>`_ (for compile errors at least.)
* Contributors: Alexander Moriarty, Glowcloud, Martin Ganeff, Michael Ferguson, Miguel Cordero, Vincent Rabaud, maracuya-robotics

1.15.2 (2018-03-22)
-------------------
* Fix minor typo (`#682 <https://github.com/ros-planning/navigation/issues/682>`_)
  This typo caused some confusion because we were searching for a semicolon in our configuration.
* Merge pull request `#677 <https://github.com/ros-planning/navigation/issues/677>`_ from ros-planning/lunar_634
  removing recomputation of cluster stats causing assertion error (`#634 <https://github.com/ros-planning/navigation/issues/634>`_)
* Merge pull request `#673 <https://github.com/ros-planning/navigation/issues/673>`_ from ros-planning/email_update_lunar
  update maintainer email (lunar)
* Remove Dead Code [Lunar] (`#646 <https://github.com/ros-planning/navigation/issues/646>`_)
  * Clean up navfn
  * Cleanup amcl
* Merge pull request `#649 <https://github.com/ros-planning/navigation/issues/649>`_ from aaronhoy/lunar_add_ahoy
  Add myself as a maintainer.
* Contributors: Aaron Hoy, David V. Lu!!, Michael Ferguson, stevemacenski

1.15.1 (2017-08-14)
-------------------

1.15.0 (2017-08-07)
-------------------
* Reference Issue `#592 <https://github.com/ros-planning/navigation/issues/592>`_ Added warning to AMCL when map is published on ... (`#604 <https://github.com/ros-planning/navigation/issues/604>`_)
* rebase fixups
* convert packages to format2
* recompute cluster stat when force_publication
* Fix CMakeLists + package.xmls (`#548 <https://github.com/ros-planning/navigation/issues/548>`_)
* amcl: fix compilation with gcc v7
* Added deps to amcl costmap_2d move_base (`#512 <https://github.com/ros-planning/navigation/issues/512>`_)
* fix order of parameters (closes `#553 <https://github.com/ros-planning/navigation/issues/553>`_)
* Fix potential string overflow and resource leak
* Contributors: Dmitry Rozhkov, Laurent GEORGE, Martin Günther, Michael Ferguson, Mikael Arguedas, Peter Harliman Liem, mryellow, vik748

1.14.0 (2016-05-20)
-------------------
* Allow AMCL to run from bag file to allow very fast testing.
* Fixes interpretation of a delayed initialpose message (see `#424 <https://github.com/ros-planning/navigation/issues/424>`_).
  The tf lookup as it was before this change was very likely to fail as
  ros::Time::now() was used to look up a tf without waiting on the tf's
  availability. Additionally, the computation of the "new pose" by
  multiplying the delta that the robot moved from the initialpose's
  timestamp to ros::Time::now() was wrong. That delta has to by multiplied
  from the right to the "old pose".
  This commit also changes the reference frame to look up this delta to be
  the odom frame as this one is supposed to be smooth and therefore the
  best reference to get relative robot motion in the robot (base link) frame.
* New unit test for proper interpretation of a delayed initialpose message.
  Modifies the set_pose.py script to be able to send an initial pose with
  a user defined time stamp at a user defined time. Adds a rostest to
  exercise this new option.
  This reveals the issues mentioned in `#424 <https://github.com/ros-planning/navigation/issues/424>`_ (the new test fails).
* Contributors: Derek King, Stephan Wirth

1.13.1 (2015-10-29)
-------------------
* adds the set_map service to amcl
* fix pthread_mutex_lock on shutdown
* Contributors: Michael Ferguson, Stephan Wirth

1.13.0 (2015-03-17)
-------------------
* amcl_node will now save latest pose on shutdown
* Contributors: Ian Danforth

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
* Bug fix to remove particle weights being reset when motion model is updated
* Integrated new sensor model which calculates the observation likelihood in a probabilistic manner
  Also includes the option to do beam-skipping (to better handle observations from dynamic obstacles)
* Pose pulled from parameter server when new map received
* Contributors: Steven Kordell, hes3pal

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
* removes useless this->z_max = z_max assignment
* Fix warning string.
* Contributors: Jeremiah Via, enriquefernandez

1.11.5 (2014-01-30)
-------------------
* Fix for `#160 <https://github.com/ros-planning/navigation/issues/160>`_
* Download test data from download.ros.org instead of willow
* Change maintainer from Hersh to Lu

1.11.4 (2013-09-27)
-------------------
* Package URL Updates
* amcl_pose and particle cloud are now published latched
* Fixed or commented out failing amcl tests.

