^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package amcl
^^^^^^^^^^^^^^^^^^^^^^^^^^

1.14.7 (2020-03-10)
-------------------

1.14.6 (2020-03-04)
-------------------
* Implement selective resampling (`#921 <https://github.com/cobalt-robotics/navigation/issues/921>`_)
* Add CLI option to trigger global localization before processing a bagfile (`#816 <https://github.com/cobalt-robotics/navigation/issues/816>`_)
  Co-authored-by: alain-m <alain@savioke.com>
* Contributors: Adi Vardi, Stephan

1.14.5 (2019-11-15)
-------------------
* fix typo for parameter beam_skip_error_threshold but bandaged for other users in AMCL (`#790 <https://github.com/ros-planning/navigation/issues/790>`_)
  * fix typo but bandage for other users
* Contributors: Steven Macenski

1.14.4 (2018-06-19)
-------------------
* Merge pull request `#731 <https://github.com/ros-planning/navigation/issues/731>`_ from maracuya-robotics/kinetic/amcl-dynamic-reconfigure
  AMCL dynamic reconfigure: Extend parameter range
* In Addition to `#696 <https://github.com/ros-planning/navigation/issues/696>`_, which checks the input arguments, this extends the
  allowed parameter range in dynamic reconfigure. The description for
  "save_pose_rate" says "-1.0 to disable" but allowed only positive values
  previously.
  Also increase maximum value for laser_max_beams for high performance
  systems.
* Merge pull request `#668 <https://github.com/ros-planning/navigation/issues/668>`_ from B0gdar/kinetic-devel
  Update laser_model_type enum on AMCL.cfg
  Adding likelihood_field_prob laser model option on AMCL.cfg to be able to control dynamic parameters with this laser sensor model.
* Contributors: Michael Ferguson, Miguel Cordero, maracuya-robotics

1.14.3 (2018-03-16)
-------------------
* Merge pull request `#678 <https://github.com/ros-planning/navigation/issues/678>`_ from SammysHP/patch-1
  Fix minor typo
* Merge pull request `#666 <https://github.com/ros-planning/navigation/issues/666>`_ from SteveMacenski/kinetic-devel
  removing recomputation of cluster stats causing assertion error (`#634 <https://github.com/ros-planning/navigation/issues/634>`_)
* Merge pull request `#672 <https://github.com/ros-planning/navigation/issues/672>`_ from ros-planning/email_update_kinetic
  update maintainer email (kinetic)
* Merge pull request `#648 <https://github.com/ros-planning/navigation/issues/648>`_ from aaronhoy/kinetic_add_ahoy
  Add myself as a maintainer.
* Contributors: Aaron Hoy, Michael Ferguson, SammysHP, stevemacenski

1.14.2 (2017-08-14)
-------------------

1.14.1 (2017-08-07)
-------------------
* Reference Issue `#592 <https://github.com/ros-planning/navigation/issues/592>`_ Added warning to AMCL when map is published on ... (`#604 <https://github.com/ros-planning/navigation/issues/604>`_)
* recompute cluster stat when force_publication
* Fix CMakeLists + package.xmls (`#548 <https://github.com/ros-planning/navigation/issues/548>`_)
* amcl: fix compilation with gcc v7
* Added deps to amcl costmap_2d move_base (`#512 <https://github.com/ros-planning/navigation/issues/512>`_)
* fix order of parameters (closes `#553 <https://github.com/ros-planning/navigation/issues/553>`_)
* Fix potential string overflow and resource leak
* Contributors: Dmitry Rozhkov, Laurent GEORGE, Martin Günther, Michael Ferguson, Peter Harliman Liem, mryellow, vik748

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

