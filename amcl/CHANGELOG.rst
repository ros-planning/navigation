^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package amcl
^^^^^^^^^^^^^^^^^^^^^^^^^^

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

