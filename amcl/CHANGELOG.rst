^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package amcl
^^^^^^^^^^^^^^^^^^^^^^^^^^

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

