^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fake_localization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.17.2 (2022-06-20)
-------------------

1.17.1 (2020-08-27)
-------------------
* Fix `#796 <https://github.com/ros-planning/navigation/issues/796>`_ (`#1017 <https://github.com/ros-planning/navigation/issues/1017>`_)
  Use ros::Time(0) instead of timestamp in message so as not to fail to lookupTransform.
* fix isolated build, `#995 <https://github.com/ros-planning/navigation/issues/995>`_ (`#997 <https://github.com/ros-planning/navigation/issues/997>`_)
* Contributors: Michael Ferguson, Ryo KOYAMA

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

1.16.4 (2020-03-04)
-------------------
* remove signals dep (`#945 <https://github.com/cobalt-robotics/navigation/issues/945>`_)
  Boost > 1.7 has signals by default
* Contributors: acxz

1.16.3 (2019-11-15)
-------------------
* Merge pull request `#831 <https://github.com/ros-planning/navigation/issues/831>`_ from ros-planning/feature/remove_slashes
  [melodic] Remove leading slashes from default frame_id parameters
* Remove leading slashes from default frame_id parameters
* Fix for `#805 <https://github.com/ros-planning/navigation/issues/805>`_ (`#813 <https://github.com/ros-planning/navigation/issues/813>`_)
* Contributors: David V. Lu, David V. Lu!!, Michael Ferguson

1.16.2 (2018-07-31)
-------------------

1.16.1 (2018-07-28)
-------------------

1.16.0 (2018-07-25)
-------------------
* Merge pull request `#690 <https://github.com/ros-planning/navigation/issues/690>`_ from ros-planning/lunar_609
  switch fake_localization to tf2.
* Contributors: Michael Ferguson, Vincent Rabaud

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
* Fix CMakeLists + package.xmls (`#548 <https://github.com/ros-planning/navigation/issues/548>`_)
* Contributors: Martin Günther, Mikael Arguedas, Vincent Rabaud

1.14.0 (2016-05-20)
-------------------

1.13.1 (2015-10-29)
-------------------
* More tolerant initial pose transform lookup.
* Contributors: Daniel Stonier

1.13.0 (2015-03-17)
-------------------

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

1.11.4 (2013-09-27)
-------------------
* Package URL Updates
* amcl_pose and particle cloud are now published latched
