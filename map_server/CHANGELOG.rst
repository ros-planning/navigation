^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package map_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
