^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package global_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.13.1 (2015-10-29)
-------------------
* Add missing angles dependecy
* Fix for `#337 <https://github.com/ros-planning/navigation/issues/337>`_
* Contributors: David V. Lu!!, Gary Servin

1.13.0 (2015-03-17)
-------------------
* Fixing various memory freeing operations
* Add Orientation Filter to Global Planner
* Contributors: Alex Bencz, David V. Lu!!, James Servos, Michael Ferguson

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
* Add Gradient Path's cycle limit to GridPath
* When clearing endpoint, do not overwrite potentials
* Consolidate usage of POT_HIGH
* Contributors: David V. Lu!!

1.11.11 (2014-07-23)
--------------------
* Minor code cleanup
* Update package.xml
* Contributors: David Lu!!, Enrique Fernández Perdomo

1.11.10 (2014-06-25)
--------------------
* Remove unnecessary colons
* global_planner now pushes the goal onto the end of the global path
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

1.11.5 (2014-01-30)
-------------------
* Global Planner Cleanup
* Create the vector reversed instead of reverse it after
* Reversed the plan vector
* global_planner: Fix bgp_plugin.xml file and install it
* Improved Global Planner (from Groovy branch)

1.11.4 (2013-09-27)
-------------------
* Package URL Updates
