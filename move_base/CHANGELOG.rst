^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package move_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.12.13 (2016-08-15)
--------------------
* Merge pull request `#495 <https://github.com/ros-planning/navigation/issues/495>`_ from corot/patch-3
  Fix `#494 <https://github.com/ros-planning/navigation/issues/494>`_: prevent zero-velocity commands during recovery behaviors
* move_base: Add move_base_msgs to find_package.
* Issue `#496 <https://github.com/ros-planning/navigation/issues/496>`_: add a max_planning_retries parameter as an alternative to planner_patience to limit the failed calls to global planner
* Fix `#494 <https://github.com/ros-planning/navigation/issues/494>`_: prevent zero-velocity commands during recovery behaviors
  Partially reverts 0a686c90c6f188a2731f6f88c2c08610f0ec907e
* Contributors: Jorge Santos, Jorge Santos Simón, Maarten de Vries, Michael Ferguson

1.12.12 (2016-06-24)
--------------------

1.12.11 (2016-06-08)
--------------------
* [Fix] Illegal vector visit when no path planned
* Contributors: gjc13

1.12.10 (2016-05-27)
--------------------

1.12.9 (2016-05-26)
-------------------

1.12.8 (2016-05-16)
-------------------

1.12.7 (2016-01-05)
-------------------

1.12.6 (2016-01-02)
-------------------

1.12.5 (2015-10-29)
-------------------

1.12.4 (2015-06-03)
-------------------

1.12.3 (2015-04-30)
-------------------
* proper locking during costmap update
* Contributors: Michael Ferguson

1.12.2 (2015-03-31)
-------------------

1.12.1 (2015-03-14)
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
