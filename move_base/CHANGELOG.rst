^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package move_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.14.0 (2016-05-20)
-------------------

1.13.1 (2015-10-29)
-------------------
* Removes installation of nonexistent directories
* use correct size for clearing window
* full name has been required for eons, this code just adds unneeded complexity
* remove ancient conversion scripts from v0.2 to v0.3
* proper locking during costmap update
* Contributors: Michael Ferguson, Thiago de Freitas Oliveira Araujo

1.13.0 (2015-03-17)
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
