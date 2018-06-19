^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package costmap_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.12.16 (2018-06-19)
--------------------
* Merge pull request `#695 <https://github.com/ros-planning/navigation/issues/695>`_ from ros-planning/indigo_675
  Fixed race condition with costmaps in LayeredCostmap::resizeMap() (backport `#675 <https://github.com/ros-planning/navigation/issues/675>`_)
  LayeredCostmap::updateMap() and LayeredCostmap::resizeMap() write to the master grid costmap.
  And these two functions can be called by different threads at the same time.
  One example of these cases is a race condition between subscriber callback thread
  dealing with dynamically-size-changing static_layer and periodical updateMap() calls from Costmap2DROS thread.
  Under the situation the master grid costmap is not thread-safe.
  LayeredCostmap::updateMap() already used the master grid costmap's lock.
* Merge pull request `#692 <https://github.com/ros-planning/navigation/issues/692>`_ from ros-planning/remove_got_footprint
  remove unused got_footprint\_
* Merge pull request `#691 <https://github.com/ros-planning/navigation/issues/691>`_ from ros-planning/rehash_620
  initialize all costmap variables
* Contributors: Jaeyoung Lee, Michael Ferguson

1.12.15 (2018-03-20)
--------------------
* Merge pull request `#671 <https://github.com/ros-planning/navigation/issues/671>`_ from ros-planning/email_update_indigo
  update maintainer email (indigo)
* fix 'enable' for static_layer with rolling window (`#659 <https://github.com/ros-planning/navigation/issues/659>`_) (`#664 <https://github.com/ros-planning/navigation/issues/664>`_)
* Merge pull request `#647 <https://github.com/ros-planning/navigation/issues/647>`_ from aaronhoy/indigo_add_ahoy
  Add myself as a maintainer.
* Contributors: Aaron Hoy, Jannik Abbenseth, Michael Ferguson

1.12.14 (2017-12-19)
--------------------
* don't update costs if inflation radius is zero
* Speedup (~60%) inflation layer update (`#525 <https://github.com/ros-planning/navigation/issues/525>`_)
* Fix CMakeLists + package.xmls (`#548 <https://github.com/ros-planning/navigation/issues/548>`_)
* Contributors: Jorge Santos Simón, Martin Günther, Michael Ferguson

1.12.13 (2016-08-15)
--------------------
* Fixed race condition with costmaps
  Modifying minx\_, miny\_,, maxx\_, and maxy\_ without locking the mutex is
  unsafe and is a race condition if updateMap is called from multiple
  threads.
* Contributors: Alex Henning

1.12.12 (2016-06-24)
--------------------
* Fixed sign error in inflation layer
* Adds warning when a layer shrinks the bounds
* Contributors: Alex Henning

1.12.11 (2016-06-08)
--------------------
* Fixed bug with inflation layer that caused underinflation
  When marking before adding to the priority queue, it was possible to
  underestimate the cost of a cell. This is both dangerous and can lead to
  unintended side-effects with navigation.
* Fixed bug with artifacts when not current
  This is due to not getting clearing observations if the marking
  observations aren't current.
* Fix bug with inflation artifacts being left behind
* Contributors: Alex Henning

1.12.10 (2016-05-27)
--------------------
* Fixes issue with costmaps shearing
* Contributors: Alex Henning

1.12.9 (2016-05-26)
-------------------
* Made costmap publishing truly lazy
* Contributors: Alex Henning

1.12.8 (2016-05-16)
-------------------
* fix resource locations to fix tests
* Fix bug with resetting static layer
* Made update map threadsafe
* Reordered initializer list to match order of declarations.
* Parametrize movementCB timer's period
* No more ghosts in the inflation layer
* Contributors: Alex Henning, Daniel Stonier, Michael Ferguson, Spyros Maniatopoulos

1.12.7 (2016-01-05)
-------------------
* Fix inflation layer locking
* Contributors: Levon Avagyan

1.12.6 (2016-01-02)
-------------------
* Fix deadlock when using multiple static layers in a single program.
* Contributors: Alex Henning

1.12.5 (2015-10-29)
-------------------
* Remove canTransform spam.
* Fix for `#382 <https://github.com/ros-planning/navigation/issues/382>`_
* Republish costmap if origin changes
* Remove extra sign definition and use proper one when padding footprint
* Remove Footprint Layer
* fix plugin warnings on throw, closes `#205 <https://github.com/ros-planning/navigation/issues/205>`_
* initialize publisher variables
* Contributors: Daniel Stonier, David Lu, Michael Ferguson

1.12.4 (2015-06-03)
-------------------
* Look for robot_radius when footprint is not set. `#206 <https://github.com/mikeferguson/navigation/issues/206>`_
* Add a first_map_only parameter so we keep reusing the first received static map
* Contributors: Jihoon Lee, Patrick Chin

1.12.3 (2015-04-30)
-------------------
* support rolling static map in any frame
* fix destructor of Costmap2D
* proper locking during costmap update
* Contributors: Michael Ferguson

1.12.2 (2015-03-31)
-------------------
* Static layer works with rolling window now
* Contributors: Michael Ferguson, Rein Appeldoorn

1.12.1 (2015-03-14)
-------------------
* fixed issue with voxel_layer and obstacle_layer both deleting the same dynamic_reconfigure::Server and causing segfaults
* Fixing various memory freeing operations
* Fix indexing error in OccupancyGridUpdate callback function.
* Contributors: Alex Bencz, David V. Lu!!, James Servos, Julse, Kaijen Hsiao

1.12.0 (2015-02-04)
-------------------
* update maintainer email
* Contributors: Michael Ferguson

1.11.15 (2015-02-03)
--------------------
* Add ARCHIVE_DESTINATION for static builds
* Contributors: Gary Servin

1.11.14 (2014-12-05)
--------------------
* added waitForTransform to bufferCloud to solve extrapolation into the future exception
* deallocate costmap_ before reallocating
* prevent div by zero in raytraceLine
* only prefix sensor_frame when it's not empty
* tf_prefix support in obstacle_layer
* remove undefined function updateUsingPlugins
* remove unused cell_data.h
* numerous style fixes
* Contributors: Andrzej Pronobis, David Lu, Jeremie Deray, Mani Monajjemi, Michael Ferguson, enriquefernandez

1.11.13 (2014-10-02)
--------------------

1.11.12 (2014-10-01)
--------------------
* costmap_2d: export library layers
* Merge pull request `#198 <https://github.com/ros-planning/navigation/issues/198>`_ from kmhallen/hydro-devel
  Fixed costmap_2d clearing from service /move_base/clear_costmaps
* Costmap Layer comments
* Add destructors for all of the layers to remove the dynamic parameter clients
* Add method for removing static observations (for testing)
* Move testing_helper
* Initial Clearing Costmap parameter change
* Fixed costmap_2d clearing from service /move_base/clear_costmaps
* Contributors: David Lu!!, Kevin Hallenbeck, Michael Ferguson

1.11.11 (2014-07-23)
--------------------
* removes trailing spaces and empty lines
* Contributors: Enrique Fernández Perdomo

1.11.10 (2014-06-25)
--------------------
* Remove unnecessary colons
* Remove unused robot_radius parameter from dynamic_reconfigure
* Contributors: Daniel Stonier, David Lu!!

1.11.9 (2014-06-10)
-------------------
* fix hypot issues, add comments to tests from tracking this down
* dynamically reconfigure the previously uninitialised variable 'combination_method', closes `#187 <https://github.com/ros-planning/navigation/issues/187>`_.
* uses ::hypot(x, y) instead of sqrt(x*x, y*y)
* Contributors: Daniel Stonier, Michael Ferguson, Enrique Fernández Perdomo

1.11.8 (2014-05-21)
-------------------

1.11.7 (2014-05-21)
-------------------
* uses %u instead of %d for unsigned int
* update build to find eigen using cmake_modules
* inflation_layer: place .top() & .pop() calls together
* add parameter to configure whether full costmap is published each time
* Contributors: Michael Ferguson, Siegfried-A. Gevatter Pujals, agentx3r, enriquefernandez

1.11.5 (2014-01-30)
-------------------
* Better threading in inflation layer
* don't set initialized until updateMap is called
* check whether costmap is initalized before publishing
* New Overwrite Methods
  updateMap method
  Fix for `#68 <https://github.com/ros-planning/navigation/issues/68>`_
  Fix for inflation memory problems
  InfIsValid `#128 <https://github.com/ros-planning/navigation/issues/128>`_
  Static layer can recieve updates and accept non-lethal values
  Obstacle layer uses track_unknown_space parameter
  Footprint layer is not longer created as top-level layer (used as part of obstacle layer instead)
* Download test data from download.ros.org instead of willow
* Change maintainer from Hersh to Lu

1.11.4 (2013-09-27)
-------------------
* Improve bounds checking 
* Reimplement Clear Costmaps Service by implementing reset functions in each layer
* Package URL Updates
* Additional static layer functionality for receiving updates
* Misc. Pointcloud fixes
* Improved eigen alignment problem on 32-bit arch.
* fixed costmap_2d tests
