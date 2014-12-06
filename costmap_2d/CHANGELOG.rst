^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package costmap_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
