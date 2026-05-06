^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_benchmarks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.11 (2022-12-21)
-------------------

1.1.10 (2022-09-13)
-------------------
* Fix namespace of planning plugin for benchmarks examples (`#3128 <https://github.com/ros-planning/moveit/issues/3128>`_)
* Replace bind() with lambdas (`#3106 <https://github.com/ros-planning/moveit/issues/3106>`_)
* Filter more invalid values in moveit_benchmark_statistics.py (`#3084 <https://github.com/ros-planning/moveit/issues/3084>`_)
* Contributors: Hugal31, Michael Görner, Robert Haschke

1.1.9 (2022-03-06)
------------------

1.1.8 (2022-01-30)
------------------

1.1.7 (2021-12-31)
------------------
* Provide ``MOVEIT_VERSION_CHECK`` macro (`#2997 <https://github.com/ros-planning/moveit/issues/2997>`_)
* Switch to ``std::bind`` (`#2967 <https://github.com/ros-planning/moveit/issues/2967>`_)
* Contributors: Jochen Sprickerhof, Robert Haschke

1.1.6 (2021-11-06)
------------------
* Use newly introduced cmake macro ``moveit_build_options()`` from ``moveit_core``
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Fix predefined poses benchmark example (`#2718 <https://github.com/ros-planning/moveit/issues/2718>`_)
* Contributors: Captain Yoshi, Robert Haschke

1.1.5 (2021-05-23)
------------------

1.1.4 (2021-05-12)
------------------

1.1.3 (2021-04-29)
------------------

1.1.2 (2021-04-08)
------------------
* Fix formatting errors
* Contributors: Tyler Weaver

1.1.1 (2020-10-13)
------------------
* [fix] python3 issues (`#2323 <https://github.com/ros-planning/moveit/issues/2323>`_)
* [maint] Cleanup MSA includes (`#2351 <https://github.com/ros-planning/moveit/issues/2351>`_)
* Contributors: Michael Görner, Robert Haschke

1.1.0 (2020-09-04)
------------------
* [feature] Benchmark combinations of predefined poses (`#1548 <https://github.com/ros-planning/moveit/issues/1548>`_)
* [feature] Support benchmarking of full planning pipelines (`#1531 <https://github.com/ros-planning/moveit/issues/1531>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Fix plot details, correcting xlabels positions and cleaning the graph (`#1658 <https://github.com/ros-planning/moveit/issues/1658>`_) (`#1668 <https://github.com/ros-planning/moveit/issues/1668>`_)
* [maint] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_, `#2004 <https://github.com/ros-planning/moveit/issues/2004>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Fix usage of panda_moveit_config (`#1904 <https://github.com/ros-planning/moveit/issues/1904>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] Do not install helper scripts in global bin destination (`#1704 <https://github.com/ros-planning/moveit/issues/1704>`_)
* [maint] Cleanup launch + config files (`#1631 <https://github.com/ros-planning/moveit/issues/1631>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Ayush Garg, Dave Coleman, Henning Kayser, Jonathan Binney, Mahmoud Ahmed Selim, Markus Vieth, Michael Görner, Robert Haschke, Sean Yen, Tyler Weaver, Yu, Yan

1.0.6 (2020-08-19)
------------------
* [maint] Adapt repository for splitted moveit_resources layout (`#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint] Migrate to clang-format-10, Fix warnings
* [maint] Optimize includes (`#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* Contributors: Markus Vieth, Robert Haschke

1.0.5 (2020-07-08)
------------------

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [feature] MoveIt benchmark improvements (`#1510 <https://github.com/ros-planning/moveit/issues/1510>`_)
  * Add pseudo experiment all_experiments to allow comparing all entries
  * Expose loadBenchmarkQueryData() for setting up custom queries
  * Add benchmark entry for comparing the 'final' result trajectory
  * Add trajectory similarity function to measure repeatability
  * Address requested changes
  * Fill empty fields in all_experiments
  * Improve variable and function names
  * Add helper function computeTrajectoryDistance()
* [maint]   Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint]   Windows build fixes
  * Fix header inclusion and other MSVC build errors (`#1636 <https://github.com/ros-planning/moveit/issues/1636>`_)
  * Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* Contributors: Henning Kayser, Michael Görner, Robert Haschke, Sean Yen, Yu, Yan

1.0.2 (2019-06-28)
------------------
* [maintenance] Removed unnecessary null pointer checks on deletion (`#1410 <https://github.com/ros-planning/moveit/issues/1410>`_)
* Contributors: Mahmoud Ahmed Selim

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Robert Haschke, Yu, Yan

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* Contributors: Dave Coleman, Robert Haschke

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* Contributors: Robert Haschke

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------
* [maintenance] Use locale independent conversion from double to string (`#1099 <https://github.com/ros-planning/moveit/issues/1099>`_)
* Contributors: Simon Schmeisser

0.10.2 (2018-10-24)
-------------------
* [capability] Benchmarking with different Motion Planners (STOMP, CHOMP, OMPL) (`#992 <https://github.com/ros-planning/moveit/issues/992>`_)
* [maintenance] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* Contributors: Mohmmad Ayman, Raghavender Sahdev, Robert Haschke, mike lautman

0.10.1 (2018-05-25)
-------------------
* migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* Contributors: Ian McMahon

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] benchmarks: always prefer local header over system installations `#630 <https://github.com/ros-planning/moveit/issues/630>`_
* Contributors: Jorge Nicho, v4hn

0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* [improve] Add install rule for examples, statistics script
* Contributors: Bence Magyar

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman

* [enhancement] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman

0.9.3 (2016-11-16)
------------------
* 0.9.3 (catkin_prepare_release again missed increment as http://answers.ros.org/question/245969/catkin_prepare_release-not-bumping-packages-in-a-certain-folder
* Merge pull request `#330 <https://github.com/ros-planning/moveit/issues/330>`_ from davetcoleman/kinetic-package.xml
  Updated package.xml maintainers and author emails
* Updated package.xml maintainers and author emails
* Contributors: Dave Coleman, Ian McMahon, Isaac I.Y. Saito

0.9.2 (2016-11-05)
------------------
* Versions that didn't get bumped by catkin_prepare_release.
* Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman, Isaac I.Y. Saito

0.9.1 (2016-10-21)
------------------
* add full VERSIONs / SONAMEs to all libraries (`#273 <https://github.com/ros-planning/moveit/issues/273>`_)
* More version consolidation for all package.xml in the moveit repo, which are not even going to be released (addition to https://github.com/ros-planning/moveit/commit/fcb8df12dead9e5a62b276c46bb0ac6e2411daca).
* More version down for release preparation to consolidate version of to-be released packages (addition to https://github.com/ros-planning/moveit/commit/56a3c6fcd39ca0b548998f04a688655d5133abe0)
* Cleanup readme (`#258 <https://github.com/ros-planning/moveit/issues/258>`_)
* Convert assorted internal shared_ptrs.
* Switch to std::unique_ptr (instead of boost::scoped_ptr).
* Use shared_ptr typedefs in BenchmarkExecutor.cpp
* Convert pluginlibs ``shared_ptrs`` to ``std::``
* Code review fixup
  Remove package benchmark_gui
  clang-format Benchmarks package
* Changes for warehouse refactor to single "moveit" repo
* New benchmarks suite from Rice
* [package.xml] Fix repository URLs. (`#194 <https://github.com/ros-planning/moveit/issues/194>`_)
* Use MOVEIT_CLASS_FORWARD for moveit classes in moveit_ros. (`#182 <https://github.com/ros-planning/moveit/issues/182>`_)
* Switched to C++11
* Contributors: Dave Coleman, Isaac I.Y. Saito, Maarten de Vries, Michael Görner, Sachin Chitta, root

0.8.3 (2016-08-21)
------------------
* [jade] More Manual adjustment of package.xml versions to 0.8.3. Remove moveit_ikfast for now (see https://github.com/ros-planning/moveit/issues/22#issuecomment-241199671). (`#96 <https://github.com/ros-planning/moveit/issues/96>`_)
* [Jade] Unify package version numbers (see https://github.com/davetcoleman/moveit_merge/issues/9). (`#79 <https://github.com/ros-planning/moveit/issues/79>`_)
* Modifications for warehouse_ros refactor (`#699 <https://github.com/ros-planning/moveit/issues/699>`_)
* Contributors: Dave Coleman, Isaac I.Y. Saito, Michael Ferguson

0.7.6 (2016-12-30)
------------------
* changelog 0.7.6
* Contributors: Isaac I.Y. Saito

0.7.5 (2016-12-25)
------------------
* changelog 0.7.5
* Contributors: Isaac I.Y. Saito

0.7.4 (2016-12-22)
------------------
* [indigo][changelog] Add blank 0.7.3 section to those that are missing it.
  Reason why doing this:
  - catkin_generate_changelog gets stuck for some reason so batch generating changelog isn't possible now.
  - Since this is the first release since 6 month ago for Indigo, lots of commit logs since then that shouldn't be wasted.
  - Decided to bump version of all packages uniformely to 0.7.4 in the hope for catkin_generate_changelog to function...
  - Turned out the accumulated commit logs are not retrieved...But we might as well want to move forward to fix https://github.com/ros-planning/moveit/issues/386
* Contributors: Isaac I.Y. Saito

0.7.3 (2016-12-20)
------------------
* add full VERSIONs / SONAMEs to all libraries (`#273 <https://github.com/ros-planning/moveit/issues/273>`_)
  This is similar to `#273 <https://github.com/ros-planning/moveit/issues/273>`_ / 0a7a895bb2ae9e171efa101f354826366fa5eaff,
  but hard-codes the version for each library instead of using the project's version.
  Thus, we have to bump the version of a library *manually* if we break ABI in a release.
  === Below is the original commit message of the patch targeting the kinetic branch.
  * add full VERSIONs / SONAMEs to all core libraries
  As a result the libraries do not install as `libmoveit_xyz.so` anymore,
  but as `libmoveit_xyz.so.${MOVEIT_VERSION}` and only provide `libmoveit_xyz.so`
  as a symlink pointing to the versioned file.
  Because this sets each library's SONAME to the *full version*, this enforces
  that *every* binary links with the versioned library file from now on and
  has to be relinked with *each* new release of MoveIt.
  The alternative would be to set the SONAME to `$MAJOR.$MINOR` and ignore the patch version,
  but because we currently stay with one `$MAJOR.$MINOR` number within each ROS distribution,
  we had (and likely will have) ABI changes in the `$PATCH` version releases too.
  The reason for this commit is that it is practically impossible to maintain full ABI compatibility
  within each ROS distribution and still add the the features/patches the community asks for.
  This has resulted in more than one ABI-incompatible MoveIt release in the recent past
  within a ROS distribution. Because the libraries have not been versioned up to now,
  there was no way to indicate the incompatible changes and users who did not rebuild
  their whole workspace with the new release encountered weird and hard-to-track segfaults
  or broken behavior.
  * add SONAMES to all non-core libraries too
* Auto code formatted Indigo branch using clang-format (`#313 <https://github.com/ros-planning/moveit/issues/313>`_)
* [package.xml] Fix repository URLs. (`#194 <https://github.com/ros-planning/moveit/issues/194>`_)
* Use MOVEIT_CLASS_FORWARD for moveit classes in moveit_ros. (`#182 <https://github.com/ros-planning/moveit/issues/182>`_) (`#183 <https://github.com/ros-planning/moveit/issues/183>`_)
* 0.7.2
* changelog 0.7.2
* 0.7.1
* changelog 0.7.1
* 0.7.0
* preparing for 0.7
* Removed trailing whitespace from entire repository
* Adding tf dep fixes `#572 <https://github.com/ros-planning/moveit/issues/572>`_
* 0.6.5
* update changelogs
* add myself as maintainer, update/remove old maintainer emails
* 0.6.4
* update changelogs
* install moveit_benchmark_statistics.py
* 0.6.3
* update changelogs
* Add missing include of scoped_ptr
* 0.6.2
* update changelog
* 0.6.1
* update changelog
* 0.6.0
* update changelog
* Removed PlanningContext clear before planning call
* 0.5.19
* 0.5.19
* Removed PlanningContext clear before planning call
* 0.5.19
* 0.5.19
* benchmarks: add missing include.
* Fix broken log & output statements.
  - Address [cppcheck: coutCerrMisusage] and [-Werror=format-extra-args] errors.
  - ROS_ERROR -> ROS_ERROR_NAMED.
  - Print size_t values portably.
* Address [-Wsign-compare] warning.
* 0.5.18
* update changelog
* 0.5.17
* update changelog
* update build system for ROS indigo
* update maintainer e-mail
* 0.5.16
* changes for release
* 0.5.15
* 0.5.14
* preparing for 0.5.14
* 0.5.13
* changelogs for release
* "0.5.12"
* Changelogs for release.
* "0.5.11"
* Changelogs for release.
* "0.5.10"
* update changelogs
* "0.5.9"
* changelogs for 0.5.9
* Cleaned up var names and debug output
* 0.5.8
* update changelog
* update changelog
* 0.5.7
* update changelog
* 0.5.6
* update changelog
* 0.5.5
* update changelog
* update changelog
* add missing include
* more porting to new APi
* more porting to new API
* 0.5.4
* update changelog
* 0.5.3
* update changelog
* make headers and author definitions aligned the same way; white space fixes
* 0.5.2
* update changelog
* 0.5.1
* update changelog
* update changelog
* 0.5.0
* white space fixes (tabs are now spaces)
* 0.4.5
* update changelog
* port to new base class for planning_interface (using planning contexts)
* Fixed per Ioan's code review
* 0.4.4
* add changelog files
* Code cleanup
* Merge branch 'groovy-devel' of github.com:davetcoleman/moveit_ros into groovy-devel
* merge fixes
* 0.4.3
* 0.4.2
* 0.4.1
* 0.4.0
* 0.3.32
* 0.3.31
* Changed for fractional factorial analysis
* More advanced parameter sweeping implmented, workspace bounds added
* Added parameter sweeping to benchmarking
* Added ability to store the goal name - the query, constraint, traj constraint, etc
* Added new command line arguments and ability to export all experiments to csv file
* remove obsolete files
* Fixed building of benchmarks for boost program_options 1.49.0.1
* 0.3.30
* 0.3.29
* 0.3.28
* 0.3.27
* 0.3.26
* using new namespace parameter in planner plugin configuration
* move benchmark gui to a separate package
* change default plugin name
* robustness fix
* refactor benchmarks into lib + executable
* using new namespace parameter in planner plugin configuration
* move benchmark gui to a separate package
* change default plugin name
* robustness fix
* refactor benchmarks into lib + executable
* add names for background jobs (eases debugging), changed the threading for how robot model is loaded (previous version had race conditions), fix some issues with incorrect usage of marker scale
* moved job management to planning scene rviz plugin, moved scene monitor initialization to background
* reorder some includes
* Fixed github url name
* Renamed variable to be more specific
* Added debug output if user tries wrong planner. This is useful if they forget the 'left_arm[' part
* Made help the default option if no params passed
* 0.3.25
* 0.3.24
* remove alignment tag from .ui, only supported in recent versions
* 0.3.23
* added goal existance checks
* show progress bar when loading a robot
* benchmark tool now includes goal offsets in the output config file
* Multiple fixes in benchmark tool. Added end effector offsets
* 0.3.22
* Use NonConst suffix
* Add multi-collision to PlanningScene
* Switch from CollisionWorld to World
* minor fix
* minor bugfix
* bugfix for benchmarking
* minor bugfix
* generate benchmark config file dialog
* new run benchmark dialog, functionality to be implemented
* fixes and interpolated ik visualization
* Merge branch 'groovy-devel' of https://github.com/ros-planning/moveit_ros into animate_trajectory
* renamed kinematic_model to robot_model, robot_model_loader to rdf_loader and planning_models_loader to robot_model_loader
* call to computeCartesianPath and visualize results
* 0.3.21
* 0.3.20
* 0.3.19
* build fixes for quantal
* 0.3.18
* missling lib for linking
* 0.3.17
* complete renaming process
* fix merge conflict
* support for cartesian trajectories in benchmarks
* load benchmark results for cartesian trajectories, only reachability for now
* sets trajectory waypoint names
* rename KinematicState to RobotState, KinematicTrajectory to RobotTrajectory
* Reset goals and trajectories when switching scenes
* Update trajectory regex when loading a scene
* History of most used databases
* Remember database url, ui fixes
* Store and load cartesian trajectories to/from the warehouse
* use new robot_trajectory lib
* waypoints for trajectories
* remove trajectories, ui fixes
* cleaning and authors
* use kinematic_state_visualization from render_tools
* fixed cmake warning
* Merge branch 'groovy-devel' of https://github.com/ros-planning/moveit_ros into marioprats/render_shapes_fix
* ui fixes
* Cleaning and better handling of signal connection
* 0.3.16
* specify start and endpoints of a trajectory
* started trajectories
* added robot_interaction and some fixes
* update to moveit changes
* ui improvements, some error checking
* Added goals and states. Switch between robots
* Double clicking on a scene loads it
* Set alpha to 1.0 by default. GUI fixes
* Use PlanningSceneDisplay for the scene monitor and rendering
* First version of benchmark tool
* API updates needed for planning interface changes in moveit_core; more importantly, plan_execution is now split into plan_with_sensing plan_execution; there is now the notion of an ExecutableMotionPlan, which can also represent results from pick& place actions; this allows us to reuse the replanning code & looking around code we had for planning in pick& place. Added callbacks for repairing motion plans
* 0.3.15
* Author names
* upadte build flags
* 0.3.14
* 0.3.13
* fixing typo
* 0.3.12
* Fix kinematic state initialization in kinematic benchmark
* 0.3.11
* 0.3.10
* 0.3.9
* 0.3.8
* 0.3.7
* 0.3.6
* 0.3.5
* 0.3.4
* overload getPlanningQueriesNames for regex use
* Include translation offset in the transform
* Added translation offsets and optionality
* Option to specify a rotation offset to apply to the goals
* Print progress info in call_benchmark
* added option for default number of ik attempts
* refactor benchmarking code
* a bit of cleaning
* call_kinematic_benchmark and benchmark_config refactor
* run_kinematic_benchmark service
* Output to file
* Initial kinematic bencharking tool
* fix buildtool tag
* fix `#83 <https://github.com/ros-planning/moveit/issues/83>`_
* warehouse now overwrites records with the same name
* 0.3.3
* Warn the user before removing constraints on the database
* handling exceptions during benchmarking as well
* Clear previous start states when loading a scene
* making some includes SYSTEM and re-adding link_directories
* fixes catkin cmake issues
* add timeout option
* add planning frame option
* remove references to PlannerCapabilities
* 0.3.2
* add the option to specify the link to constrain
* change how we return results to avoid apparent ros::service issue
* 0.3.1
* add group override option
* minor fixes for running benchmarks
* 0.3.0
* using the new warehouse functionality in the benchmarks
* 0.2.29
* 0.2.28
* 0.2.27
* 0.2.26
* update example
* add construction of demo dbs; multiple feature enhancements for warehouse + benchmarks
* add demos
* add demos
* minor fixes for loading plugins
* 0.2.25
* minor fix
* 0.2.24
* using specification of start states in benchmarking
* more work on computing benchmarks when goal is specified as poses
* separate benchmark lib
* 0.2.23
* 0.2.22
* 0.2.21
* 0.2.20
* 0.2.19
* 0.2.18
* 0.2.17
* 0.2.16
* 0.2.15
* 0.2.14
* 0.2.13
* 0.2.12
* 0.2.11
* 0.2.10
* 0.2.9
* 0.2.8
* 0.2.7
* 0.2.6
* 0.2.5
* 0.2.4
* 0.2.3
* 0.2.2
* add some command line options
* fix include locations again
* add dummy manipulation pkg; bump versions, fix install targets
* update linked libs, install python pkgs + bump version
* rename folders
* build system for moveit_ros_benchmarks
* moving things around
* Contributors: Acorn, Adam Leeper, Adolfo Rodriguez Tsouroukdissian, Benjamin Chrétien, Dave Coleman, Dave Hershberger, Ioan Sucan, Isaac I.Y. Saito, Mario Prats, Michael Ferguson, Michael Görner, Mr-Yellow, Paul Mathieu, Sachin Chitta, arjungm, isucan, v4hn
