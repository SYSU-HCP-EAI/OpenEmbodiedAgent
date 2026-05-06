^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_simple_controller_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.11 (2022-12-21)
-------------------

1.1.10 (2022-09-13)
-------------------
* Replace bind() with lambdas (`#3106 <https://github.com/ros-planning/moveit/issues/3106>`_)
* Contributors: Michael Görner

1.1.9 (2022-03-06)
------------------

1.1.8 (2022-01-30)
------------------

1.1.7 (2021-12-31)
------------------
* ``simple_controller_manager``: add ``max_effort`` parameter to ``GripperCommand`` action (`#2984 <https://github.com/ros-planning/moveit/issues/2984>`_)
* Switch to ``std::bind`` (`#2967 <https://github.com/ros-planning/moveit/issues/2967>`_)
* Contributors: Jochen Sprickerhof, Rick Staa

1.1.6 (2021-11-06)
------------------
* Use newly introduced cmake macro ``moveit_build_options()`` from ``moveit_core``
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Facilitate spotting of empty controller names, using quotes (`#2761 <https://github.com/ros-planning/moveit/issues/2761>`_)
* Contributors: G.A. vd. Hoorn, Robert Haschke

1.1.5 (2021-05-23)
------------------

1.1.4 (2021-05-12)
------------------

1.1.3 (2021-04-29)
------------------

1.1.2 (2021-04-08)
------------------

1.1.1 (2020-10-13)
------------------
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski

1.1.0 (2020-09-04)
------------------
* [feature] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [feature] Allow different controllers for execution `#1832 <https://github.com/ros-planning/moveit/issues/1832>`_)
* [feature] ControllerManager: wait for done-callback (`#1783 <https://github.com/ros-planning/moveit/issues/1783>`_)
* [feature] Use CMAKE_CXX_STANDARD to enforce c++14 for portability (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [fix] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [fix] Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [fix] add missing space to log (`#1477 <https://github.com/ros-planning/moveit/issues/1477>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Dave Coleman, Henning Kayser, Jonathan Binney, Leroy Rügemer, Robert Haschke, Sean Yen, Tyler Weaver, Yu, Yan, llach

1.0.6 (2020-08-19)
------------------
* [maint] Migrate to clang-format-10
* Contributors: Robert Haschke

1.0.5 (2020-07-08)
------------------

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [fix]   Handle "default" parameter in MoveitControllerManagers
  MoveIt{Fake|Simple}ControllerManager::getControllerState() now correctly returns current state
* [maint] Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint] Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [fix]   `ControllerManager`: wait for done-callback (`#1783 <https://github.com/ros-planning/moveit/issues/1783>`_)
* Contributors: Robert Haschke, Sean Yen, Luca Lach

1.0.2 (2019-06-28)
------------------

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Yu, Yan

1.0.0 (2019-02-24)
------------------
* [maintenance] cleanup SimpleControllerManager https://github.com/ros-planning/moveit/pull/1352
* Contributors: Robert Haschke

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [maintenance] Code Cleanup (`#1196 <https://github.com/ros-planning/moveit/issues/1196>`_)
* Contributors: Robert Haschke

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------

0.10.2 (2018-10-24)
-------------------
* [maintenance] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* [maintenance] add minimum required pluginlib version (`#927 <https://github.com/ros-planning/moveit/issues/927>`_)
* Contributors: Mikael Arguedas, Mohmmad Ayman, Robert Haschke, mike lautman

0.10.1 (2018-05-25)
-------------------
* switch to ROS_LOGGER from CONSOLE_BRIDGE (`#874 <https://github.com/ros-planning/moveit/issues/874>`_)
* Contributors: Mikael Arguedas, Xiaojian Ma

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [capability][kinetic onward] optionally wait for controllers indefinitely (`#695 <https://github.com/ros-planning/moveit/issues/695>`_)
* Contributors: Bruno Brito, Michael Görner

0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------
* [fix] include order (`#529 <https://github.com/ros-planning/moveit/issues/529>`_)
* Contributors: Michael Goerner

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* [fix] assertion error when result not returned (`#378 <https://github.com/ros-planning/moveit/issues/378>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman, Michael Ferguson

0.9.3 (2016-11-16)
------------------

0.5.7 (2016-01-30)
------------------
* expose headers of moveit_simple_controller_manager
* Removed redundant logging information
* More informative warning message about multi-dof trajectories.
* Contributors: Dave Coleman, Dave Hershberger, Mathias Lüdtke

0.5.6 (2014-03-23)
------------------
* Allow simple controller manager to ignore virtual joints without failing
* Contributors: Dave Coleman

0.5.5 (2013-09-30)
------------------
* properly fill in the gripper command effort
* allow trajectories with >1 points, use the last point of any trajectory
* added better error reporting for FollowJointTrajectoryControllers

0.5.4 (2013-09-24)
------------------

0.5.3 (2013-09-23)
------------------
* make things a bit more robust
* make headers and author definitions aligned the same way; white space fixes
* fix `#1 <https://github.com/ros-planning/moveit_plugins/issues/1>`_

0.5.1 (2013-07-30)
------------------
* ns parameter is now action_ns, get rid of defaults

0.5.0 (2013-07-16)
------------------
* white space fixes (tabs are now spaces)

0.4.1 (2013-07-03)
------------------
* minor updates to package.xml

0.4.0 (2013-06-06)
------------------
* debs look good, bump to 0.4.0

0.1.0 (2013-06-05)
------------------
* add metapackage, clean up build in controller manager
* remove the now dead loaded controller stuff
* break out follow/gripper into separate headers
* initial working version
