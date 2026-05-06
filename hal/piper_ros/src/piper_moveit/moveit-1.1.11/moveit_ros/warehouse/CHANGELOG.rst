^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_warehouse
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.11 (2022-12-21)
-------------------

1.1.10 (2022-09-13)
-------------------
* Replace bind() with lambdas (`#3106 <https://github.com/ros-planning/moveit/issues/3106>`_)
* Contributors: Michael Görner, Robert Haschke

1.1.9 (2022-03-06)
------------------

1.1.8 (2022-01-30)
------------------

1.1.7 (2021-12-31)
------------------
* Switch to ``std::bind`` (`#2967 <https://github.com/ros-planning/moveit/issues/2967>`_)
* Contributors: Jochen Sprickerhof

1.1.6 (2021-11-06)
------------------
* Use newly introduced cmake macro ``moveit_build_options()`` from ``moveit_core``
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: Robert Haschke, pvanlaar

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
* [maint] Cleanup MSA includes (`#2351 <https://github.com/ros-planning/moveit/issues/2351>`_)
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski, Robert Haschke

1.1.0 (2020-09-04)
------------------
* [feature] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [fix] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Ayush Garg, Dave Coleman, Henning Kayser, Jonathan Binney, Max Krichenbauer, Robert Haschke, Sean Yen, Tyler Weaver, Yu, Yan

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
* [maint] Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint] Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint] Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [fix]   Fix potential use of uninitialized variable (`#1818 <https://github.com/ros-planning/moveit/issues/1818>`_)
* Contributors: Max Krichenbauer, Robert Haschke, Sean Yen, Yu, Yan

1.0.2 (2019-06-28)
------------------

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Robert Haschke, Yu, Yan

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* Contributors: Robert Haschke

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------

0.10.2 (2018-10-24)
-------------------
* [fix] Text refrences to MoveIt (`#1020 <https://github.com/ros-planning/moveit/issues/1020>`_)
* [enhancement] warehouse: added params for timeout + #retries (`#1008 <https://github.com/ros-planning/moveit/issues/1008>`_)
* [maintenance] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* Contributors: Mohmmad Ayman, Robert Haschke, dg-shadow, mike lautman

0.10.1 (2018-05-25)
-------------------
* [maintenance] migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* Contributors: Ian McMahon

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [package.xml] Add a release-maintainer. Cleanup `#649 <https://github.com/ros-planning/moveit/pull/649>`_

0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* [fix] warehouse services (`#474 <https://github.com/ros-planning/moveit/issues/474>`_)
* Contributors: Beatriz Leon

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* Contributors: Dave Coleman

0.9.4 (2017-02-06)
------------------
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman

0.9.3 (2016-11-16)
------------------

0.6.6 (2016-06-08)
------------------
* Removed trailing whitespace from entire repository
* comments addressed
* changed to global node handle so warehouse connection params work correctly
* camelCase
* removed extraneous includes
* added delete and rename
* now takes port + host from param server
* removed defunct code
* checks db connection is good
* services advertised
* Contributors: Dave Coleman, dg

0.6.5 (2015-01-24)
------------------
* update maintainers
* Contributors: Michael Ferguson

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------

0.6.0 (2014-10-27)
------------------

0.5.19 (2014-06-23)
-------------------

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update maintainer e-mail
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------

0.5.12 (2014-01-03)
-------------------

0.5.11 (2014-01-03)
-------------------

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------

0.5.8 (2013-10-11)
------------------

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* porting to new RobotState API

0.5.4 (2013-08-14)
------------------

* make headers and author definitions aligned the same way; white space fixes

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* white space fixes (tabs are now spaces)

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
