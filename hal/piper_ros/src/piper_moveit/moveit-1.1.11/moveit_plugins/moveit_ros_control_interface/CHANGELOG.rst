^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_control_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.11 (2022-12-21)
-------------------

1.1.10 (2022-09-13)
-------------------

1.1.9 (2022-03-06)
------------------

1.1.8 (2022-01-30)
------------------

1.1.7 (2021-12-31)
------------------

1.1.6 (2021-11-06)
------------------
* Use newly introduced cmake macro ``moveit_build_options()`` from ``moveit_core``
* Fix reversed check in switchControllers (`#2726 <https://github.com/ros-planning/moveit/issues/2726>`_)
* Contributors: Nathan Brooks, Robert Haschke

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
* [feature] Remove support for Indigo's ros_control (`#2128 <https://github.com/ros-planning/moveit/issues/2128>`_)
* [feature] Add support for pos_vel_controllers and pos_vel_acc_controllers (`#1806 <https://github.com/ros-planning/moveit/issues/1806>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] clang-tidy-fix `modernize-loop-convert` to entire code base (`#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Dave Coleman, Henning Kayser, Jonathan Binney, Robert Haschke, Sandro Magalhães, Sean Yen, Tyler Weaver, Yu, Yan

1.0.6 (2020-08-19)
------------------

1.0.5 (2020-07-08)
------------------

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [maint]   Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint]   Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [feature] Add support for pos_vel_controllers and pos_vel_acc_controllers (`#1806 <https://github.com/ros-planning/moveit/issues/1806>`_)
* Contributors: Robert Haschke, Sandro Magalhães, Sean Yen

1.0.2 (2019-06-28)
------------------

1.0.1 (2019-03-08)
------------------

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

0.10.1 (2018-05-25)
-------------------

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------

0.9.9 (2017-08-06)
------------------
* [improve] add backward compatibility patch for indigo (`#551 <https://github.com/ros-planning/moveit/issues/551>`_)
* Contributors: Michael Görner

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------

0.9.5 (2017-03-08)
------------------

0.9.4 (2017-02-06)
------------------
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman, Michael Goerner

0.9.3 (2016-11-16)
------------------

0.5.7 (2016-01-30)
------------------
* C++03 conforming nested templates
* fixed typo, added example config
* added brief decription tags
* formatted code to roscpp style
* improved documentation
* introduced getAbsName
* Added missing lock
* pre-allocate handles
* fixed typos
* set version to match the others
* fixed a lot of typos
* Intitial version of moveit_ros_control_interface package
* Contributors: Mathias Luedtke
