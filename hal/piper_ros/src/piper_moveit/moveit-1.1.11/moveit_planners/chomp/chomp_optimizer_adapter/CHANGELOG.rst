^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_chomp_optimizer_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* CHOMP: Rename param 'clearence' to 'clearance' (`#2702 <https://github.com/ros-planning/moveit/issues/2702>`_, `#2707 <https://github.com/ros-planning/moveit/issues/2707>`_)
* Contributors: Martin Günther, Michael Görner, Robert Haschke

1.1.5 (2021-05-23)
------------------

1.1.4 (2021-05-12)
------------------

1.1.3 (2021-04-29)
------------------

1.1.2 (2021-04-08)
------------------
* Sanitize CHOMP initialization method parameter (`#2540 <https://github.com/ros-planning/moveit/issues/2540>`_)
* Contributors: Cong Liu

1.1.1 (2020-10-13)
------------------

1.1.0 (2020-09-04)
------------------
* [feature] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [feature] Allow ROS namespaces for planning request adapters (`#1530 <https://github.com/ros-planning/moveit/issues/1530>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [fix] Fix Chomp planning adapter (`#1525 <https://github.com/ros-planning/moveit/issues/1525>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Dave Coleman, Henning Kayser, Robert Haschke, Sean Yen, Tyler Weaver, Yu, Yan

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
* [maint] Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* Contributors: Robert Haschke, Sean Yen, Yu, Yan

1.0.2 (2019-06-28)
------------------
* [fix] Chomp planning adapter: Fix spurious error message Fix "Found empty JointState message"
* Contributors: Robert Haschke

1.0.1 (2019-03-08)
------------------
* [fix] segfault in chomp adapter (`#1377 <https://github.com/ros-planning/moveit/issues/1377>`_)
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
* [maintenance] Moved CHOMP optimizer into own package (`#1251 <https://github.com/ros-planning/moveit/issues/1251>`_)
* rename default_planner_request_adapters/CHOMPOptimizerAdapter -> chomp/OptimizerAdapter
* Contributors: Robert Haschke
