^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_move_group
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.11 (2022-12-21)
-------------------
* Merge PR `#3262 <https://github.com/ros-planning/moveit/issues/3262>`_: Short-circuit planning adapters

  - Early return from failing planning adapters, namely FixStartStateCollision and FixStartStatePathConstraint
  - Propagate the error code via `MotionPlanResponse::error_code\_`
  - Add string translations for all error codes
* Cleanup translation of MoveItErrorCode to string

  - Move default code to moveit_core/utils
  - Override defaults in existing getActionResultString()
  - Provide translations for all error codes defined in moveit_msgs
* Drop shortcut avoiding planning when all constraints are already met (`#3228 <https://github.com/ros-planning/moveit/issues/3228>`_)

  When calling the PlanAndExcute action, there was a shortcut avoiding planning (and execution) when all goal constraints were already met.
  However, this check also succeeded when there was an error, e.g. resolving link names.
  As proper link resolution might require some planning request adapters to be executed, e.g. ResolveConstraintFrames, we should not skip planning.
  Note, calling the ``PlanOnly`` action, didn't have this shortcut as well, resulting in inconsistent behavior.
* Contributors: Robert Haschke

1.1.10 (2022-09-13)
-------------------
* Limit Cartesian speed for link(s) (`#2856 <https://github.com/ros-planning/moveit/issues/2856>`_)
* Optionally enable dynamics monitoring in move_group node (`#3137 <https://github.com/ros-planning/moveit/issues/3137>`_)
* Replace bind() with lambdas (`#3106 <https://github.com/ros-planning/moveit/issues/3106>`_)
* Contributors: Michael Görner

1.1.9 (2022-03-06)
------------------
* Fix missing include (`#3051 <https://github.com/ros-planning/moveit/issues/3051>`_)
* Contributors: Tobias Fischer

1.1.8 (2022-01-30)
------------------
* Improve loading of planning pipelines (`#3036 <https://github.com/ros-planning/moveit/issues/3036>`_)

  Ensure that planning pipelines considered in `~/planning_pipelines/*` actually have a `planning_plugin` parameter defined.
  Otherwise, issue an error message.
* Contributors: Robert Haschke

1.1.7 (2021-12-31)
------------------
* Switch to ``std::bind`` (`#2967 <https://github.com/ros-planning/moveit/issues/2967>`_)
* Contributors: Jochen Sprickerhof

1.1.6 (2021-11-06)
------------------
* Use newly introduced cmake macro ``moveit_build_options()`` from ``moveit_core``
* Introduce a reference frame for collision objects (`#2037 <https://github.com/ros-planning/moveit/issues/2037>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: Felix von Drigalski, Robert Haschke, pvanlaar

1.1.5 (2021-05-23)
------------------

1.1.4 (2021-05-12)
------------------

1.1.3 (2021-04-29)
------------------

1.1.2 (2021-04-08)
------------------
* Fix formatting errors
* Support multiple planning pipelines with MoveGroup via MoveItCpp (`#2127 <https://github.com/ros-planning/moveit/issues/2127>`_)
* Run multiple planning pipelines with MoveGroup using MoveItCpp
* Fix missing isEmpty check in compute_ik service (`#2544 <https://github.com/ros-planning/moveit/issues/2544>`_)
* Contributors: Henning Kayser, Michael Görner, Tyler Weaver

1.1.1 (2020-10-13)
------------------
* [fix] Let the max number of contacts be the amount of world objects + link models with geometry (`#2355 <https://github.com/ros-planning/moveit/issues/2355>`_)
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski, Loy van Beek, Michael Görner

1.1.0 (2020-09-04)
------------------
* [feature] Start new joint_state_publisher_gui on param use_gui (`#2257 <https://github.com/ros-planning/moveit/issues/2257>`_)
* [feature] TfPublisher: fixup and add attached collsion objects (`#1792 <https://github.com/ros-planning/moveit/issues/1792>`_)
* [feature] move_group capability for publishing planning scene frames to the tf system (`#1761 <https://github.com/ros-planning/moveit/issues/1761>`_)
* [feature] get_planning_scene_service: return full scene when nothing was requested (`#1424 <https://github.com/ros-planning/moveit/issues/1424>`_)
* [feature] Separate source file for CartesianInterpolator (`#1149 <https://github.com/ros-planning/moveit/issues/1149>`_)
* [fix]   Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix]   Fix TfPublisher subframe publishing (`#2002 <https://github.com/ros-planning/moveit/issues/2002>`_)
* [maint] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_) (`#2004 <https://github.com/ros-planning/moveit/issues/2004>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Move isEmpty() test functions to moveit_core/utils (`#1627 <https://github.com/ros-planning/moveit/issues/1627>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Dave Coleman, Mike Lautman, Robert Haschke, Felix von Drigalski, Jens P, Jonathan Binney, JonasTietz, Michael Görner, Tyler Weaver, Yoan Mollard, Yu, Yan, v4hn

1.0.6 (2020-08-19)
------------------
* [maint] Adapt repository for splitted moveit_resources layout (`#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint] Migrate to clang-format-10
* Contributors: Robert Haschke, Michael Görner

1.0.5 (2020-07-08)
------------------

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [maint] Move `get_planning_scene` service into `PlanningSceneMonitor` for reusability (`#1854 <https://github.com/ros-planning/moveit/issues/1854>`_)
* [maint] Cleanup move_group capabilities (`#1515 <https://github.com/ros-planning/moveit/issues/1515>`_)
* [maint] Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint] Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint] Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* Contributors: Robert Haschke, Sean Yen, Yu, Yan

1.0.2 (2019-06-28)
------------------

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Yu, Yan

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* [improve] Remove (redundant) random seeding and #attempts from RobotState::setFromIK() as the IK solver perform random seeding themselves. `#1288 <https://github.com/ros-planning/moveit/issues/1288>`_
* Contributors: Robert Haschke

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [fix] Fixed various memory leaks (`#1104 <https://github.com/ros-planning/moveit/issues/1104>`_)
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* [maintenance] Code Cleanup
  * `#1180 <https://github.com/ros-planning/moveit/issues/1180>`_
  * `#1196 <https://github.com/ros-planning/moveit/issues/1196>`_
* Contributors: Dave Coleman, Robert Haschke

0.10.2 (2018-10-24)
-------------------
* [fix] PlanningSceneMonitor lock `#1033 <https://github.com/ros-planning/moveit/issues/1033>`_: Fix `#868 <https://github.com/ros-planning/moveit/issues/868>`_ (`#1057 <https://github.com/ros-planning/moveit/issues/1057>`_)
* [maintenance] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* [maintenance] add minimum required pluginlib version (`#927 <https://github.com/ros-planning/moveit/issues/927>`_)
* Contributors: Mikael Arguedas, Mohmmad Ayman, Robert Haschke, mike lautman

0.10.1 (2018-05-25)
-------------------
* Remove deprecated ExecuteTrajectoryServiceCapability (`#833 <https://github.com/ros-planning/moveit/issues/833>`_)
* migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* Add namespace capabilities to moveit_commander (`#835 <https://github.com/ros-planning/moveit/issues/835>`_)
* [fix] MoveAction capability can drop cancel request if it is sent shortly after goal is sent (`#756 <https://github.com/ros-planning/moveit/issues/756>`_)
* Contributors: Dave Coleman, Ian McMahon, Mikael Arguedas, Robert Haschke, Will Baker

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] always return true in MoveGroupPlanService callback `#674 <https://github.com/ros-planning/moveit/pull/674>`_
* [improve] adding swp's to gitignore and removing redundant capabilites from capability_names.h (`#704 <https://github.com/ros-planning/moveit/issues/704>`_)
* Contributors: Mike Lautman, Shingo Kitagawa

0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------

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
* [fix] race conditions when updating PlanningScene (`#350 <https://github.com/ros-planning/moveit/issues/350>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman, Robert Haschke

0.9.3 (2016-11-16)
------------------
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------

0.6.6 (2016-06-08)
------------------
* added missing validity check
  iterator found with `configs.find()` needs to be validated before use...
* Removed trailing whitespace from entire repository
* moved planner params services into existing capability QueryPlannerInterfaces
* capability plugin MoveGroupPlannerParamsService to get/set planner params
* Fixed bug(?) in move_group::MoveGroupKinematicsService::computeIK link name selection.
* Contributors: Dave Coleman, Mihai Pomarlan, Robert Haschke

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
* Merge pull request `#522 <https://github.com/ros-planning/moveit_ros/issues/522>`_ from mikeferguson/indigo-devel
  add dependency on std_srvs (part of octomap clearing service)
* Contributors: Ioan A Sucan, Michael Ferguson

0.6.1 (2014-10-31)
------------------

0.6.0 (2014-10-27)
------------------
* Added move_group capability for clearing octomap.
* Contributors: Dave Hershberger, Sachin Chitta

0.5.19 (2014-06-23)
-------------------
* Address [cppcheck: duplicateIf] error.
  The same condition was being checked twice, and the same action was being taken.
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update maintainer e-mail
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------
* empty state should be a diff state, otherwise attached objects are deleted
* Contributors: sachinc

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
* Re-ordered movegroup's initialization, so capabilities start after monitors.
* correcting maintainer email
* Added planning feedback to gui, refactored states tab

0.5.8 (2013-10-11)
------------------

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* porting to new RobotState API
* more console output

0.5.4 (2013-08-14)
------------------

* make headers and author definitions aligned the same way; white space fixes
* Dependency for move_group_capabilities_base fixed.

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* white space fixes (tabs are now spaces)
* port to new base class for planning_interface (using planning contexts)

0.4.5 (2013-07-03)
------------------
* Fixed for moveit_msgs/JointLimits.h no such file or directory

0.4.4 (2013-06-26)
------------------
* fix `#259 <https://github.com/ros-planning/moveit_ros/issues/259>`_ and `#260 <https://github.com/ros-planning/moveit_ros/issues/260>`_.
