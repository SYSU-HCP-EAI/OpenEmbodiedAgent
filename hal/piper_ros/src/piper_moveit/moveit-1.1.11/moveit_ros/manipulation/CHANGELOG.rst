^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_manipulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Switch to ``std::bind`` (`#2967 <https://github.com/ros-planning/moveit/issues/2967>`_)
* Contributors: Jochen Sprickerhof

1.1.6 (2021-11-06)
------------------
* Use newly introduced cmake macro ``moveit_build_options()`` from ``moveit_core``
* Introduce a reference frame for collision objects (`#2037 <https://github.com/ros-planning/moveit/issues/2037>`_)
* Add missing dependencies to generated dynamic_reconfigure headers (`#2772 <https://github.com/ros-planning/moveit/issues/2772>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: Felix von Drigalski, Mathias Lüdtke, Robert Haschke, pvanlaar

1.1.5 (2021-05-23)
------------------

1.1.4 (2021-05-12)
------------------

1.1.3 (2021-04-29)
------------------

1.1.2 (2021-04-08)
------------------
* Order PlaceLocations by quality during planning (`#2378 <https://github.com/ros-planning/moveit/issues/2378>`_)
* Contributors: Markus Vieth

1.1.1 (2020-10-13)
------------------
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski

1.1.0 (2020-09-04)
------------------

1.0.6 (2020-08-19)
------------------
* [maint] Migrate to clang-format-10
* [maint] Optimize includes (`#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* Contributors: Markus Vieth, Robert Haschke

1.0.5 (2020-07-15)
------------------

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
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
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
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
* [fix] Eigen alignment issuses due to missing aligned allocation (`#1039 <https://github.com/ros-planning/moveit/issues/1039>`_)
* [enhancement] Add info messages to pick and place routine (`#1004 <https://github.com/ros-planning/moveit/issues/1004>`_)
* [maintenance] add minimum required pluginlib version (`#927 <https://github.com/ros-planning/moveit/issues/927>`_)
* Contributors: Adrian Zwiener, Felix von Drigalski, Mikael Arguedas, Mohmmad Ayman, Robert Haschke, mike lautman

0.10.1 (2018-05-25)
-------------------
* [maintenance] migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* Contributors: Ian McMahon

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [capability][kinetic onward][moveit_ros_planning_interface] Adapt pick pipeline to function without object (`#599 <https://github.com/ros-planning/moveit/issues/599>`_)
* Contributors: 2scholz

0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* [fix] Set planning frame correctly in evaluation of reachable and valid pose filter (`#476 <https://github.com/ros-planning/moveit/issues/476>`_)
* Contributors: Yannick Jonetzko

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* [fix] race conditions when updating PlanningScene (`#350 <https://github.com/ros-planning/moveit/issues/350>`_)
* Contributors: Dave Coleman, Robert Haschke

0.9.3 (2016-11-16)
------------------
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* [enhancement] remove grasp service support from pick_place's fillGrasp (`#328 <https://github.com/ros-planning/moveit/issues/328>`_)
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon, Michael Goerner

0.9.2 (2016-11-05)
------------------

0.6.6 (2016-06-08)
------------------
* replaced cmake_modules dependency with eigen
* [jade] eigen3 adjustment
* Removed trailing whitespace from entire repository
* Contributors: Dave Coleman, Isaac I.Y. Saito, Robert Haschke

0.6.5 (2015-01-24)
------------------
* update maintainers
* Contributors: Michael Ferguson

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------
* trivial fixes for warnings
* use named logging for manipulation components
* Contributors: Michael Ferguson

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------

0.6.0 (2014-10-27)
------------------
* Fix bug in place-planning where attached object was not considered in plan.
* Contributors: Dave Hershberger, Sachin Chitta

0.5.19 (2014-06-23)
-------------------
* fixes `#461 <https://github.com/ros-planning/moveit_ros/issues/461>` and a potential segfault
* Now check if there is a time for the gripper trajectory.
  If there is a time, use that one on the gripper trajectory, if not, keep
  with the previous strategy of using a default time.
* Contributors: Michael Ferguson, Sammy Pfeiffer

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* fix merge
* refactor how we use params for pick&place
* set the pose frame so we don't get a crash in approach&translate
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------
* ApproachAndTranslateStage dynamic reconfigure bug fixed.
  The bug shows up in test code, where it becomes apparent that creating a ros::NodeHandle
  in a static initializer makes it very difficult to call ros::init() before creating
  the first NodeHandle.
* Contributors: Dave Hershberger

0.5.12 (2014-01-03)
-------------------

0.5.11 (2014-01-03)
-------------------
* Fixed internal comment.
* Contributors: Dave Hershberger

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* corrected maintainers email
* PickPlace: Added comments, renamed variables to be more specific
* use ROS_ERROR instead of logError

0.5.8 (2013-10-11)
------------------
* fix `#331 <https://github.com/ros-planning/moveit_ros/issues/331>`_.
* try to identify the eef and group based on the attached object name

0.5.7 (2013-10-01)
------------------
* use the fact we know an eef must be defined for the place action to simplify code
* abort place if eef cannot be determined, fixes `#325 <https://github.com/ros-planning/moveit_ros/issues/325>`_.
* fix segfault in approach translate

0.5.6 (2013-09-26)
------------------
* dep on manipulation_msgs needs to be added here

0.5.5 (2013-09-23)
------------------
* use new messages for pick & place
* porting to new RobotState API

0.5.4 (2013-08-14)
------------------

* make headers and author definitions aligned the same way; white space fixes
* adding manipulation tab, fixed bugs in planning scene interface

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
* bugfixes
