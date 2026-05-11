^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_planning_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.11 (2022-12-21)
-------------------

1.1.10 (2022-09-13)
-------------------
* Limit Cartesian speed for link(s) (`#2856 <https://github.com/ros-planning/moveit/issues/2856>`_)
* Generalize computeCartesianPath() to consider a link_offset (`#3197 <https://github.com/ros-planning/moveit/issues/3197>`_)
* Validate JointState argument to `moveit_commander.set_joint_value_target` (`#3187 <https://github.com/ros-planning/moveit/issues/3187>`_)
* Add dual arm test (`#3119 <https://github.com/ros-planning/moveit/issues/3119>`_)
* Replace bind() with lambdas (`#3106 <https://github.com/ros-planning/moveit/issues/3106>`_)
* Replace obsolete distutils.core with setuptools (`#3103 <https://github.com/ros-planning/moveit/issues/3103>`_)
* Contributors: Filip Sund, Michael Görner, Robert Haschke, Stephanie Eng, cambel, v4hn

1.1.9 (2022-03-06)
------------------

1.1.8 (2022-01-30)
------------------

1.1.7 (2021-12-31)
------------------
* Move ``MoveItErrorCode`` class to ``moveit_core`` (`#3009 <https://github.com/ros-planning/moveit/issues/3009>`_)
* Fix uninitialized ``RobotState`` in ``MoveGroupInterface`` (`#3008 <https://github.com/ros-planning/moveit/issues/3008>`_)
* Switch to ``std::bind`` (`#2967 <https://github.com/ros-planning/moveit/issues/2967>`_)
* Contributors: Captain Yoshi, Jafar Abdi, Jochen Sprickerhof

1.1.6 (2021-11-06)
------------------
* Use test_environment.launch in unittests (`#2949 <https://github.com/ros-planning/moveit/issues/2949>`_)
* Use newly introduced cmake macro ``moveit_build_options()`` from ``moveit_core``
* Add missing replan/look options to interface (`#2892 <https://github.com/ros-planning/moveit/issues/2892>`_)
* PSI: get object.pose from new msg field (`#2877 <https://github.com/ros-planning/moveit/issues/2877>`_)
* Introduce a reference frame for collision objects (`#2037 <https://github.com/ros-planning/moveit/issues/2037>`_)
* Fix trajectory constraints for ``moveit_commander`` (`#2429 <https://github.com/ros-planning/moveit/issues/2429>`_)
* ``MGI::setStartState``: Only fetch current state when new state is diff (`#2775 <https://github.com/ros-planning/moveit/issues/2775>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: Felix von Drigalski, Gauthier Hentz, Kevin Chang, Michael Görner, Robert Haschke, pvanlaar

1.1.5 (2021-05-23)
------------------
* Allow selecting planning pipeline in MotionSequenceAction (`#2657 <https://github.com/ros-planning/moveit/issues/2657>`_)
* Contributors: Felix von Drigalski

1.1.4 (2021-05-12)
------------------
* planning_interface: synchronize async interfaces in unit tests (`#2640 <https://github.com/ros-planning/moveit/issues/2640>`_)
* Contributors: Michael Görner

1.1.3 (2021-04-29)
------------------
* Add planning_pipeline_id setting to Python MGI (`#2622 <https://github.com/ros-planning/moveit/issues/2622>`_)
* Fix docstring in MGI API (`#2626 <https://github.com/ros-planning/moveit/issues/2626>`_)
* Contributors: Felix von Drigalski, Michael Görner

1.1.2 (2021-04-08)
------------------
* Fix formatting errors
* Support multiple planning pipelines with MoveGroup via MoveItCpp (`#2127 <https://github.com/ros-planning/moveit/issues/2127>`_)
* Move moveit_cpp to moveit_ros_planning
* Add get_active_joint_names (`#2533 <https://github.com/ros-planning/moveit/issues/2533>`_)
* Add debugging log statement for a common error (`#2509 <https://github.com/ros-planning/moveit/issues/2509>`_)
* Replaced eigen+kdl conversions with tf2_eigen + tf2_kdl (`#2472 <https://github.com/ros-planning/moveit/issues/2472>`_)
* Improve robustness of subframes test (`#2488 <https://github.com/ros-planning/moveit/issues/2488>`_)
* Improve robustness of move group interface test (`#2484 <https://github.com/ros-planning/moveit/issues/2484>`_)
* Fix scaling factor parameter names (`#2452 <https://github.com/ros-planning/moveit/issues/2452>`_)
* Python interface improvements. Fix `#1966 <https://github.com/ros-planning/moveit/issues/1966>`_, add enforceBounds (`#2356 <https://github.com/ros-planning/moveit/issues/2356>`_)
* Unit Test for ByteString-based ROS msg conversion (`#2362 <https://github.com/ros-planning/moveit/issues/2362>`_)
* Contributors: Bjar Ne, Boston Cleek, Henning Kayser, Peter Mitrano, Shota Aoki, Tyler Weaver, petkovich

1.1.1 (2020-10-13)
------------------
* [feature] moveit_cpp: more informative error message, cover another potential failure condition. (`#2336 <https://github.com/ros-planning/moveit/issues/2336>`_)
* [fix] Make GILReleaser exception-safe (`#2363 <https://github.com/ros-planning/moveit/issues/2363>`_)
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* [maint] Replace panda_moveit_config -> moveit_resources_panda_moveit_config (`#2300 <https://github.com/ros-planning/moveit/issues/2300>`_)
* Contributors: AndyZe, Bjar Ne, Felix von Drigalski, Robert Haschke

1.1.0 (2020-09-04)
------------------
* [feature] Use Eigen::Transform::linear() instead of rotation() (`#1964 <https://github.com/ros-planning/moveit/issues/1964>`_)
* [feature] move_group pick place test (`#2031 <https://github.com/ros-planning/moveit/issues/2031>`_)
* [feature] Check for grasp service - general cleanup MGI (`#2077 <https://github.com/ros-planning/moveit/issues/2077>`_)
* [feature] Integration test to defend subframe tutorial (`#1757 <https://github.com/ros-planning/moveit/issues/1757>`_)
* [feature] Release Python GIL for C++ calls (`#1947 <https://github.com/ros-planning/moveit/issues/1947>`_)
* [feature] Add default velocity/acceleration scaling factors (`#1890 <https://github.com/ros-planning/moveit/issues/1890>`_)
* [feature] Improve move_group_interface's const correctness (`#1715 <https://github.com/ros-planning/moveit/issues/1715>`_)
* [feature] Add get_jacobian_matrix to moveit_commander (`#1501 <https://github.com/ros-planning/moveit/issues/1501>`_)
* [feature] Add named frames to CollisionObjects (`#1439 <https://github.com/ros-planning/moveit/issues/1439>`_)
* [feature] Added GILRelease to pick and place (`#2272 <https://github.com/ros-planning/moveit/issues/2272>`_)
* [feature] Add missing variants of place from list of PlaceLocations and Poses in the python interface (`#2231 <https://github.com/ros-planning/moveit/issues/2231>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Resolve PSI lock-up in RViz display (`#1951 <https://github.com/ros-planning/moveit/issues/1951>`_)
* [fix] Fix flaky moveit_cpp test (`#1781 <https://github.com/ros-planning/moveit/issues/1781>`_)
* [fix] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] Fix a parameter mix-up in moveit_cpp loading (`#2187 <https://github.com/ros-planning/moveit/issues/2187>`_)
* [maint] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [maint] update dependencies for python3 in noetic (`#2131 <https://github.com/ros-planning/moveit/issues/2131>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_, `#1586 <https://github.com/ros-planning/moveit/issues/1586>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Fix some clang tidy issues (`#2004 <https://github.com/ros-planning/moveit/issues/2004>`_)
* [maint] export  moveit_py_bindings_tools library (`#1970 <https://github.com/ros-planning/moveit/issues/1970>`_)
* [maint] Fix usage of panda_moveit_config (`#1904 <https://github.com/ros-planning/moveit/issues/1904>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Fix typo in cmake file (`#1857 <https://github.com/ros-planning/moveit/issues/1857>`_)
* [maint] Reduce console output warnings (`#1845 <https://github.com/ros-planning/moveit/issues/1845>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* [maint] improve [get|set]JointValueTarget in python wrapper (`#858 <https://github.com/ros-planning/moveit/issues/858>`_)
* [maint] moveit_commander.MoveGroupInterface.plan() to better align with C++ MoveGroup::plan() (`#790 <https://github.com/ros-planning/moveit/issues/790>`_)
* Contributors: AndyZe, Ayush Garg, Bence Magyar, Bjar Ne, Dave Coleman, Felix von Drigalski, Gerard Canal, Guilhem Saurel, Henning Kayser, Jafar Abdi, JafarAbdi, Jere Liukkonen, Jonathan Binney, Kunal Tyagi, Luca Rinelli, Mahmoud Ahmed Selim, Markus Vieth, Martin Pecka, Masaki Murooka, Michael Ferguson, Michael Görner, Niklas Fiedler, Robert Haschke, Ryosuke Tajima, Sean Yen, Tyler Weaver, Yeshwanth, Yu, Yan, mvieth, v4hn

1.0.6 (2020-08-19)
------------------
* [maint]   Adapt repository for splitted moveit_resources layout (`#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint]   Migrate to clang-format-10, Fix warnings
* [maint]   Optimize includes (`#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* [feature] Exposed parameter wait_for_servers and getPlannerId() API in MoveGroup's Python API (`#2201 <https://github.com/ros-planning/moveit/issues/2201>`_)
* Contributors: Gerard Canal, Markus Vieth, Robert Haschke, Michael Görner

1.0.5 (2020-07-08)
------------------
* [maint]   Remove dependency on panda_moveit_config (#2194 <https://github.com/ros-planning/moveit/issues/2194>`_, #2197 <https://github.com/ros-planning/moveit/issues/2197>`_)
* [maint]   Adapt linking to eigenpy (`#2118 <https://github.com/ros-planning/moveit/issues/2118>`_)
* [maint]   Replace robot_model and robot_state namespaces with moveit::core (`#2135 <https://github.com/ros-planning/moveit/issues/2135>`_)
* [feature] PlanningComponent: Load plan_request_params (`#2033 <https://github.com/ros-planning/moveit/issues/2033>`_)
* [feature] MoveItCpp: a high-level C++ planning API (`#1656 <https://github.com/ros-planning/moveit/issues/1656>`_)
* [fix]     Validate action client pointer before access
* [fix]     Wait and check for the grasp service
* [maint]   Add tests for move_group interface (`#1995 <https://github.com/ros-planning/moveit/issues/1995>`_)
* Contributors: AndyZe, Henning Kayser, Jafar Abdi, Michael Görner, Robert Haschke, Tyler Weaver, Yeshwanth

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [feature] `MoveGroupInterface`: Add execution methods for moveit_msgs::RobotTrajectory (`#1955 <https://github.com/ros-planning/moveit/issues/1955>`_)
* [feature] Allow to instantiate a `PlanningSceneInterface` w/ and w/o a running `move_group` node
* [fix]     Release Python `GIL` for C++ calls (`#1947 <https://github.com/ros-planning/moveit/issues/1947>`_)
* [feature] Expose reference_point_position parameter in getJacobian() (`#1595 <https://github.com/ros-planning/moveit/issues/1595>`_)
* [feature] `MoveGroupInterface`: Expose `constructPickGoal` and `constructPlaceGoal` (`#1498 <https://github.com/ros-planning/moveit/issues/1498>`_)
* [feature] `python MoveGroupInterface`: Added custom time limit for `wait_for_servers()` (`#1444 <https://github.com/ros-planning/moveit/issues/1444>`_)
* [maint]   Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint]   Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint]   Improve Python 3 compatibility (`#1870 <https://github.com/ros-planning/moveit/issues/1870>`_)
  * Replaced StringIO with BytesIO for python msg serialization
  * Use py_bindings_tools::ByteString as byte-based serialization buffer on C++ side
* [feature] Export moveit_py_bindings_tools library
* [maint]   Fix various build issues on Windows
  * Use `.pyd` as the output suffix for Python module on Windows. (`#1637 <https://github.com/ros-planning/moveit/issues/1637>`_)
  * Favor ros::Duration.sleep over sleep. (`#1634 <https://github.com/ros-planning/moveit/issues/1634>`_)
  * Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [maint]   Updated deprecation method: MOVEIT_DEPRECATED -> [[deprecated]] (`#1748 <https://github.com/ros-planning/moveit/issues/1748>`_)
* [maint]   `eigenpy`: switched to system package (`#1737 <https://github.com/ros-planning/moveit/issues/1737>`_)
* [featue]  `PlanningSceneInterface`: wait for its two services
* [feature] Select time parametrization algorithm in retime_trajectory (`#1508 <https://github.com/ros-planning/moveit/issues/1508>`_)
* Contributors: Bjar Ne, Felix von Drigalski, Kunal Tyagi, Luca Rinelli, Masaki Murooka, Michael Görner, Niklas Fiedler, Robert Haschke, Sean Yen, Yu, Yan, mvieth, v4hn

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
* [improve] Remove (redundant) random seeding and #attempts from RobotState::setFromIK() as the IK solver perform random seeding themselves. `#1288 <https://github.com/ros-planning/moveit/issues/1288>`_
* Contributors: Dave Coleman, Robert Haschke

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------
* [fix] Fixed destruction order of shared tf2::Buffer / tf2::TransformListener (`#1261 <https://github.com/ros-planning/moveit/pull/1261>`_)
* Contributors: Robert Haschke

0.10.6 (2018-12-09)
-------------------
* [fix] Fixed various memory leaks (`#1104 <https://github.com/ros-planning/moveit/issues/1104>`_)
  * SharedStorage: Use weak_ptrs for caching
* [enhancement] Add getMoveGroupClient() to move_group_interface (`#1215 <https://github.com/ros-planning/moveit/issues/1215>`_)
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* [maintenance] Remove deprecated MoveGroup class (`#1211 <https://github.com/ros-planning/moveit/issues/1211>`_)
* [maintenance] Use C++14 (`#1146 <https://github.com/ros-planning/moveit/issues/1146>`_)
* [maintenance] Code Cleanup
  * `#1179 <https://github.com/ros-planning/moveit/issues/1179>`_
  * `#1196 <https://github.com/ros-planning/moveit/issues/1196>`_
* Contributors: Alex Moriarty, Dave Coleman, Martin Günther, Michael Görner, Robert Haschke

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------
* [capability] Get available planning group names from MoveGroup C++ (`#1159 <https://github.com/ros-planning/moveit/issues/1159>`_)
* Contributors: Dave Coleman

0.10.2 (2018-10-24)
-------------------
* [capability] Added plan_only flags to pick and place (`#862 <https://github.com/ros-planning/moveit/issues/862>`_)
* [maintenance] Python3 support (`#1103 <https://github.com/ros-planning/moveit/issues/1103>`_, `#1054 <https://github.com/ros-planning/moveit/issues/1054>`_)
* [fix] optional namespace args (`#929 <https://github.com/ros-planning/moveit/issues/929>`_)
* Contributors: David Watkins, Michael Görner, Mohmmad Ayman, Robert Haschke, mike lautman

0.10.1 (2018-05-25)
-------------------
* [maintenance] Remove deprecated ExecuteTrajectoryServiceCapability (`#833 <https://github.com/ros-planning/moveit/issues/833>`_)
* [maintenance] migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* [maintenance] switch to ROS_LOGGER from CONSOLE_BRIDGE (`#874 <https://github.com/ros-planning/moveit/issues/874>`_)
* [capability] namespace to moveit_commander (`#835 <https://github.com/ros-planning/moveit/issues/835>`_)
* Constrained Cartesian planning using moveit commander (`#805 <https://github.com/ros-planning/moveit/issues/805>`_)
* Simplify adding CollisionObjects with colors (`#810 <https://github.com/ros-planning/moveit/issues/810>`_)
* support TrajectoryConstraints in MoveGroupInterface + MoveitCommander (`#793 <https://github.com/ros-planning/moveit/issues/793>`_)
* Add API to get planner_id (`#788 <https://github.com/ros-planning/moveit/issues/788>`_)
* Allow wait time to be specified for getCurrentState() (`#685 <https://github.com/ros-planning/moveit/issues/685>`_)
* Contributors: 2scholz, Akiyoshi Ochiai, Bence Magyar, Dave Coleman, Ian McMahon, Robert Haschke, Will Baker, Xiaojian Ma, srsidd

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] MoveGroupInterface: Fixed computeCartesianPath to use selected end-effector. (`#580 <https://github.com/ros-planning/moveit/issues/580>`_)
* [capability][kinetic onward] Adapt pick pipeline to function without object (`#599 <https://github.com/ros-planning/moveit/issues/599>`_)
* [improve] Disabled copy constructors and added a move constructor to MoveGroupInterface (`#664 <https://github.com/ros-planning/moveit/issues/664>`_)
* Contributors: 2scholz, Dennis Hartmann, Jonathan Meyer, Simon Schmeisser

0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* [improve] MoveGroupInterface: add public interface to construct the MotionPlanRequest (`#461 <https://github.com/ros-planning/moveit/issues/461>`_)
* Contributors: Michael Goerner

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* [enhancement][MoveGroup] Add getLinkNames function (`#440 <https://github.com/ros-planning/moveit/issues/440>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* [fix] move_group.cpp: seg fault bug (`#426 <https://github.com/ros-planning/moveit/issues/426>`_)
* [fix] mgi: show correct include path in doxygen (`#419 <https://github.com/ros-planning/moveit/issues/419>`_)
* [fix] fix race conditions when updating PlanningScene (`#350 <https://github.com/ros-planning/moveit/issues/350>`_)
* [fix] issue `#373 <https://github.com/ros-planning/moveit/issues/373>`_ for Kinetic (`#377 <https://github.com/ros-planning/moveit/issues/377>`_) (`#385 <https://github.com/ros-planning/moveit/issues/385>`_)
* [capability] PSI: add apply* functions that use ApplyPlanningScene.srv (`#381 <https://github.com/ros-planning/moveit/issues/381>`_)
* [maintenance] Fix test file issues (`#415 <https://github.com/ros-planning/moveit/pull/415>`_, `#412 <https://github.com/ros-planning/moveit/issues/412>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Bastian Gaspers, Dave Coleman, Isaac I.Y. Saito, Jorge Santos Simon, Michael Goerner, Robert Haschke

0.9.3 (2016-11-16)
------------------

0.6.6 (2016-06-08)
------------------
* replaced cmake_modules dependency with eigen
* [jade] eigen3 adjustment
* merge indigo-devel changes (PR `#633 <https://github.com/ros-planning/moveit_ros/issues/633>`_ trailing whitespace) into jade-devel
* Removed trailing whitespace from entire repository
* planning_interface::MoveGroup::get/setPlannerParams
* new method MoveGroup::getDefaultPlannerId(const std::string &group)
  ... to retrieve default planner config from param server
  moved corresponding code from rviz plugin to MoveGroup interface
  to facilitate re-use
* fixing conflicts, renaming variable
* Merge pull request `#589 <https://github.com/ros-planning/moveit_ros/issues/589>`_ from MichaelStevens/set_num_planning_attempts
  adding set_num_planning_attempts to python interface
* comments addressed
* Added python wrapper for setMaxVelocityScalingFactor
* saves robot name to db from moveit. also robot name accessible through robot interface python wrapper
* adding set_num_planning_attempts to python interface
* Merge pull request `#571 <https://github.com/ros-planning/moveit_ros/issues/571>`_ from ymollard/indigo-devel
  Added python wrapper for MoveGroup.asyncExecute()
* Added python wrapper for MoveGroup.asyncExecute()
* Add retime_trajectory to moveit python wrapper
* add getHandle to move_group_interface
* Updated documentation on move() to inform the user that an asynchronus spinner is required. Commonly new users don't do this and move() blocks permanently
* Contributors: Dave Coleman, Dave Hershberger, Isaac I.Y. Saito, Kei Okada, Michael Stevens, Robert Haschke, Sachin Chitta, Scott, Yoan Mollard, dg, ferherranz

0.6.5 (2015-01-24)
------------------
* update maintainers
* Add time factor support for iterative_time_parametrization
* Contributors: Michael Ferguson, kohlbrecher

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------
* include correct ``boost::*_ptr`` class for boost 1.57.
* Contributors: v4hn

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------

0.6.0 (2014-10-27)
------------------
* Add missing variants of place (PlaceLocation, place anywhere) for python interface
* Python wrapper for getEndEffectorTips()
* Contributors: Dave Coleman, Sachin Chitta, corot

0.5.19 (2014-06-23)
-------------------
* Add check for planning scene monitor connection, with 5 sec delay
* Contributors: Dave Coleman

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* added move_group python interface bindings to move group interface
  function:
  void setPathConstraints(const moveit_msgs::Constraint &constraint)
  in order to be able to set path constraints from python scripts
  directly and no need to use the DB.
* Use member NodeHandle in action clients.
  Currently services and topics are already using the member NodeHandle instance,
  but not the action clients.
  This is relevant for two reasons:
  - Consistency in the resulting ROS API namespace (everything in the same namespace).
  - Consistency in the spinning policy. All services, topics and actions will be spinned
  by the same NodeHandle, and whatever custom (or not) spinners and callback queues it
  has associated.
* adding error code returns to relevant functions
* Contributors: Adolfo Rodriguez Tsouroukdissian, Emili Boronat, Ioan A Sucan, Sachin Chitta

0.5.16 (2014-02-27)
-------------------
* adding node handle to options in move_group_interface
* adding get for active joints
* Contributors: Sachin Chitta

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------
* add API for setting the number of motion plans to be evaluated via the MoveGroupInterface
* move_group_interface: improve documentation
* Contributors: Acorn Pooley, Ioan Sucan

0.5.12 (2014-01-03)
-------------------

0.5.11 (2014-01-03)
-------------------
* Fixed bug in computeCartesianPathPython.
* Adding collision object interface to planning_scene interface.
* Contributors: Acorn Pooley, Sachin Chitta

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* Fixed doxygen function-grouping.
* Added planning feedback to gui, refactored states tab

0.5.8 (2013-10-11)
------------------
* add function to start state monitor in move_group_interface::MoveGroup

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------
* update planning options

0.5.5 (2013-09-23)
------------------
* add support for setting joint targets from approximate IK
* specifies python version 2.7 for linking (fixes `#302 <https://github.com/ros-planning/moveit_ros/issues/302>`_)
* use new messages for pick & place
* expand functionality of MoveGroupInterface
* porting to new RobotState API

0.5.4 (2013-08-14)
------------------

* make pick more general
* use message serialization for python bindings
* remove CollisionMap, expose topic names in PlanningSceneMonitor, implement detach / attach operations as requested by `#280 <https://github.com/ros-planning/moveit_ros/issues/280>`_
* make headers and author definitions aligned the same way; white space fixes

0.5.2 (2013-07-15)
------------------
* move msgs to common_msgs

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* white space fixes (tabs are now spaces)

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
* some refactoring
