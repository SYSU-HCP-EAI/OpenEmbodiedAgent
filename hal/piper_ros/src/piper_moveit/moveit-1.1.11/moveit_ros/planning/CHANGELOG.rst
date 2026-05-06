^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.11 (2022-12-21)
-------------------
* Backport ruckig trajectory_processing plugin (`#2902 <https://github.com/ros-planning/moveit/issues/2902>`_)
* Allow planning with multiple pipelines in parallel with moveit_cpp (`#3244 <https://github.com/ros-planning/moveit/issues/3244>`_)
* Merge PR `#3262 <https://github.com/ros-planning/moveit/issues/3262>`_: Short-circuit planning adapters

  - Early return from failing planning adapters, namely FixStartStateCollision and FixStartStatePathConstraint
  - Propagate the error code via `MotionPlanResponse::error_code\_`
  - Add string translations for all error codes
* Cleanup translation of MoveItErrorCode to string

  - Move default code to moveit_core/utils
  - Override defaults in existing getActionResultString()
  - Provide translations for all error codes defined in moveit_msgs
* Short-circuit planning request adapters
* Contributors: Robert Haschke, Simon Schmeisser

1.1.10 (2022-09-13)
-------------------
* Limit Cartesian speed for link(s) (`#2856 <https://github.com/ros-planning/moveit/issues/2856>`_)
* trajectory execution manager: reactivate tests (`#3177 <https://github.com/ros-planning/moveit/issues/3177>`_)
* Clean up TrajectoryExecutionManager API (`#3178 <https://github.com/ros-planning/moveit/issues/3178>`_)
* MoveItCpp: Allow multiple pipelines (`#3131 <https://github.com/ros-planning/moveit/issues/3131>`_)
* Replace bind() with lambdas (`#3106 <https://github.com/ros-planning/moveit/issues/3106>`_)
* Contributors: Jochen Sprickerhof, Michael Görner, Robert Haschke, cambel, v4hn

1.1.9 (2022-03-06)
------------------

1.1.8 (2022-01-30)
------------------
* Fix deprecation warning in moveit_cpp (`#3019 <https://github.com/ros-planning/moveit/issues/3019>`_)
* Contributors: Jeroen

1.1.7 (2021-12-31)
------------------
* Move ``MoveItErrorCode`` class to ``moveit_core`` (`#3009 <https://github.com/ros-planning/moveit/issues/3009>`_)
* Switch to ``std::bind`` (`#2967 <https://github.com/ros-planning/moveit/issues/2967>`_)
* ``MoveitCpp``: Added ability to set path constraints for ``PlanningComponent`` (`#2959 <https://github.com/ros-planning/moveit/issues/2959>`_)
* ``RDFLoader``: clear buffer before reading content (`#2963 <https://github.com/ros-planning/moveit/issues/2963>`_)
* Reset markers on ``display_contacts`` topic for a new planning attempt (`#2944 <https://github.com/ros-planning/moveit/issues/2944>`_)
* Contributors: Colin Kohler, Jafar Abdi, Jochen Sprickerhof, Rick Staa, Robert Haschke

1.1.6 (2021-11-06)
------------------
* Use newly introduced cmake macro ``moveit_build_options()`` from ``moveit_core``
* Split ``CollisionPluginLoader`` (`#2834 <https://github.com/ros-planning/moveit/issues/2834>`_)

  To avoid circular dependencies, but enable reuse of the ``CollisionPluginLoader``, the non-ROS part was moved into ``moveit_core/moveit_collision_detection.so``
  and the ROS part (reading the plugin name from the parameter server) into ``moveit_ros_planning/moveit_collision_plugin_loader.so`` (as before).
* Introduce a reference frame for collision objects (`#2037 <https://github.com/ros-planning/moveit/issues/2037>`_)
* Fix RDFLoader: uninitialized member (`#2806 <https://github.com/ros-planning/moveit/issues/2806>`_)
* Add missing dependencies to generated dynamic_reconfigure headers (`#2772 <https://github.com/ros-planning/moveit/issues/2772>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* PSM: Read padding parameters from correct namespace (`#2706 <https://github.com/ros-planning/moveit/issues/2706>`_)
* Load ``max_safe_path_cost`` into namespace ``sense_for_plan`` (`#2703 <https://github.com/ros-planning/moveit/issues/2703>`_)
* Fix order of member initialization (`#2687 <https://github.com/ros-planning/moveit/issues/2687>`_)
* Move ``OccMapTree`` to ``moveit_core/collision_detection`` (`#2684 <https://github.com/ros-planning/moveit/issues/2684>`_)
* Contributors: Felix von Drigalski, Martin Günther, Mathias Lüdtke, Michael Görner, Robert Haschke, Simon Schmeisser, Tyler Weaver, pvanlaar, werner291

1.1.5 (2021-05-23)
------------------
* Revert "Lock the octomap/octree while collision checking (`#2683 <https://github.com/ros-planning/moveit/issues/2683>`_)
* Unify and simplify CSM::haveCompleteState overloads (`#2663 <https://github.com/ros-planning/moveit/issues/2663>`_)
* Contributors: Michael Görner

1.1.4 (2021-05-12)
------------------
* Lock the octomap/octree while collision checking (`#2596 <https://github.com/ros-planning/moveit/issues/2596>`_)
* Use private NodeHandle instead of child for PlanningPipeline topics (`#2652 <https://github.com/ros-planning/moveit/issues/2652>`_)
* Print an error if the planning pipelines are empty (`#2639 <https://github.com/ros-planning/moveit/issues/2639>`_)
* Simplify logic in PSM (`#2632 <https://github.com/ros-planning/moveit/issues/2632>`_)
* Contributors: Henning Kayser, Luc Bettaieb, Michael Görner, Simon Schmeisser, Rojas Rafael

1.1.3 (2021-04-29)
------------------

1.1.2 (2021-04-08)
------------------
* Fix formatting errors
* PlanExecution: Correctly handle preempt-requested flag (`#2554 <https://github.com/ros-planning/moveit/issues/2554>`_)
* Support multiple planning pipelines with MoveGroup via MoveItCpp (`#2127 <https://github.com/ros-planning/moveit/issues/2127>`_)
* Move moveit_cpp to moveit_ros_planning
  Deprecate namespace ``moveit::planning_interface::`` in favor of ``moveit_cpp::``
* thread safety in clear octomap & only update geometry (`#2500 <https://github.com/ros-planning/moveit/issues/2500>`_)
* Fix some typos in comments (`#2466 <https://github.com/ros-planning/moveit/issues/2466>`_)
* FixStartStateBounds: Copy attached bodies when adapting the start state (`#2398 <https://github.com/ros-planning/moveit/issues/2398>`_)
* Contributors: Henning Kayser, Michael Görner, Robert Haschke, Simon Schmeisser, Tyler Weaver, Udbhavbisarya23

1.1.1 (2020-10-13)
------------------
* [fix] some clang-tidy issues on Travis (`#2337 <https://github.com/ros-planning/moveit/issues/2337>`_)
* [fix] various issues with Noetic build (`#2327 <https://github.com/ros-planning/moveit/issues/2327>`_)
* [fix] "Clear Octomap" button, disable when no octomap is published (`#2320 <https://github.com/ros-planning/moveit/issues/2320>`_)
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski, Robert Haschke

1.1.0 (2020-09-04)
------------------
* [feature] Use Eigen::Transform::linear() instead of rotation() (`#1964 <https://github.com/ros-planning/moveit/issues/1964>`_)
* [feature] Bullet collision detection (`#1839 <https://github.com/ros-planning/moveit/issues/1839>`_) (`#1504 <https://github.com/ros-planning/moveit/issues/1504>`_)
* [feature] Allow different controllers for execution (`#1832 <https://github.com/ros-planning/moveit/issues/1832>`_)
* [feature] Adding continuous collision detection to Bullet (`#1551 <https://github.com/ros-planning/moveit/issues/1551>`_)
* [feature] plan_execution: refine logging for invalid paths (`#1705 <https://github.com/ros-planning/moveit/issues/1705>`_)
* [feature] Unified Collision Environment Integration (`#1584 <https://github.com/ros-planning/moveit/issues/1584>`_)
* [feature] Allow ROS namespaces for planning request adapters (`#1530 <https://github.com/ros-planning/moveit/issues/1530>`_)
* [feature] Add named frames to CollisionObjects (`#1439 <https://github.com/ros-planning/moveit/issues/1439>`_)
* [feature] get_planning_scene_service: return full scene when nothing was requested (`#1424 <https://github.com/ros-planning/moveit/issues/1424>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Initialize zero dynamics in CurrentStateMonitor (`#1883 <https://github.com/ros-planning/moveit/issues/1883>`_)
* [fix] memory leak (`#1526 <https://github.com/ros-planning/moveit/issues/1526>`_)
* [maint] Adapt repository for splitted moveit_resources layout (`#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint] partially transition unused test bin to rostest (`#2158 <https://github.com/ros-planning/moveit/issues/2158>`_)
* [maint] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_, `#2004 <https://github.com/ros-planning/moveit/issues/2004>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Fix usage of panda_moveit_config (`#1904 <https://github.com/ros-planning/moveit/issues/1904>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Adapt cmake for Bullet (`#1744 <https://github.com/ros-planning/moveit/issues/1744>`_)
* [maint] Readme for speed benchmark (`#1648 <https://github.com/ros-planning/moveit/issues/1648>`_)
* [maint] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] Improve variable naming in RobotModelLoader (`#1759 <https://github.com/ros-planning/moveit/issues/1759>`_)
* [maint] Move isEmpty() test functions to moveit_core/utils (`#1627 <https://github.com/ros-planning/moveit/issues/1627>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Ayush Garg, Bianca Homberg, Dave Coleman, Felix von Drigalski, Henning Kayser, Jens P, Jonathan Binney, Markus Vieth, Martin Pecka, Max Krichenbauer, Michael Görner, Robert Haschke, Sean Yen, Simon Schmeisser, Tyler Weaver, Yu, Yan, jschleicher, livanov93, llach

1.0.6 (2020-08-19)
------------------
* [fix]   Fix segfault in PSM::clearOctomap() (`#2193 <https://github.com/ros-planning/moveit/issues/2193>`_)
* [maint] Migrate to clang-format-10
* [maint] Optimize includes (`#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* Contributors: Henning Kayser, Markus Vieth, Robert Haschke

1.0.5 (2020-07-08)
------------------
* [feature] Trajectory Execution: fix check for start state position (`#2157 <https://github.com/ros-planning/moveit/issues/2157>`_)
* [feature] Improve responsiveness of PlanningSceneDisplay (`#2049 <https://github.com/ros-planning/moveit/issues/2049>`_)
  - PlanningSceneMonitor: increate update frequency from 10Hz to 30Hz
  - send RobotState diff if only position changed
* Contributors: Michael Görner, Robert Haschke, Simon Schmeisser

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [fix]     `CurrentStateMonitor`: Initialize velocity/effort with unset dynamics
* [fix]     Fix spurious warning message (# IK attempts) (`#1876 <https://github.com/ros-planning/moveit/issues/1876>`_)
* [maint]   Move `get_planning_scene` service into `PlanningSceneMonitor` for reusability (`#1854 <https://github.com/ros-planning/moveit/issues/1854>`_)
* [feature] Forward controller names to TrajectoryExecutionManager
* [fix]     Always copy dynamics if enabled in CurrentStateMonitor (`#1676 <https://github.com/ros-planning/moveit/issues/1676>`_)
* [feature] TrajectoryMonitor: zero sampling frequency disables trajectory recording (`#1542 <https://github.com/ros-planning/moveit/issues/1542>`_)
* [feature] Add user warning when planning fails with multiple constraints (`#1443 <https://github.com/ros-planning/moveit/issues/1443>`_)
* [maint]   Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint]   Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint]   Windows build fixes
  * Fix header inclusion and other MSVC build errors (`#1636 <https://github.com/ros-planning/moveit/issues/1636>`_)
  * Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
  * Favor ros::Duration.sleep over sleep. (`#1634 <https://github.com/ros-planning/moveit/issues/1634>`_)
  * Remove GCC extensions (`#1583 <https://github.com/ros-planning/moveit/issues/1583>`_)
  * Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [fix]     Fix potential memory leak in `RDFLoader` (`#1828 <https://github.com/ros-planning/moveit/issues/1828>`_)
  [maint]   Use smart pointers to avoid explicit new/delete
* [fix]     `TrajectoryExecutionManager`: fix race condition (`#1709 <https://github.com/ros-planning/moveit/issues/1709>`_)
* [fix]     Correctly propagate error if time parameterization fails (`#1562 <https://github.com/ros-planning/moveit/issues/1562>`_)
* [maint]   move `occupancy_map_monitor` into its own package (`#1533 <https://github.com/ros-planning/moveit/issues/1533>`_)
* [feature] `PlanExecution`: return executed trajectory (`#1538 <https://github.com/ros-planning/moveit/issues/1538>`_)
* Contributors: Felix von Drigalski, Henning Kayser, Max Krichenbauer, Michael Görner, Robert Haschke, Sean Yen, Yu, Yan, jschleicher, livanov93, Luca Lach

1.0.2 (2019-06-28)
------------------
* [fix] Removed MessageFilter for /collision_object messages (`#1406 <https://github.com/ros-planning/moveit/issues/1406>`_)
* Contributors: Robert Haschke

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Robert Haschke, Yu, Yan

1.0.0 (2019-02-24)
------------------
* [maintenance] Travis: enable warnings and catkin_lint checker (`#1332 <https://github.com/ros-planning/moveit/issues/1332>`_)
* [improve] Remove (redundant) random seeding and #attempts from RobotState::setFromIK() as the IK solver perform random seeding themselves. `#1288 <https://github.com/ros-planning/moveit/issues/1288>`_
* Contributors: Robert Haschke

0.10.8 (2018-12-24)
-------------------
* [maintenance] RDFLoader / RobotModelLoader: remove TinyXML API (`#1254 <https://github.com/ros-planning/moveit/issues/1254>`_)
* [enhancement] Cmdline tool to print planning scene info (`#1239 <https://github.com/ros-planning/moveit/issues/1239>`_)
* Contributors: Dave Coleman, Robert Haschke

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [fix] Fixed various memory leaks (`#1104 <https://github.com/ros-planning/moveit/issues/1104>`_)
  * KinematicsPluginLoader: only cache the latest instance
  * Use createUniqueInstance()
* [fix] Use correct trajectory_initialization_method parameter (`#1237 <https://github.com/ros-planning/moveit/issues/1237>`_)
* [enhancement] Pass RobotModel to IK, avoiding multiple loading (`#1166 <https://github.com/ros-planning/moveit/issues/1166>`_)
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* [maintenance] Use C++14 (`#1146 <https://github.com/ros-planning/moveit/issues/1146>`_)
* [maintenance] Code Cleanup
  * `#1179 <https://github.com/ros-planning/moveit/issues/1179>`_
  * `#1180 <https://github.com/ros-planning/moveit/issues/1180>`_
  * `#1196 <https://github.com/ros-planning/moveit/issues/1196>`_
* [maintenance] Change dynamic reconfigure limits for allowed_goal_duration_margin to 30s (`#993 <https://github.com/ros-planning/moveit/issues/993>`_)
* Contributors: Alex Moriarty, Dave Coleman, Hamal Marino, Michael Görner, Robert Haschke, Stephan

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------
* [fix] Build regression (`#1170 <https://github.com/ros-planning/moveit/issues/1170>`_)
* Contributors: Robert Haschke

0.10.3 (2018-10-29)
-------------------
* [fix] Build regression (`#1134 <https://github.com/ros-planning/moveit/issues/1134>`_)
* Contributors: Robert Haschke

0.10.2 (2018-10-24)
-------------------
* [fix] Chomp package handling issue `#1086 <https://github.com/ros-planning/moveit/issues/1086>`_ that was introduced in `ubi-agni/hotfix-#1012 <https://github.com/ubi-agni/hotfix-/issues/1012>`_
* [fix] PlanningSceneMonitor lock `#1033 <https://github.com/ros-planning/moveit/issues/1033>`_: Fix `#868 <https://github.com/ros-planning/moveit/issues/868>`_ (`#1057 <https://github.com/ros-planning/moveit/issues/1057>`_)
* [fix] CurrentStateMonitor update callback for floating joints to handle non-identity joint origins `#984 <https://github.com/ros-planning/moveit/issues/984>`_
* [fix] Eigen alignment issuses due to missing aligned allocation (`#1039 <https://github.com/ros-planning/moveit/issues/1039>`_)
* [fix] reset moveit_msgs::RobotState.is_diff to false (`#968 <https://github.com/ros-planning/moveit/issues/968>`_) This fixes a regression introduced in `#939 <https://github.com/ros-planning/moveit/issues/939>`_.
* [capability][chomp] Addition of CHOMP planning adapter for optimizing result of other planners (`#1012 <https://github.com/ros-planning/moveit/issues/1012>`_)
* [capability] new dynamic-reconfigure parameter wait_for_trajectory_completion to disable waiting for convergence independently from start-state checking. (`#883 <https://github.com/ros-planning/moveit/issues/883>`_)
* [capability] Option for controller-specific duration parameters (`#785 <https://github.com/ros-planning/moveit/issues/785>`_)
* [enhancement] do not wait for robot convergence, when trajectory_execution_manager finishes with status != SUCCEEDED (`#1011 <https://github.com/ros-planning/moveit/issues/1011>`_)
* [enhancement] allow execution of empty trajectories (`#940 <https://github.com/ros-planning/moveit/issues/940>`_)
* [enhancement] avoid warning spam: "Unable to update multi-DOF joint" (`#935 <https://github.com/ros-planning/moveit/issues/935>`_)
* Contributors: 2scholz, Adrian Zwiener, Kei Okada, Michael Görner, Mohmmad Ayman, Raghavender Sahdev, Robert Haschke, Simon Schmeisser, Xaver Kroischke, mike lautman, srsidd

0.10.1 (2018-05-25)
-------------------
* [fix] explicitly enforce updateSceneWithCurrentState() in waitForCurrentRobotState() (`#824 <https://github.com/ros-planning/moveit/issues/824>`_)
* Support static TFs for multi-DOF joints in CurrentStateMonitor (`#799 <https://github.com/ros-planning/moveit/issues/799>`_)
* support xacro args (`#796 <https://github.com/ros-planning/moveit/issues/796>`_)
* CSM: wait for *active* joint states only (`#792 <https://github.com/ros-planning/moveit/issues/792>`_)
* skip non-actuated joints for execution (`#754 <https://github.com/ros-planning/moveit/issues/754>`_)
* Iterative cubic spline interpolation (`#441 <https://github.com/ros-planning/moveit/issues/441>`_)
* Floating Joint Support in CurrentStateMonitor (`#748 <https://github.com/ros-planning/moveit/issues/748>`_)
* validate multi-dof trajectories before execution (`#713 <https://github.com/ros-planning/moveit/issues/713>`_)
* Contributors: Bruno Brito, Dave Coleman, Ian McMahon, Ken Anderson, Michael Görner, Mikael Arguedas, Robert Haschke

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] Avoid segfault when validating a multidof-only trajectory (`#691 <https://github.com/ros-planning/moveit/issues/691>`_). Fixes `#539 <https://github.com/ros-planning/moveit/issues/539>`_
* [fix] find and link against tinyxml where needed (`#569 <https://github.com/ros-planning/moveit/issues/569>`_)
* [capability] Multi DOF Trajectory only providing translation not velocity (`#555 <https://github.com/ros-planning/moveit/issues/555>`_)
* Contributors: Isaac I.Y. Saito, Michael Görner, Mikael Arguedas, Troy Cordie

0.9.9 (2017-08-06)
------------------
* [fix] Change getCurrentExpectedTrajectory index so collision detection is still performed even if the path timing is not known (`#550 <https://github.com/ros-planning/moveit/issues/550>`_)
* [fix] Support for MultiDoF only trajectories `#553 <https://github.com/ros-planning/moveit/pull/553>`_
* [fix] ros_error macro name (`#544 <https://github.com/ros-planning/moveit/issues/544>`_)
* [fix] check plan size for plan length=0 `#535 <https://github.com/ros-planning/moveit/issues/535>`_
* Contributors: Cyrille Morin, Michael Görner, Mikael Arguedas, Notou, Unknown

0.9.8 (2017-06-21)
------------------
* [fix] Include callback of execution status if trajectory is invalid. (`#524 <https://github.com/ros-planning/moveit/issues/524>`_)
* Contributors: dougsm

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* [fix] gcc6 build error (`#471 <https://github.com/ros-planning/moveit/issues/471>`_, `#458 <https://github.com/ros-planning/moveit/issues/458>`_)
* [fix] undefined symbol in planning_scene_monitor (`#463 <https://github.com/ros-planning/moveit/issues/463>`_)
* Contributors: Dmitry Rozhkov, Ruben Burger

0.9.5 (2017-03-08)
------------------
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar

0.9.4 (2017-02-06)
------------------
* [fix] race conditions when updating PlanningScene (`#350 <https://github.com/ros-planning/moveit/issues/350>`_)
* [maintenance] Use static_cast to cast to const. (`#433 <https://github.com/ros-planning/moveit/issues/433>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman, Maarten de Vries, Robert Haschke

0.9.3 (2016-11-16)
------------------
* [fix] cleanup urdfdom compatibility (`#319 <https://github.com/ros-planning/moveit/issues/319>`_)
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon, Robert Haschke

0.9.2 (2016-11-05)
------------------
* [Capability] compatibility to urdfdom < 0.4 (`#317 <https://github.com/ros-planning/moveit/issues/317>`_)
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman, Robert Haschke

0.6.6 (2016-06-08)
------------------
* Add library moveit_collision_plugin_loader as an exported catkin library (`#678 <https://github.com/ros-planning/moveit_ros/issues/678>`_)
* replaced cmake_modules dependency with eigen
* [jade] eigen3 adjustment
* Fix compilation with C++11.
* Enable optional build against Qt5, use -DUseQt5=On to enable it
* merge indigo-devel changes (PR `#633 <https://github.com/ros-planning/moveit_ros/issues/633>`_ trailing whitespace) into jade-devel
* Removed trailing whitespace from entire repository
* Optional ability to copy velocity and effort to RobotState
* cherry-picked PR `#614 <https://github.com/ros-planning/moveit_ros/issues/614>`_
  fixed segfault on shutdown
* fixed segfault on shutdown
  use of pluginlib's createUnmanagedInstance() is strongly discouraged:
  http://wiki.ros.org/class_loader#Understanding_Loading_and_Unloading
  here, the kinematics plugin libs were unloaded before destruction of corresponding pointers
* Deprecate shape_tools
* CurrentStateMonitor no longer requires hearing mimic joint state values.
* Fix crash due to robot state not getting updated (moveit_ros `#559 <https://github.com/ros-planning/moveit_ros/issues/559>`_)
* Contributors: Dave Coleman, Dave Hershberger, Isaac I.Y. Saito, Levi Armstrong, Maarten de Vries, Robert Haschke, Simon Schmeisser (isys vision), kohlbrecher

0.6.5 (2015-01-24)
------------------
* update maintainers
* perception: adding RAII-based locking for OccMapTree
* perception: adding locks to planning scene monitor
* Add time factor support for iterative_time_parametrization
* Contributors: Jonathan Bohren, Michael Ferguson, kohlbrecher

0.6.4 (2014-12-20)
------------------
* Namespaced "traj_execution" for all trajectory_execution_manager logging
* planning_scene_monitor: add ros parameter for adding a wait-for-transform lookup time
  fixes `#465 <https://github.com/ros-planning/moveit_ros/issues/465>`_
* Contributors: Dave Coleman, Jonathan Bohren

0.6.3 (2014-12-03)
------------------
* add plugin interface for collision detectors
* fix missing return value
* trivial fixes for warnings
* Contributors: Michael Ferguson

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------
* re-add libqt4 dependency (previously came from pcl-all)
* Contributors: Michael Ferguson

0.6.0 (2014-10-27)
------------------
* Removed leadings slash from rosparam for robot padding
* Added move_group capability for clearing octomap.
* Made loading octomap_monitor optional in planning_scene_monitor when using WorldGeometryMonitor
* Contributors: Dave Coleman, Dave Hershberger, Sachin Chitta, ahb

0.5.19 (2014-06-23)
-------------------
* Updated doxygen comment in TrajectoryExecutionManager.
* Added more informative error message text when cant' find controllers.
* robot_model_loader.cpp: added call to KinematicsBase::supportsGroup().
* Fix [-Wreorder] warning.
* Fix broken log & output statements.
  - Address [cppcheck: coutCerrMisusage] and [-Werror=format-extra-args] errors.
  - ROS_ERROR -> ROS_ERROR_NAMED.
  - Print size_t values portably.
* Address [-Wreturn-type] warning.
* Address [cppcheck: postfixOperator] warning.
* Address [cppcheck: duplicateIf] error.
  The same condition was being checked twice, and the same action was being taken.
* Add check for planning scene monitor connection, with 5 sec delay
* Fix for building srv_kinematics_plugin
* New ROS service call-based IK plugin
* Allow planning groups to have more than one tip
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Dave Hershberger

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* update maintainer e-mail
* Namespace a debug message
* Minor non-functional changes to KDL
* Contributors: Dave Coleman, Ioan Sucan

0.5.16 (2014-02-27)
-------------------
* Copy paste error fix
* Contributors: fivef

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------
* remove debug printfs
* planning_scene_monitor: add requestPlanningSceneState()
* planning_scene_monitor: fix race condition
* planning_scene_monitor: fix state update bug
  The rate of state updates is limited to dt_state_update per second.
  When an update arrived it was not processed if another was recently
  processed.  This meant that if a quick sequence of state updates
  arrived and then no updates arrive for a while that the last update(s)
  were not seen until another arrives (which may be much later or
  never). This fixes the bug by periodically checking for pending
  updates and running them if they have been pending longer than
  dt_state_update.
* add default_robot_link_padding/scale, set padding/scale value for each robot link, see https://github.com/ros-planning/moveit_ros/issues/402
* fix LockedPlanningSceneRW docs
  fix the text that was originally copied from another class
  (from LockedPlanningSceneRO)
  it mentioned an incorrect return value type of
  LockedPlanningSceneRW::operator->()
* Contributors: Acorn Pooley, Filip Jares, Kei Okada

0.5.12 (2014-01-03)
-------------------
* Fixed trailing underscores in CHANGELOGs.
* Contributors: Dave Hershberger

0.5.11 (2014-01-03)
-------------------
* planning_scene_monitor: slight code simplification
* planning_scene_monitor: fix scope of local vars
* planning_scene_monitor: fix init bug
  scene_const\_ not set if scene passed to constructor.
* kdl_kinematics_plugin: fix warning
* Contributors: Acorn Pooley

0.5.10 (2013-12-08)
-------------------
* fixing how joint names are filled up, fixed joints were getting included earlier, also resizing consistency limits for when random positions near by function is being called
* Contributors: Sachin Chitta

0.5.9 (2013-12-03)
------------------
* Doxygen: added warnings and details to planning_scene_monitor.h
* correcting maintainer email
* remove duplicate header
* Fixed exported targets
* Fixed dependency issue
* fixing joint limits setup for mimic joints
* implement feature requests
* clear monitored octomap when needed (see `#315 <https://github.com/ros-planning/moveit_ros/issues/315>`_)
* fix the adapter for fixing path constraints for initial states
* fixed computation of dimension\_.
* bugfixes in indexing added states for path adapters
* fixes for mimic joints and redundant joints

0.5.8 (2013-10-11)
------------------
* add executable for publishing scene geometry from text
* Made the goal duration margin and scaling optional rosparameters
* bugfixes

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* fix the event triggered on updating attached objects
* make scene monitor trigger updates only when needed
* fix loading of default params
* port to new RobotState API, new messages
* make sure we do not overwrite attached bodies, when updating the current state
* fix `#308 <https://github.com/ros-planning/moveit_ros/issues/308>`_
* fix `#304 <https://github.com/ros-planning/moveit_ros/issues/304>`_
* fix issue with sending trajectories for passive/mimic/fixed joints
* pass effort along

0.5.4 (2013-08-14)
------------------

* remove CollisionMap, expose topic names in PlanningSceneMonitor, implement detach / attach operations as requested by `#280 <https://github.com/ros-planning/moveit_ros/issues/280>`_
* make headers and author definitions aligned the same way; white space fixes
* move background_processing lib to core
* add option to disable trajectory monitoring

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* minor doc fixes
* add docs for planning pipeline
* cleanup build system
* fixing approximate ik calculation
* white space fixes (tabs are now spaces)
* adding check for approximate solution flag
* adding options struct to kinematics base
* port to new base class for planning_interface (using planning contexts)

0.4.5 (2013-07-03)
------------------
* Namespaced ROS_* log messages for better debug fitering - added 'kdl' namespace
* remove dep
* make searchPositionIK actually const, and thread-safe
* Made debug output look better

0.4.4 (2013-06-26)
------------------
* fix `#210 <https://github.com/ros-planning/moveit_ros/issues/210>`_
* added dynamic reconfigure parameters to allow enabling/disabling of trajectory duration monitoring. fixes `#256 <https://github.com/ros-planning/moveit_ros/issues/256>`_
* add state operations evaluation tool
* warn when time parametrization fails
* moved exceptions headers
