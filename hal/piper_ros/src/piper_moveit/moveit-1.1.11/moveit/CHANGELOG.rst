^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* [feature][visualization] Clean up Rviz Motion Planning plugin, add tooltips (`#2310 <https://github.com/ros-planning/moveit/issues/2310>`_)
* [feature][moveit_servo] A library for servoing toward a moving pose (`#2203 <https://github.com/ros-planning/moveit/issues/2203>`_)
* [feature][moveit_setup_assistant] Allow showing both, visual and collision geometry (`#2352 <https://github.com/ros-planning/moveit/issues/2352>`_)
* [fix][moveit_setup_assistant] layout (`#2349 <https://github.com/ros-planning/moveit/issues/2349>`_)
* [fix][moveit_setup_assistant] group editing (`#2350 <https://github.com/ros-planning/moveit/issues/2350>`_)
* [fix][moveit_setup_assistant] disappearing robot on change of reference frame (`#2335 <https://github.com/ros-planning/moveit/issues/2335>`_)
* Contributors: Felix von Drigalski, Michael Görner, Robert Haschke, Tyler Weaver, Yoan Mollard

1.1.0 (2020-09-04)
------------------
* [maint] Use standard cmake text for metapackages (`#1620 <https://github.com/ros-planning/moveit/issues/1620>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 for portability (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Dave Coleman, Jonathan Binney, Robert Haschke, Sean Yen

1.0.1 (2019-03-08)
------------------
* [fix] segfault in chomp adapter (`#1377 <https://github.com/ros-planning/moveit/issues/1377>`_)
* [capability] Graphically print current robot joint states with joint limits (`#1358 <https://github.com/ros-planning/moveit/issues/1358>`_)
* [capability] python PlanningSceneInterface.add_cylinder() (`#1372 <https://github.com/ros-planning/moveit/issues/1372>`_)
* [capability] Add time-optimal trajectory parameterization https://github.com/ros-planning/moveit/pull/1365
* [capability] FCL as a plugin  https://github.com/ros-planning/moveit/pull/1370
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Dave Coleman, Robert Haschke, Yu, Yan

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* [fix][moveit_setup_assistant] memory leaks (`#1292 <https://github.com/ros-planning/moveit/issues/1292>`_)
* [fix][moveit_core] invert waypoint velocities on reverse (`#1335 <https://github.com/ros-planning/moveit/issues/1335>`_)
* [fix][moveit_core]  Added missing robot state update to iterative spline parameterization to prevent warnings. (`#1298 <https://github.com/ros-planning/moveit/issues/1298>`_)
* [fix][moveit_core]  robot_model_test_utils depends on message generation (`#1286 <https://github.com/ros-planning/moveit/issues/1286>`_)
* [capability][moveit_kinematics] Adapt ikfast plugin to new KinematicsBase API. `#1320 <https://github.com/ros-planning/moveit/issues/1320>`_
* [improve] computeCartesianPath: limit joint-space jumps with IK consistency limits (`#1293 <https://github.com/ros-planning/moveit/issues/1293>`_)
* [improve] cleanup LMA kinematics solver `#1318 <https://github.com/ros-planning/moveit/issues/1318>`_
* [improve] Remove (redundant) random seeding and #attempts from RobotState::setFromIK() as the IK solver perform random seeding themselves. `#1288 <https://github.com/ros-planning/moveit/issues/1288>`_
* [improve] Kinematics tests, kdl cleanup `#1272 <https://github.com/ros-planning/moveit/issues/1272>`_, `#1294 <https://github.com/ros-planning/moveit/issues/1294>`_
* [improve][moveit_core]  Make FCL shape cache thread-local (`#1316 <https://github.com/ros-planning/moveit/issues/1316>`_)
* [improve][moveit_kinematics] KDL IK solver improvements (`#1321 <https://github.com/ros-planning/moveit/issues/1321>`_)
* [improve][moveit_setup_assistant] support dark themes (`#1173 <https://github.com/ros-planning/moveit/issues/1173>`_)
* [improve][moveit_ros_robot_interaction] cleanup RobotInteraction (`#1287 <https://github.com/ros-planning/moveit/issues/1287>`_)
* [improve][moveit_ros_robot_interaction] limit IK timeout to 0.1s for a responsive interaction behaviour (`#1291 <https://github.com/ros-planning/moveit/issues/1291>`_)
* [maintenance] cleanup SimpleControllerManager https://github.com/ros-planning/moveit/pull/1352
* [maintenance][moveit_core]  Add coverage analysis for moveit_core (`#1133 <https://github.com/ros-planning/moveit/issues/1133>`_)
* Contributors: Alexander Gutenkunst, Dave Coleman, Jonathan Binney, Keerthana Subramanian Manivannan, Martin Oehler, Michael Görner, Mike Lautman, Robert Haschke, Simon Schmeisser

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------

0.10.5 (2018-11-01)
-------------------
* [fix] Build regression (`#1174 <https://github.com/ros-planning/moveit/issues/1174>`_)
* [doc] Update README for ROS Melodic (`#1171 <https://github.com/ros-planning/moveit/issues/1171>`_)
* Contributors: Chris Lalancette, Ian McMahon

0.10.4 (2018-10-29)
-------------------
* [fix] Build regression (`#1170 <https://github.com/ros-planning/moveit/issues/1170>`_)
* Contributors: Robert Haschke

0.10.3 (2018-10-29)
-------------------
* [fix] Build regression (`#1134 <https://github.com/ros-planning/moveit/issues/1134>`_)
* [fix] compiler warnings (`#1089 <https://github.com/ros-planning/moveit/issues/1089>`_)
* [capability] Get available planning group names from MoveGroup C++ (`#1159 <https://github.com/ros-planning/moveit/issues/1159>`_)
* [maintenance] Store more settings of rviz' PlanningFrame (`#1135 <https://github.com/ros-planning/moveit/issues/1135>`_)
* [code] cleanup, improvements (`#1107 <https://github.com/ros-planning/moveit/issues/1107>`_, `#1099 <https://github.com/ros-planning/moveit/issues/1099>`_, `#1108 <https://github.com/ros-planning/moveit/issues/1108>`_, `#1144 <https://github.com/ros-planning/moveit/issues/1144>`_, `#1099 <https://github.com/ros-planning/moveit/issues/1099>`_)
* Contributors: Alexander Gutenkunst, Dave Coleman, Robert Haschke, Simon Schmeisser

0.10.2 (2018-10-24)
-------------------
* [fix] Text refrences to MoveIt (`#1020 <https://github.com/ros-planning/moveit/issues/1020>`_)
* [fix] Eigen alignment issuses due to missing aligned allocation (`#1039 <https://github.com/ros-planning/moveit/issues/1039>`_)
* [fix][chomp] changelogs: migration from tf -> tf2 only accidentally became part of 0.9.12's changelog
* [fix] Chomp package handling issue `#1086 <https://github.com/ros-planning/moveit/issues/1086>`_ that was introduced in `ubi-agni/hotfix-#1012 <https://github.com/ubi-agni/hotfix-/issues/1012>`_
* [fix] PlanningSceneMonitor lock `#1033 <https://github.com/ros-planning/moveit/issues/1033>`_: Fix `#868 <https://github.com/ros-planning/moveit/issues/868>`_ (`#1057 <https://github.com/ros-planning/moveit/issues/1057>`_)
* [fix] optional namespace args (`#929 <https://github.com/ros-planning/moveit/issues/929>`_)
* [fix] CurrentStateMonitor update callback for floating joints to handle non-identity joint origins `#984 <https://github.com/ros-planning/moveit/issues/984>`_
* [fix] reset moveit_msgs::RobotState.is_diff to false (`#968 <https://github.com/ros-planning/moveit/issues/968>`_) This fixes a regression introduced in `#939 <https://github.com/ros-planning/moveit/issues/939>`_.
* [fix][chomp] needs to depend on cmake_modules. (`#976 <https://github.com/ros-planning/moveit/issues/976>`_)
* [fix][moveit_ros_visualization] build issue in boost/thread/mutex.hpp (`#1055 <https://github.com/ros-planning/moveit/issues/1055>`_)
* [fix][moveit_ros_perception] planning scene lock when octomap updates too quickly (`#920 <https://github.com/ros-planning/moveit/issues/920>`_)
* [fix][moveit_fake_controller_manager] latch initial pose published by fake_controller_manager (`#1092 <https://github.com/ros-planning/moveit/issues/1092>`_)
* [fix][moveit_setup_assistant] Some bugs (`#1022 <https://github.com/ros-planning/moveit/issues/1022>`_, `#1013 <https://github.com/ros-planning/moveit/issues/1013>`_)
* [fix] continous joint limits are always satisfied (`#729 <https://github.com/ros-planning/moveit/issues/729>`_)
* [capability] adaptions for OMPL 1.4 (`#903 <https://github.com/ros-planning/moveit/issues/903>`_)
* [capability][chomp] Failure recovery options for CHOMP by tweaking parameters (`#987 <https://github.com/ros-planning/moveit/issues/987>`_)
* [capability] New screen for automatically generating interfaces to low level controllers(`#951 <https://github.com/ros-planning/moveit/issues/951>`_, `#994 <https://github.com/ros-planning/moveit/issues/994>`_, `#908 <https://github.com/ros-planning/moveit/issues/908>`_)
* [capability][moveit_setup_assistant] Perception screen for using laser scanner point clouds. (`#969 <https://github.com/ros-planning/moveit/issues/969>`_)
* [enhancement][GUI][moveit_setup_assistant] Logo for MoveIt 2.0, cleanup appearance (`#1059 <https://github.com/ros-planning/moveit/issues/1059>`_)
* [enhancement][GUI][moveit_setup_assistant] added a setup assistant window icon (`#1028 <https://github.com/ros-planning/moveit/issues/1028>`_)
* [capability][chomp] Addition of CHOMP planning adapter for optimizing result of other planners (`#1012 <https://github.com/ros-planning/moveit/issues/1012>`_)
* [capability][chomp] Failure recovery options for CHOMP by tweaking parameters (`#987 <https://github.com/ros-planning/moveit/issues/987>`_)
* [capability][chomp] cleanup of unused parameters and code + addition of trajectory initialization methods (linear, cubic, quintic-spline) (`#960 <https://github.com/ros-planning/moveit/issues/960>`_)
* [capability][moveit_ros_planning] new dynamic-reconfigure parameter wait_for_trajectory_completion to disable waiting for convergence independently from start-state checking. (`#883 <https://github.com/ros-planning/moveit/issues/883>`_)
* [capability][moveit_ros_planning] Option for controller-specific duration parameters (`#785 <https://github.com/ros-planning/moveit/issues/785>`_)
* [capability] Added plan_only flags to pick and place (`#862 <https://github.com/ros-planning/moveit/issues/862>`_)
* [capability][moveit_kinematics] add IKP_Translation{X,Y,Z}AxisAngle4D to the cpp template, see https://github.com/ros-planning/moveit/issues/548#issuecomment-316298918
* [capability] Benchmarking with different Motion Planners (STOMP, CHOMP, OMPL) (`#992 <https://github.com/ros-planning/moveit/issues/992>`_)
* [enhancement][warehouse] added params for timeout + #retries (`#1008 <https://github.com/ros-planning/moveit/issues/1008>`_)
* [enhancement][moveit_ros_planning] do not wait for robot convergence, when trajectory_execution_manager finishes with status != SUCCEEDED (`#1011 <https://github.com/ros-planning/moveit/issues/1011>`_)
* [enhancement][moveit_ros_planning] allow execution of empty trajectories (`#940 <https://github.com/ros-planning/moveit/issues/940>`_)
* [enhancement][moveit_ros_planning] avoid warning spam: "Unable to update multi-DOF joint" (`#935 <https://github.com/ros-planning/moveit/issues/935>`_)
* [enhancement] Add info messages to pick and place routine (`#1004 <https://github.com/ros-planning/moveit/issues/1004>`_)
* [maintenance] Python3 support (`#1103 <https://github.com/ros-planning/moveit/issues/1103>`_, `#1054 <https://github.com/ros-planning/moveit/issues/1054>`_)
* [maintenance] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* [maintenance] add minimum required pluginlib version (`#927 <https://github.com/ros-planning/moveit/issues/927>`_)
* Contributors: 2scholz, Adrian Zwiener, Alexander Guten kunst, Andrey Troitskiy, Chris Lalancette, d-walsh, Dave Coleman, David Watkins, dcconner, dg-shadow, Felix von Drigalski, Isaac Saito, Jonathan Binney, Kei Okada, Martin Guenther, Michael Goerner, Mikael Arguedas, Mike Lautman, Mohmmad Ayman, Raghavender Sahdev, Ridhwan Luthra, Robert Haschke, Simon Schmeisser, Sohieb Abdelrahman, srsidd, Timon Engelke, Xaver Kroischke

0.10.1 (2018-05-25)
-------------------

0.9.11 (2017-12-25)
-------------------
* [fix][moveit_core] #723; attached bodies are not shown in trajectory visualization anymore `#724 <https://github.com/ros-planning/moveit/issues/724>`_
* [fix][moveit_core] Shortcomings in kinematics plugins `#714 <https://github.com/ros-planning/moveit/issues/714>`_
* Contributors: Henning Kayser, Michael Görner, Robert Haschke

0.9.10 (2017-12-09)
-------------------
* [fix][moveit_ros_planning] Avoid segfault when validating a multidof-only trajectory (`#691 <https://github.com/ros-planning/moveit/issues/691>`_). Fixes `#539 <https://github.com/ros-planning/moveit/issues/539>`_
* [fix][moveit_ros_planning] find and link against tinyxml where needed (`#569 <https://github.com/ros-planning/moveit/issues/569>`_)
* [fix][moveit_ros_visualization] don't crash on empty robot_description in RobotState plugin `#688 <https://github.com/ros-planning/moveit/issues/688>`_
* [fix][moveit_ros_visualization] RobotState rviz previewer: First message from e.g. latching publishers is not applied to robot state correctly (`#596 <https://github.com/ros-planning/moveit/issues/596>`_)
* [fix][moveit_ros_planning_interface] MoveGroupInterface: Fixed computeCartesianPath to use selected end-effector. (`#580 <https://github.com/ros-planning/moveit/issues/580>`_)
* [fix][moveit_ros_move_group] always return true in MoveGroupPlanService callback `#674 <https://github.com/ros-planning/moveit/pull/674>`_
* [fix][moveit_ros_benchmarks] benchmarks: always prefer local header over system installations `#630 <https://github.com/ros-planning/moveit/issues/630>`_
* [fix][moveit_setup_assistant][kinetic onward] msa: use qt4-compatible API for default font (`#682 <https://github.com/ros-planning/moveit/issues/682>`_)
* [fix][moveit_setup_assistant][kinetic onward] replace explicit use of Arial with default application font (`#668 <https://github.com/ros-planning/moveit/issues/668>`_)
* [fix][moveit_setup_assistant] add moveit_fake_controller_manager to run_depend of moveit_config_pkg_template/package.xml.template (`#613 <https://github.com/ros-planning/moveit/issues/613>`_)
* [fix][moveit_setup_assistant] find and link against tinyxml where needed (`#569 <https://github.com/ros-planning/moveit/issues/569>`_)
* [fix][moveit_kinematics][kinetic onward] Fix create_ikfast_moveit_plugin to comply with format 2 of the package.xml. Remove collada_urdf dependency `#666 <https://github.com/ros-planning/moveit/pull/666>`_
* [fix][moveit_kinematics] create_ikfast_moveit_plugin: fixed directory variable for templates that were moved to ikfast_kinematics_plugin `#620 <https://github.com/ros-planning/moveit/issues/620>`_
* [fix][moveit_experimental] remove explicit fcl depends `#632 <https://github.com/ros-planning/moveit/pull/632>`_
* [fix][moveit_core] Add missing logWarn argument (`#707 <https://github.com/ros-planning/moveit/issues/707>`_)
* [fix][moveit_core] IKConstraintSampler: Fixed transform from end-effector to ik chain tip. `#582 <https://github.com/ros-planning/moveit/issues/582>`_
* [fix][moveit_core] robotStateMsgToRobotState: is_diff==true => not empty `#589 <https://github.com/ros-planning/moveit/issues/589>`_
* [fix][moveit_commander] Bugs in moveit_commander/robot.py (`#621 <https://github.com/ros-planning/moveit/issues/621>`_)
* [fix][moveit_commander] pyassimp regression workaround  (`#581 <https://github.com/ros-planning/moveit/issues/581>`_)
* [capability][moveit_ros_planning] Multi DOF Trajectory only providing translation not velocity (`#555 <https://github.com/ros-planning/moveit/issues/555>`_)
* [capability][moveit_ros_planning_interface][kinetic onward] Adapt pick pipeline to function without object (`#599 <https://github.com/ros-planning/moveit/issues/599>`_)
* [capability][moveit_simple_controller_manager][kinetic onward] optionally wait for controllers indefinitely (`#695 <https://github.com/ros-planning/moveit/issues/695>`_)
* [capability] Multi DOF Trajectory only providing translation not velocity (`#555 <https://github.com/ros-planning/moveit/issues/555>`_)
* [capability] Adds parameter lookup function for kinematics plugins (`#701 <https://github.com/ros-planning/moveit/issues/701>`_)
* [improve][moveit_ros_planning_interface] Disabled copy constructors and added a move constructor to MoveGroupInterface (`#664 <https://github.com/ros-planning/moveit/issues/664>`_)
* [improve][moveit_ros_perception] removed deprecated pluginlib macro (`#677 <https://github.com/ros-planning/moveit/issues/677>`_)
* [improve][moveit_ros_move_group] adding swp's to gitignore and removing redundant capabilites from capability_names.h (`#704 <https://github.com/ros-planning/moveit/issues/704>`_)
* [improve][moveit_kinematics] IKFastTemplate: Expand solutions to full joint range in searchPositionIK `#598 <https://github.com/ros-planning/moveit/issues/598>`_
* [improve][moveit_kinematics] IKFastTemplate: searchPositionIK now returns collision-free solution which is nearest to seed state. (`#585 <https://github.com/ros-planning/moveit/issues/585>`_)
* [improve][moveit_core] Make operator bool() explicit `#696 <https://github.com/ros-planning/moveit/pull/696>`_
* [improve][moveit_core] Get msgs from Planning Scene `#663 <https://github.com/ros-planning/moveit/issues/663>`_
* [improve][moveit_core] moveit_core: export DEPENDS on LIBFCL `#632 <https://github.com/ros-planning/moveit/pull/632>`_
* [improve][moveit_core] RobotState: Changed multi-waypoint version of computeCartesianPath to test joint space jumps after all waypoints are generated. (`#576 <https://github.com/ros-planning/moveit/issues/576>`_)
* [improve][moveit_core] Better debug output for IK tip frames (`#603 <https://github.com/ros-planning/moveit/issues/603>`_)
* [improve][moveit_core] New debug console colors YELLOW PURPLE (`#604 <https://github.com/ros-planning/moveit/issues/604>`_)
* [maintenance][moveit_planners_ompl][kinetic onward] Remove OutputHandlerROS from ompl_interface (`#609 <https://github.com/ros-planning/moveit/issues/609>`_)
* [doc][moveit_ros_visualization] Document auto scale in Rviz plugin (`#602 <https://github.com/ros-planning/moveit/issues/602>`_)
* Contributors: axelschroth, 2scholz, Bence Magyar, Bruno Brito, Dave Coleman, Dennis Hartmann, fsuarez6, G.A. vd. Hoorn, Henning Kayser, Isaac I.Y. Saito, Jonathan Meyer, Jorge Nicho, Kei Okada, Konstantin Selyunin, Michael Goerner, Mikael Arguedas, Mike Lautman, Phil, Shingo Kitagawa, Simon Schmeisser, Simon Schmeisser, Sarah Elliott, Shingo Kitagawa, Troy Cordie, William Woodall

0.9.9 (2017-08-06)
------------------
* Fixation in the contained packages:

  * [fix][moveit_ros_planning] Change getCurrentExpectedTrajectory index so collision detection is still performed even if the path timing is not known (`#550 <https://github.com/ros-planning/moveit/issues/550>`_)
  * [fix][moveit_ros_planning] check plan size for plan length=0 `#535 <https://github.com/ros-planning/moveit/issues/535>`_
  * [fix][moveit_ros_planning] ros_error macro name (`#544 <https://github.com/ros-planning/moveit/issues/544>`_)
  * [fix][moveit_ros_visualization] RobotStateVisualization: clear before load to avoid segfault `#572 <https://github.com/ros-planning/moveit/pull/572>`_
  * [fix][setup_assistant] Fix for lunar (`#542 <https://github.com/ros-planning/moveit/issues/542>`_) (fix `#506 <https://github.com/ros-planning/moveit/issues/506>`_)
  * [fix][moveit_core] segfault due to missing string format parameter. (`#547 <https://github.com/ros-planning/moveit/issues/547>`_)
  * [fix][moveit_core] doc-comment for robot_state::computeAABB (`#516 <https://github.com/ros-planning/moveit/issues/516>`_)
* Improvement in the contained packages:

  * [improve][moveit_ros_planning] Chomp use PlanningScene (`#546 <https://github.com/ros-planning/moveit/issues/546>`_) to partially address `#305 <https://github.com/ros-planning/moveit/issues/305>`_
  * [improve][moveit_ros_control_interface] add backward compatibility patch for indigo (`#551 <https://github.com/ros-planning/moveit/issues/551>`_)
  * [improve][moveit_planners_ompl] Optional forced use of JointModelStateSpaceFactory (`#541 <https://github.com/ros-planning/moveit/issues/541>`_)
  * [improve][moveit_kinematics] Modify ikfast_template for getPositionIK single solution results (`#537 <https://github.com/ros-planning/moveit/issues/537>`_)
* Contributors: Cyrille Morin, henhenhen, Martin Pecka, Simon Schmeisser, Michael Goerner, Mikael Arguedas, nsnitish

0.9.8 (2017-06-21)
------------------
* [fix][moveit_ros_visualization] TrajectoryVisualization crash if no window_context exists (`#523 <https://github.com/ros-planning/moveit/issues/523>`_, `#525 <https://github.com/ros-planning/moveit/issues/525>`_)
* [fix][moveit_ros_visualization]  robot display: Don't reload robot model upon topic change (Fixes `#528 <https://github.com/ros-planning/moveit/issues/528>`_)
* [fix][moveit_ros_planning] Include callback of execution status if trajectory is invalid. (`#524 <https://github.com/ros-planning/moveit/issues/524>`_)
* [fix][simple_controller_manager] include order (`#529 <https://github.com/ros-planning/moveit/issues/529>`_)
* [enhance][moveit_ros_visualization]  rviz display: stop trajectory visualization on new plan. Fixes `#526 <https://github.com/ros-planning/moveit/issues/526>`_ (`#531 <https://github.com/ros-planning/moveit/issues/531>`_, `#510 <https://github.com/ros-planning/moveit/issues/510>`_).
* [enhance][moveit_setup_assistant] setup assistant: add use_gui param to demo.launch (`#532 <https://github.com/ros-planning/moveit/issues/532>`_)
* [build][moveit_kinematics] adjust cmake_minimum_required for add_compile_options (`#521 <https://github.com/ros-planning/moveit/issues/521>`_)
* [build][moveit_kinematics] ikfast_kinematics_plugin: Add c++11 compile option. This is required for Kinetic.
* [build][moveit_kinematics] ikfast_kinematics_plugin: Write XML files as UTF-8 (`#514 <https://github.com/ros-planning/moveit/issues/514>`_)
* [build][moveit_ros_visualization] add Qt-moc guards for boost 1.64 compatibility (`#534 <https://github.com/ros-planning/moveit/issues/534>`_)
* Contributors: dougsm, Martin Guenther, Michael Goerner, Isaac I.Y. Saito, Simon Schmeisser, Yannick Jonetzko, henhenhen

0.9.7 (2017-06-05)
------------------
* [fix][ikfast_kinematics_plugin][Kinetic+] Add c++11 compile option `#515 <https://github.com/ros-planning/moveit/pull/515>`_
* [fix][moveit_kinematics][Indigo] Eigen3 dependency (`#470 <https://github.com/ros-planning/moveit/issues/470>`_)
* [fix][moveit_ros] Build for Ubuntu YZ by adding BOOST_MATH_DISABLE_FLOAT128 (`#505 <https://github.com/ros-planning/moveit/issues/505>`_)
* [fix][moveit_core] checks for empty name arrays messages before parsing the robot state message data (`#499 <https://github.com/ros-planning/moveit/issues/499>`_)
* [capability][visualization] New panel with a slider to control the visualized trajectory (`#491 <https://github.com/ros-planning/moveit/issues/491>`_) (`#508 <https://github.com/ros-planning/moveit/issues/508>`_)
* [improve][MSA] Open a directory where setup_assistant.launch was started. (`#509 <https://github.com/ros-planning/moveit/issues/509>`_)
* Contributors: Jorge Nicho, Michael Goerner, Martin Guenther, YuehChuan, Dave Coleman, Isaac I.Y. Saito, Mikael Arguedas

0.9.6 (2017-04-12)
------------------
* [fix] warehouse services (`#474 <https://github.com/ros-planning/moveit/issues/474>`_)
* [fix][moveit_ros_visualization] RViz plugin some cosmetics and minor refactoring `#482 <https://github.com/ros-planning/moveit/issues/482>`_
* [fix][moveit_ros_visualization] rviz panel: Don't add object marker if the wrong tab is selected `#454 <https://github.com/ros-planning/moveit/pull/454>`_
* [fix][moveit_ros_robot_interaction] `catkin_make -DCMAKE_ENABLE_TESTING=0` failure (`#478 <https://github.com/ros-planning/moveit/issues/478>`_)
* [fix] gcc6 build error (`#471 <https://github.com/ros-planning/moveit/issues/471>`_, `#458 <https://github.com/ros-planning/moveit/issues/458>`_)
* [fix][moveit_ros_manipulation] Set planning frame correctly in evaluation of reachable and valid pose filter (`#476 <https://github.com/ros-planning/moveit/issues/476>`_)
* [fix] gcc6 build error (`#471 <https://github.com/ros-planning/moveit/issues/471>`_, `#458 <https://github.com/ros-planning/moveit/issues/458>`_)
* [fix] undefined symbol in planning_scene_monitor (`#463 <https://github.com/ros-planning/moveit/issues/463>`_)
* [fix][moveit_planners_ompl] Always update initial robot state to prevent dirty robot state error. `#448 <https://github.com/ros-planning/moveit/pull/448>`_
* [fix][moveit_core] PlanarJointModel::getVariableRandomPositionsNearBy (`#464 <https://github.com/ros-planning/moveit/issues/464>`_)
* [improve][moveit_ros_visualization] RobotState display [kinetic] (`#465 <https://github.com/ros-planning/moveit/issues/465>`_)
* [improve][moveit_ros_planning_interface] MoveGroupInterface: add public interface to construct the MotionPlanRequest (`#461 <https://github.com/ros-planning/moveit/issues/461>`_)
* [improve][moveit_ros_benchmarks] Add install rule for examples, statistics script
* [improve] Add warning if no IK solvers found (`#485 <https://github.com/ros-planning/moveit/issues/485>`_)
* Contributors: Ruben Burger, Dave Coleman, Yannick Jonetzko, Henning Kayser, Beatriz Leon, Bence Magyar, Jorge Nicho, Tamaki Nishino, Michael Goerner, Dmitry Rozhkov, Isaac I.Y. Saito


0.9.5 (2017-03-08)
------------------
* [fix] correct "simplify widget handling" `#452 <https://github.com/ros-planning/moveit/pull/452>`_ This reverts "simplify widget handling (`#442 <https://github.com/ros-planning/moveit/issues/442>`_)"
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [fix] Regression on Ubuntu Xenial; numpy.ndarray indices bug (from `#86 <https://github.com/ros-planning/moveit/issues/86>`_) (`#450 <https://github.com/ros-planning/moveit/issues/450>`_).
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* [enhancement][MoveGroup] Add getLinkNames function (`#440 <https://github.com/ros-planning/moveit/issues/440>`_)
* [doc][moveit_commander] added description for set_start_state (`#447 <https://github.com/ros-planning/moveit/issues/447>`_)
* Contributors: Adam Allevato, Dave Coleman, Bence Magyar, Dave Coleman, Isaac I.Y. Saito, Yannick Jonetzko, Ravi Prakash Joshi

0.9.4 (2017-02-06)
------------------

0.9.3 (2016-11-16)
------------------
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------

0.9.0 (2016-10-19)
------------------
* Initial release into ROS Kinetic
