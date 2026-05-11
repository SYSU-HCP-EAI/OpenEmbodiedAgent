^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.11 (2022-12-21)
-------------------
* Fix some consistency issues in PlanningScene handling (`#3298 <https://github.com/ros-planning/moveit/issues/3298>`_)

  * Allow Plane collision-object creation from rviz
  * Simplify Cone rendering
  * Visualize PLANE shapes as a large, thin box
* Merge fixes+improvements to ``PlanningScene`` editing in rviz: `#3263 <https://github.com/ros-planning/moveit/issues/3263>`_, `#3264 <https://github.com/ros-planning/moveit/issues/3264>`_, `#3296 <https://github.com/ros-planning/moveit/issues/3296>`_

  * Fix error "QBackingStore::endPaint() called with active painter"
  * Remove limitation to one-shape collision objects
  * Fix segfault on object scaling: only update a _valid_ scene marker
  * JointsWidget: Copy full RobotState on updates (attached collision objects were missing)
  * Factor out ``addCollisionObjectToList()`` from ``populateCollisionObjectsList()``
  * Simplify ``MotionPlanningFrame::addSceneObject``
  * Directly call ``populateCollisionObjectsList()`` if possible

    Calling was previously deferred into a main loop job, because most
    callers already held a PlanningScene lock, thus causing recursive locking and a deadlock.
    By simply passing the locked scene, these issues can be avoided.
    As a fallback, the PlanningScene lock is still acquired in the function.
  * ``updateQueryStates()`` after removal of attached objects
  * Clear scene objects: only clear locally. To publish changes, one should explicitly click the "Publish" button.
  * Scene Object List: allow extended selection mode
  * always update query states - even if they are disabled for visualization
  * only allow execution if start state is up-to-date
* Merge PR `#3227 <https://github.com/ros-planning/moveit/issues/3227>`_: Improve MotionPlanning plugin's JointsWidget
  * Add units to sliders in JointsWidget and to spinbox editor
  * Avoid need for extra click to operate joint slider
  * allow control of joints via keyboard
* Contributors: Robert Haschke

1.1.10 (2022-09-13)
-------------------
* Fix rviz segfault when changing move group during execution (`#3123 <https://github.com/ros-planning/moveit/issues/3123>`_)
* Replace bind() with lambdas (`#3106 <https://github.com/ros-planning/moveit/issues/3106>`_)
* Replace obsolete distutils.core with setuptools (`#3103 <https://github.com/ros-planning/moveit/issues/3103>`_)
* Contributors: Michael Görner, Robert Haschke, bsygo

1.1.9 (2022-03-06)
------------------
* Add PS3 dual shock model to moveit joy (`#3025 <https://github.com/ros-planning/moveit/issues/3025>`_)
* Add option to use simulation time for rviz trajectory display (`#3055 <https://github.com/ros-planning/moveit/issues/3055>`_)
* Contributors: Job van Dieten, Martin Oehler

1.1.8 (2022-01-30)
------------------

1.1.7 (2021-12-31)
------------------
* Move ``MoveItErrorCode`` class to ``moveit_core`` (`#3009 <https://github.com/ros-planning/moveit/issues/3009>`_)
* ``RobotState::attachBody``: Migrate to ``unique_ptr`` argument (`#3011 <https://github.com/ros-planning/moveit/issues/3011>`_)
* Fix "ClassLoader: SEVERE WARNING" on reset of MPD (`#2925 <https://github.com/ros-planning/moveit/issues/2925>`_)
* Switch to ``std::bind`` (`#2967 <https://github.com/ros-planning/moveit/issues/2967>`_)
* Various fixes to MotionPlanning display (`#2944 <https://github.com/ros-planning/moveit/issues/2944>`_)

  * Avoid flickering of the progress bar
  * Joints widget: avoid flickering of the nullspace slider
* Modernize: std::make_shared
* Contributors: Jafar Abdi, JafarAbdi, Jochen Sprickerhof, Robert Haschke, pvanlaar

1.1.6 (2021-11-06)
------------------
* Re-initialize params, subscribers, and topics when the ``MoveGroupNS`` has changed (`#2922 <https://github.com/ros-planning/moveit/issues/2922>`_)
* Use newly introduced cmake macro ``moveit_build_options()`` from ``moveit_core``
* Do not save/restore warehouse parameters (`#2865 <https://github.com/ros-planning/moveit/issues/2865>`_) but use the ROS parameters only
* PSD: Correctly update robot's base pose (`#2876 <https://github.com/ros-planning/moveit/issues/2876>`_)
* Fix Python2: convert keys() into list (`#2862 <https://github.com/ros-planning/moveit/issues/2862>`_)
* MP panel: fix order of input widgets for shape size (`#2847 <https://github.com/ros-planning/moveit/issues/2847>`_)
* Use relative topic name in trajectory visualization to allow namespacing (`#2835 <https://github.com/ros-planning/moveit/issues/2835>`_)
* MotionPlanningFrame: Gracefully handle undefined parent widget, e.g. for use via ``librviz.so`` (`#2833 <https://github.com/ros-planning/moveit/issues/2833>`_)
* Introduce a reference frame for collision objects (`#2037 <https://github.com/ros-planning/moveit/issues/2037>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Support arbitrary real-time factors in trajectory visualization (`#2745 <https://github.com/ros-planning/moveit/issues/2745>`_)

  Replaced special value ``REALTIME`` to accept arbitrary real-time factors in the format ``<number>x``, e.g. ``3x``.
* Joints tab: Fix handling of mimic + passive joints (`#2744 <https://github.com/ros-planning/moveit/issues/2744>`_)
* Fix ``TrajectoryPanel``: Keep "Pause/Play" button in correct state (`#2737 <https://github.com/ros-planning/moveit/issues/2737>`_)
* Fixed error: ``moveit_joy: RuntimeError: dictionary changed size during iteration`` (`#2625 <https://github.com/ros-planning/moveit/issues/2625>`_, `#2628 <https://github.com/ros-planning/moveit/issues/2628>`_)
* Contributors: Felix von Drigalski, Michael Görner, Rick Staa, Robert Haschke, Yuri Rocha, lorepieri8, pvanlaar

1.1.5 (2021-05-23)
------------------

1.1.4 (2021-05-12)
------------------

1.1.3 (2021-04-29)
------------------
* Several minor fixups in PlanningSceneDisplay (`#2618 <https://github.com/ros-planning/moveit/issues/2618>`_)
* Contributors: Michael Görner, Robert Haschke

1.1.2 (2021-04-08)
------------------
* Fix various issues in PlanningScene / MotionPlanning displays (`#2588 <https://github.com/ros-planning/moveit/issues/2588>`_)
* Support multiple planning pipelines with MoveGroup via MoveItCpp (`#2127 <https://github.com/ros-planning/moveit/issues/2127>`_)
* Allow selecting planning pipeline in RViz MotionPlanningDisplay
* Catch exceptions during RobotModel loading in rviz (`#2468 <https://github.com/ros-planning/moveit/issues/2468>`_)
* Fix QObject::connect: Cannot queue arguments of type 'QVector<int>' (`#2392 <https://github.com/ros-planning/moveit/issues/2392>`_)
* Contributors: Henning Kayser, Michael Görner, Robert Haschke, Simon Schmeisser, Tyler Weaver

1.1.1 (2020-10-13)
------------------
* [feature] Clean up Rviz Motion Planning plugin, add tooltips (`#2310 <https://github.com/ros-planning/moveit/issues/2310>`_)
* [fix]     "Clear Octomap" button, disable when no octomap is published (`#2320 <https://github.com/ros-planning/moveit/issues/2320>`_)
* [fix]     clang-tidy warning (`#2334 <https://github.com/ros-planning/moveit/issues/2334>`_)
* [fix]     python3 issues (`#2323 <https://github.com/ros-planning/moveit/issues/2323>`_)
* [maint]   Cleanup MSA includes (`#2351 <https://github.com/ros-planning/moveit/issues/2351>`_)
* [maint]   Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski, Michael Görner, Robert Haschke

1.1.0 (2020-09-04)
------------------

1.0.6 (2020-08-19)
------------------
* [feature] MP display: add units to joints tab (`#2264 <https://github.com/ros-planning/moveit/issues/2264>`_)
* [feature] Allow adding planning scene shapes from rviz panel (`#2198 <https://github.com/ros-planning/moveit/issues/2198>`_)
* [feature] Default to Planning tab initially (`#2061 <https://github.com/ros-planning/moveit/issues/2061>`_)
* [fix]     Fix deferred robot model loading (`#2245 <https://github.com/ros-planning/moveit/issues/2245>`_)
* [maint]   Migrate to clang-format-10
* [maint]   Optimize includes (`#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* Contributors: Jorge Nicho, Markus Vieth, Michael Görner, Robert Haschke, Michael Görner

1.0.5 (2020-07-08)
------------------
* [feature] Improve rviz GUI to add PlanningScene objects. Ask for scaling large meshes. (`#2142 <https://github.com/ros-planning/moveit/issues/2142>`_)
* [maint]   Replace robot_model and robot_state namespaces with moveit::core (`#2135 <https://github.com/ros-planning/moveit/issues/2135>`_)
* [maint]   Fix catkin_lint issues (`#2120 <https://github.com/ros-planning/moveit/issues/2120>`_)
* [feature] PlanningSceneDisplay speedup (`#2049 <https://github.com/ros-planning/moveit/issues/2049>`_)
* [feature] Added support for PS4 joystick (`#2060 <https://github.com/ros-planning/moveit/issues/2060>`_)
* [fix]     MP display: planning attempts are natural numbers (`#2076 <https://github.com/ros-planning/moveit/issues/2076>`_, `#2082 <https://github.com/ros-planning/moveit/issues/2082>`_)
* Contributors: Felix von Drigalski, Henning Kayser, Jafar Abdi, Michael Görner, Robert Haschke, Simon Schmeisser, TrippleBender

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [fix]     `MotionPlanningDisplay`: change internal shortcut Ctrl+R to Ctrl+I (`#1967 <https://github.com/ros-planning/moveit/issues/1967>`_)
* [fix]     Remove `PlanningSceneInterface` from rviz display, but use its `PlanningSceneMonitor` instead
* [fix]     Fix segfault in `RobotStateVisualization` (`#1941 <https://github.com/ros-planning/moveit/issues/1941>`_)
* [feature] Provide visual feedback on success of requestPlanningSceneState()
* [feature] Wait for `get_planning_scene` in background (`#1934 <https://github.com/ros-planning/moveit/issues/1934>`_)
* [feature] Reduce step size for pose-adapting widgets
* [fix]     Reset `scene_marker` when disabling motion planning panel
* [fix]     Enable/disable motion planning panel with display
* [fix]     Enable/disable pose+scale group box when collision object is selected/deselected
* [fix]     Correctly populate the list of scene objects in the motion planning panel
* [feature] Resize scene marker with collision object
* [feature] Show attached bodies in trajectory trail (`#1766 <https://github.com/ros-planning/moveit/issues/1766>`_)
* [fix]     Fix `REALTIME` trajectory playback (`#1683 <https://github.com/ros-planning/moveit/issues/1683>`_)
* [maint]   Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint]   Notice changes in rviz planning panel requiring saving (`#1991 <https://github.com/ros-planning/moveit/issues/1991>`_)
* [maint]   Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint]   Improve Python 3 compatibility (`#1870 <https://github.com/ros-planning/moveit/issues/1870>`_)
  * Replaced StringIO with BytesIO for python msg serialization
  * Use py_bindings_tools::ByteString as byte-based serialization buffer on C++ side
* [maint]   Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [fix]     Fix pruning of enclosed nodes when rendering octomap in RViz (`#1685 <https://github.com/ros-planning/moveit/issues/1685>`_)
* [fix]     Fix missing `scene_manager` initialization in OcTreeRender's  constructor (`#1817 <https://github.com/ros-planning/moveit/issues/1817>`_)
* [feature] new `Joints` tab in RViz motion panel (`#1308 <https://github.com/ros-planning/moveit/issues/1308>`_)
* [feature] Add `<previous>` robot state to RViz motion panel (`#1742 <https://github.com/ros-planning/moveit/issues/1742>`_)
* Contributors: Bjar Ne, Dale Koenig, MarqRazz, Max Krichenbauer, Michael Görner, Robert Haschke, RyodoTanaka, Sean Yen, Takara Kasai, Yannick Jonetzko, Yu, Yan, v4hn

1.0.2 (2019-06-28)
------------------
* [maintenance] Removed unnecessary null pointer checks on deletion (`#1410 <https://github.com/ros-planning/moveit/issues/1410>`_)
* Contributors: Mahmoud Ahmed Selim

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Isaac Robert Haschke, Yu, Yan

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* Contributors: Dave Coleman, Robert Haschke

0.10.8 (2018-12-24)
-------------------
* [fix] Handle exceptions in rviz plugins (`#1267 <https://github.com/ros-planning/moveit/issues/1267>`_)
* Contributors: Christian Rauch, Robert Haschke

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [enhancement] Add check box for CartesianPath planning (`#1238 <https://github.com/ros-planning/moveit/issues/1238>`_)
* [enhancement] Improve MotionPlanning panel (`#1198 <https://github.com/ros-planning/moveit/issues/1198>`_)
  * Allow selection of planning group in planning panel
  * Choose start and goal state directly from combobox
* [fix] rviz crash when changing the planning group while executing (`#1198 <https://github.com/ros-planning/moveit/issues/1198>`_)
* [fix] Fix several issues in rendering of attached bodies (`#1199 <https://github.com/ros-planning/moveit/issues/1199>`_)
  * Show / hide attached body together with robot
  * Force PlanningScene rendering on enable
  * Link SceneDisplay's attached-body-color to TrajectoryVisualization's one
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* [maintenance] Use C++14 (`#1146 <https://github.com/ros-planning/moveit/issues/1146>`_)
* [maintenance] Cleanup Robot Interaction (`#1194 <https://github.com/ros-planning/moveit/issues/1194>`_)
  * Postpone subscription to trajectory topic
  * Fix memory leaks
* [maintenance] Simplify shared tf2 buffer usage (`#1196 <https://github.com/ros-planning/moveit/issues/1196>`_)
* [maintenance] Code Cleanup (`#1179 <https://github.com/ros-planning/moveit/issues/1179>`_)
* Remove obsolete eigen_conversions dependency (`#1181 <https://github.com/ros-planning/moveit/issues/1181>`_)
* Contributors: Alex Moriarty, Benjamin Scholz, Dave Coleman, Kei Okada, Michael Görner, Robert Haschke, Sven Krause

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------
* [maintenance] Store more settings of rviz' PlanningFrame (`#1135 <https://github.com/ros-planning/moveit/issues/1135>`_)
* [maintenance] Lint visualization (`#1144 <https://github.com/ros-planning/moveit/issues/1144>`_)
* Contributors: Alexander Gutenkunst, Dave Coleman

0.10.2 (2018-10-24)
-------------------
* [fix] build issue in boost/thread/mutex.hpp (`#1055 <https://github.com/ros-planning/moveit/issues/1055>`_)
* [fix] optional namespace args (`#929 <https://github.com/ros-planning/moveit/issues/929>`_)
* [maintenance] Python3 support (`#1103 <https://github.com/ros-planning/moveit/issues/1103>`_, `#1054 <https://github.com/ros-planning/moveit/issues/1054>`_)
* [maintenance] add minimum required pluginlib version (`#927 <https://github.com/ros-planning/moveit/issues/927>`_)
* Contributors: Michael Görner, Mikael Arguedas, Mohmmad Ayman, Robert Haschke, Timon Engelke, mike lautman

0.10.1 (2018-05-25)
-------------------
* [maintenance] migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* [feature] rviz plugin: set start/goal RobotState from external (`#823 <https://github.com/ros-planning/moveit/issues/823>`_)
  - /rviz/moveit/update_custom_start_state
  - /rviz/moveit/update_custom_goal_state
  stopping from external:
  - /rviz/moveit/stop
* [feature] namespace capabilities for moveit_commander (`#835 <https://github.com/ros-planning/moveit/issues/835>`_)
* [fix] consider shape transform for OcTree
* [fix] realtime trajectory display (`#761 <https://github.com/ros-planning/moveit/issues/761>`_)
* Contributors: Alexander Rössler, Dave Coleman, Ian McMahon, Mikael Arguedas, Pan Hy, Phy, Robert Haschke, Will Baker

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] don't crash on empty robot_description in RobotState plugin `#688 <https://github.com/ros-planning/moveit/issues/688>`_
* [fix] RobotState rviz previewer: First message from e.g. latching publishers is not applied to robot state correctly (`#596 <https://github.com/ros-planning/moveit/issues/596>`_)
* [doc] Document auto scale in Rviz plugin (`#602 <https://github.com/ros-planning/moveit/issues/602>`_)
* Contributors: Dave Coleman, Isaac I.Y. Saito, Simon Schmeisser, axelschroth

0.9.9 (2017-08-06)
------------------
* [fix] RobotStateVisualization: clear before load to avoid segfault `#572 <https://github.com/ros-planning/moveit/pull/572>`_
* Contributors: v4hn

0.9.8 (2017-06-21)
------------------
* [fix] TrajectoryVisualization crash if no window_context exists (`#523 <https://github.com/ros-planning/moveit/issues/523>`_, `#525 <https://github.com/ros-planning/moveit/issues/525>`_)
* [fix] robot display: Don't reload robot model upon topic change (Fixes `#528 <https://github.com/ros-planning/moveit/issues/528>`_)
* [build] add Qt-moc guards for boost 1.64 compatibility (`#534 <https://github.com/ros-planning/moveit/issues/534>`_)
* [enhance] rviz display: stop trajectory visualization on new plan. Fixes `#526 <https://github.com/ros-planning/moveit/issues/526>`_ (`#531 <https://github.com/ros-planning/moveit/issues/531>`_, `#510 <https://github.com/ros-planning/moveit/issues/510>`_).
* Contributors: Isaac I.Y. Saito, Simon Schmeisser, Yannick Jonetzko, henhenhen, v4hn


0.9.7 (2017-06-05)
------------------
* [capability] New panel with a slider to control the visualized trajectory (`#491 <https://github.com/ros-planning/moveit/issues/491>`_) (`#508 <https://github.com/ros-planning/moveit/issues/508>`_)
* [fix] Build for Ubuntu YZ by adding BOOST_MATH_DISABLE_FLOAT128 (`#505 <https://github.com/ros-planning/moveit/issues/505>`_)
* Contributors: Dave Coleman, Mikael Arguedas

0.9.6 (2017-04-12)
------------------
* [fix] RViz plugin some cosmetics and minor refactoring `#482 <https://github.com/ros-planning/moveit/issues/482>`_
* [fix] rviz panel: Don't add object marker if the wrong tab is selected `#454 <https://github.com/ros-planning/moveit/pull/454>`_
* [improve] RobotState display [kinetic] (`#465 <https://github.com/ros-planning/moveit/issues/465>`_)
* Contributors: Jorge Nicho, Michael Goerner, Yannick Jonetzko

0.9.5 (2017-03-08)
------------------
* [fix] correct "simplify widget handling" `#452 <https://github.com/ros-planning/moveit/pull/452>`_ This reverts "simplify widget handling (`#442 <https://github.com/ros-planning/moveit/issues/442>`_)"
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman, Isaac I.Y. Saito, Yannick Jonetzko

0.9.4 (2017-02-06)
------------------
* [fix] race conditions when updating PlanningScene (`#350 <https://github.com/ros-planning/moveit/issues/350>`_)
* [enhancement] Add colours to trajectory_visualisation display (`#362 <https://github.com/ros-planning/moveit/issues/362>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Bence Magyar, Dave Coleman, Robert Haschke

0.9.3 (2016-11-16)
------------------
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman

0.6.6 (2016-06-08)
------------------
* cleanup cmake tests, fix empty output
* added missing rostest dependency (`#680 <https://github.com/ros-planning/moveit_ros/issues/680>`_), fixes c6d0ede (`#639 <https://github.com/ros-planning/moveit_ros/issues/639>`_)
* [moveit joy] Add friendlier error message
* relax Qt-version requirement
  Minor Qt version updates are ABI-compatible with each other:
  https://wiki.qt.io/Qt-Version-Compatibility
* replaced cmake_modules dependency with eigen
* [jade] eigen3 adjustment
* always (re)create collision object marker
  other properties than pose (such as name of the marker) need to be adapted too
* use getModelFrame() as reference frame for markers
* moved "Publish Scene" button to "Scene Objects" tab
  previous location on "Context" tab was weird
* cherry-pick PR `#635 <https://github.com/ros-planning/moveit_ros/issues/635>`_ from indigo-devel
* unify Qt4 / Qt5 usage across cmake files
  - fetch Qt version from rviz
  - define variables/macros commonly used for Qt4 and Qt5
  - QT_LIBRARIES
  - qt_wrap_ui()
* leave frame transforms to rviz
  The old code
  (1.) reimplemented frame transforms in rviz
  although it could simply utilize rviz' FrameManager
  (2.) assumed the transform between the model-frame
  and the fixed_frame was constant and only needed to be updated
  if the frame changes (ever tried to make the endeffector
  your fixed frame?)
  (3.) was broken because on startup calculateOffsetPosition was called
  *before* the robot model is loaded, so the first (and usually only)
  call to calculateOffsetPosition failed.
  Disabling/Enabling the display could be used to work around this...
  This fixes all three issues.
* display planned path in correct rviz context
  This was likely a typo.
* Solved parse error with Boost 1.58. Fixes `#653 <https://github.com/ros-planning/moveit_ros/issues/653>`_
* Enable optional build against Qt5, use -DUseQt5=On to enable it
* explicitly link rviz' default_plugin library
  The library is not exported anymore and now is provided separately from rviz_LIBRARIES.
  See https://github.com/ros-visualization/rviz/pull/979 for details.
* merge indigo-devel changes (PR `#633 <https://github.com/ros-planning/moveit_ros/issues/633>`_ trailing whitespace) into jade-devel
* Removed trailing whitespace from entire repository
* correctly handle int and float parameters
  Try to parse parameter as int and float (in that series)
  and use IntProperty or FloatProperty on success to have
  input checking.
  Floats formatted without decimal dot, e.g. "0", will be
  considered as int!
  All other parameters will be handled as string.
* access planner params in rviz' MotionPlanningFrame
* new method MoveGroup::getDefaultPlannerId(const std::string &group)
  ... to retrieve default planner config from param server
  moved corresponding code from rviz plugin to MoveGroup interface
  to facilitate re-use
* correctly initialize scene robot's parameters after initialization
  - loaded parameters were ignored
  - changed default alpha value to 1 to maintain previous behaviour
* load default_planner_config from default location
  instead of loading from `/<ns>/default_planner_config`, use
  `/<ns>/move_group/<group>/default_planner_config`, which is the default
  location for `planner_configs` too
* Merge pull request `#610 <https://github.com/ros-planning/moveit_ros/issues/610>`_: correctly update all markers after robot motion
* fixing conflicts, renaming variable
* Merge pull request `#612 <https://github.com/ros-planning/moveit_ros/issues/612>`_ from ubi-agni/interrupt-traj-vis
  interrupt trajectory visualization on arrival of new display trajectory
* cherry-picked PR `#611 <https://github.com/ros-planning/moveit_ros/issues/611>`_: fix segfault when disabling and re-enabling TrajectoryVisualization
* cherry-picked PR `#609 <https://github.com/ros-planning/moveit_ros/issues/609>`_: load / save rviz' workspace config
* added missing initialization
* correctly setAlpha for new trail
* fixed race condition for trajectory-display interruption
* cleanup TrajectoryVisualization::update
  simplified code to switch to new trajectory / start over animation in loop mode
* new GUI property to allow immediate interruption of displayed trajectory
* immediately show trajectory after planning (interrupting current display)
* fix segfault when disabling and re-enabling TrajectoryVisualization
* update pose of all markers when any marker moved
  Having several end-effector markers attached to a group (e.g. a multi-
  fingered hand having an end-effector per fingertip and an end-effector
  for the hand base), all markers need to update their pose on any motion
  of any marker. In the example: if the hand base is moved, the fingertip
  markers should be moved too.
* use move_group/default_workspace_bounds as a fallback for workspace bounds
* code style cleanup
* fixed tab order of rviz plugin widgets
* load / save rviz' workspace config
* saves robot name to db from moveit. also robot name accessible through robot interface python wrapper
* Added install rule to install moveit_joy.py.
* motion_planning_frame_planning: use /default_planner_config parma to specify default planning algorithm
* Avoid adding a slash if getMoveGroupNS() is empty.
  If the getMoveGroupNS() returns an empty string, ros::names::append() inserts a slash in front of 'right', which changes it to a global name.
  Checking getMoveGroupNS() before calling append removes the issue.
  append() behaviour will not be changed in ros/ros_comm.
* Contributors: Ammar Najjar, Dave Coleman, Isaac I.Y. Saito, Jochen Welle, Kei Okada, Michael Ferguson, Michael Görner, Robert Haschke, Sachin Chitta, Simon Schmeisser (isys vision), TheDash, Thomas Burghout, dg, v4hn

0.6.5 (2015-01-24)
------------------
* update maintainers
* Created new trajectory display, split from motion planning display
* Added new trajectory display inside of motion planning display
* Fix bug with alpha property in trajectory robot
* Optimized number of URDFs loaded
* Changed motion planning Rviz icon to MoveIt icon
* Add time factor support for iterative_time_parametrization
* Contributors: Dave Coleman, Michael Ferguson, kohlbrecher

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------
* fix duplicate planning attempt box, also fix warning about name
* Contributors: Michael Ferguson

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------
* Fixed joystick documentation
* Joystick documentation and queue_size addition
* Contributors: Dave Coleman

0.6.0 (2014-10-27)
------------------
* Added move_group capability for clearing octomap.
* Fix coding style according to the moveit style
* Better user output, kinematic solver error handling, disclaimer
* Remove sample launch file for joystick and update
  joystick python script.
  1) Use moveit-python binding to parse SRDF.
  2) Make the speed slower to control the marker from joystick.
  3) Change joystick button mapping to be suitable for the users.
* Update joystick documentation and rename the
  the launch file for joy stick program.
  Shorten the message the check box to toggle
  communication with joy stick script.
* add checkbox to toggle if moveit rviz plugin subscribes
  the topics to be used for communication to the external ros nodes.
  update moveit_joy.py to parse srdf to know planning_groups and the
  names of the end effectors and support multi-endeffector planning groups.
* motion_planning_rviz_plugin: add move_group namespace option
  This allows multiple motion_planning_rviz_plugin /
  planning_scene_rviz_plugin to be used in RViz and connect to
  differently-namespaced move_group nodes.
* moved planning_attempts down one row in gui to maintain gui width
* Added field next to planning_time for planning_attempts
  Now, ParallelPlanner terminates either due to timeout, or due to this many attempts.
  Note, that ParallelPlanner run's Dijkstra's on all the nodes of all the sucessful plans (hybridize==true).
* adding PoseStamped topic to move the interactive marker from other ros nodes
  such as joystick programs.
* motion_planning_rviz_plugin: add move_group namespace option
  This allows multiple motion_planning_rviz_plugin /
  planning_scene_rviz_plugin to be used in RViz and connect to
  differently-namespaced move_group nodes.
* Contributors: Chris Lewis, Dave Coleman, Dave Hershberger, Jonathan Bohren, Ryohei Ueda, Sachin Chitta

0.5.19 (2014-06-23)
-------------------
* Changed rviz plugin action server wait to non-simulated time
* Fix [-Wreorder] warning.
* Fix RobotState rviz plugin to not display when disabled
* Add check for planning scene monitor connection, with 5 sec delay
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman

0.5.18 (2014-03-23)
-------------------
* add pkg-config as dep
* find PkgConfig before using pkg_check_modules
  PC specific functions mustn't be used before including PkgConfig
* Contributors: Ioan Sucan, v4hn

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* update maintainer e-mail
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------
* back out problematic ogre fixes
* robot_interaction: split InteractionHandler into its own file
* Switched from isStateColliding to isStateValid
* Changed per PR review
* Clean up debug output
* Added ability to set a random <collision free> start/goal position
* Merge branch 'hydro-devel' of https://github.com/ros-planning/moveit_ros into acorn_rviz_stereo
* rviz: prepare for Ogre1.10
* Contributors: Acorn Pooley, Dave Coleman

0.5.14 (2014-02-06)
-------------------

0.5.13 (2014-02-06)
-------------------
* remove debug printfs
* planning_scene_display: use requestPlanningSceneState()
  Get current planning scene state when planning scene display is
  enabled and/or model is loaded.
* Fix Parse error at "BOOST_JOIN" error
  See: https://bugreports.qt-project.org/browse/QTBUG-22829
* Contributors: Acorn Pooley, Benjamin Chretien

0.5.12 (2014-01-03)
-------------------

0.5.11 (2014-01-03)
-------------------
* Added back-link to tutorial and updated moveit website URL.
* Ported MoveIt RViz plugin tutorial to sphinx.
* Contributors: Dave Hershberger

0.5.10 (2013-12-08)
-------------------

0.5.9 (2013-12-03)
------------------
* correcting maintainer email
* Fixed an occasional crash bug in rviz plugin caused by gui calls in non-gui thread.
* Added planning feedback to gui, refactored states tab
* Stored states are auto loaded when warehouse database is connected

0.5.8 (2013-10-11)
------------------
* Added option to rviz plugin to show scene robot collision geometry

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------

0.5.5 (2013-09-23)
------------------
* Fix crash when the destructor is called before onInitialize
* remove call for getting the combined joint limits of a group
* bugfixes
* porting to new RobotState API
* use new helper class from rviz for rendering meshes

0.5.4 (2013-08-14)
------------------

* Added manipulation tab, added plan id to manipulation request
* make headers and author definitions aligned the same way; white space fixes
* using action client for object recognition instead of topic
* move background_processing lib to core
* display collision pairs instead of simply colliding links

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------

0.5.0 (2013-07-12)
------------------
* fix `#275 <https://github.com/ros-planning/moveit_ros/issues/275>`_
* white space fixes (tabs are now spaces)

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
* remove root_link_name property
* add status tab to Rviz plugin
