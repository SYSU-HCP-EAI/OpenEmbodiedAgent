^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_commander
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.11 (2022-12-21)
-------------------
* Fix some consistency issues in PlanningScene handling (`#3298 <https://github.com/ros-planning/moveit/issues/3298>`_)

  * Allow Plane collision-object creation from rviz
  * Allow to create cone collision-object from python PSI
  * rviz: Visualize PLANE shapes as a large, thin box
* Merge fixes+improvements to ``PlanningScene`` editing in rviz: `#3263 <https://github.com/ros-planning/moveit/issues/3263>`_, `#3264 <https://github.com/ros-planning/moveit/issues/3264>`_, `#3296 <https://github.com/ros-planning/moveit/issues/3296>`_
* ``moveit_commander.PlanningSceneInterface``: use synchronous interface by default
* Contributors: Robert Haschke

1.1.10 (2022-09-13)
-------------------
* Limit Cartesian speed for link(s) (`#2856 <https://github.com/ros-planning/moveit/issues/2856>`_)
* Validate JointState argument to `moveit_commander.set_joint_value_target` (`#3187 <https://github.com/ros-planning/moveit/issues/3187>`_)
* Adjust python PSI to new CollisionObject.pose interface (`#3176 <https://github.com/ros-planning/moveit/issues/3176>`_)
* moveit_commander: make current joint state copy-paste-able (`#3133 <https://github.com/ros-planning/moveit/issues/3133>`_)
* Replace obsolete distutils.core with setuptools (`#3103 <https://github.com/ros-planning/moveit/issues/3103>`_)
* Contributors: Filip Sund, Michael Görner, Stephanie Eng

1.1.9 (2022-03-06)
------------------
* Fix move_group_commander.go(tuple(...)) (`#3066 <https://github.com/ros-planning/moveit/issues/3066>`_)
* Contributors: Hongzhuo Liang, Michael Görner

1.1.8 (2022-01-30)
------------------

1.1.7 (2021-12-31)
------------------

1.1.6 (2021-11-06)
------------------
* Use relative imports (`#2912 <https://github.com/ros-planning/moveit/issues/2912>`_)
* Fix trajectory constraints (`#2429 <https://github.com/ros-planning/moveit/issues/2429>`_)
* Fix ``get_planning_pipeline_id`` in Python MGI (`#2753 <https://github.com/ros-planning/moveit/issues/2753>`_)
* Contributors: Felix von Drigalski, Kevin Chang, Michael Görner, Robert Haschke

1.1.5 (2021-05-23)
------------------
* Allow selecting planning pipeline in MotionSequenceAction (`#2657 <https://github.com/ros-planning/moveit/issues/2657>`_)
* Contributors: Felix von Drigalski

1.1.4 (2021-05-12)
------------------

1.1.3 (2021-04-29)
------------------
* Add planning_pipeline_id setting to Python MGI (`#2622 <https://github.com/ros-planning/moveit/issues/2622>`_)
* Add clear() to Python PSI, allow empty call to remove_attached_object (`#2609 <https://github.com/ros-planning/moveit/issues/2609>`_)
* Contributors: Felix von Drigalski

1.1.2 (2021-04-08)
------------------
* Fix formatting errors
* Add utility functions to Python PSI: add/detach objects, apply_planning_scene() (`#2532 <https://github.com/ros-planning/moveit/issues/2532>`_)
* add get_active_joint_names (`#2533 <https://github.com/ros-planning/moveit/issues/2533>`_)
* Add get_current_state_bounded(), get_current_state(), enforce_bounds() to MoveGroupCommander (`#2356 <https://github.com/ros-planning/moveit/issues/2356>`_)
* Contributors: Felix von Drigalski, Peter Mitrano, Tyler Weaver

1.1.1 (2020-10-13)
------------------
* [feature] MGC: Improve exception messages (`#2318 <https://github.com/ros-planning/moveit/issues/2318>`_)
* [fix] ROS namespacing in moveit_commander's PSI (`#2347 <https://github.com/ros-planning/moveit/issues/2347>`_)
* [fix] python3 issues (`#2323 <https://github.com/ros-planning/moveit/issues/2323>`_)
* Contributors: Michael Görner, Peter Mitrano, Robert Haschke

1.1.0 (2020-09-04)
------------------
* [feature] Add missing variants of place from list of PlaceLocations and Poses in the python interface (`#2231 <https://github.com/ros-planning/moveit/issues/2231>`_)
* [fix]     Add default velocity/acceleration scaling factors (`#1890 <https://github.com/ros-planning/moveit/issues/1890>`_)
* [fix]     Handle the updated plan() function of MoveGroupCommander (`#1640 <https://github.com/ros-planning/moveit/issues/1640>`_)
* [fix]     Fix `failing tutorial <https://github.com/ros-planning/moveit_tutorials/issues/301>`_ (`#1459 <https://github.com/ros-planning/moveit/issues/1459>`_)
* [maint]   Update dependencies for python3 in noetic (`#2131 <https://github.com/ros-planning/moveit/issues/2131>`_)
* [maint]   Better align MoveGroupInterface.plan() with C++ MoveGroup::plan() (`#790 <https://github.com/ros-planning/moveit/issues/790>`_)
* Contributors: Bence Magyar, Bjar Ne, Dave Coleman, Felix von Drigalski, Gerard Canal, Jafar Abdi, Masaki Murooka, Michael Ferguson, Michael Görner, Pavel-P, Raphael Druon, Robert Haschke, Ryodo Tanaka, Ryosuke Tajima, Sean Yen, v4hn

1.0.6 (2020-08-19)
------------------
* [maint]   Adapt repository for splitted moveit_resources layout (`#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [feature] Exposed parameter wait_for_servers and getPlannerId() API in MoveGroup's Python API (`#2201 <https://github.com/ros-planning/moveit/issues/2201>`_)
* Contributors: Gerard Canal, Robert Haschke, Michael Görner

1.0.5 (2020-07-08)
------------------
* [fix]   Python 3 fix (`#2030 <https://github.com/ros-planning/moveit/issues/2030>`_)
* Contributors: Henning Kayser, Michael Görner, Robert Haschke

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [feature] Expose reference_point_position parameter in getJacobian() (`#1595 <https://github.com/ros-planning/moveit/issues/1595>`_)
* [maint]   Improve Python 3 compatibility (`#1870 <https://github.com/ros-planning/moveit/issues/1870>`_)
  * Replaced StringIO with BytesIO for python msg serialization
  * Use py_bindings_tools::ByteString as byte-based serialization buffer on C++ side
* [fix]     Fix service call to utilize original name space (`#1959 <https://github.com/ros-planning/moveit/issues/1959>`_)
* [maint]   Windows compatibility: fallback to using `pyreadline` (`#1635 <https://github.com/ros-planning/moveit/issues/1635>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [fix]     Fix planning scene interface not respecting custom namespace (`#1815 <https://github.com/ros-planning/moveit/issues/1815>`_)
* [maint]   moveit_commander: python3 import fixes (`#1786 <https://github.com/ros-planning/moveit/issues/1786>`_)
* [fix]     python planning_scene_interface: fix attaching objects (`#1624 <https://github.com/ros-planning/moveit/issues/1624>`_)
* [feature] Select time parametrization algorithm in retime_trajectory (`#1508 <https://github.com/ros-planning/moveit/issues/1508>`_)
* Contributors: Bjar Ne, Felix von Drigalski, Masaki Murooka, Pavel-P, Raphael Druon, Robert Haschke, Ryodo Tanaka, Sean Yen, v4hn

1.0.2 (2019-06-28)
------------------
* [feature]     Add get_jacobian_matrix to moveit_commander (`#1501 <https://github.com/ros-planning/moveit/issues/1501>`_)
* [maintanance] Cleanup Python PlanningSceneInterface (`#1405 <https://github.com/ros-planning/moveit/issues/1405>`_, `#789 <https://github.com/ros-planning/moveit/issues/789>`_)
* Contributors: Bence Magyar, Robert Haschke, Ryosuke Tajima

1.0.1 (2019-03-08)
------------------
* [capability] python PlanningSceneInterface.add_cylinder() (`#1372 <https://github.com/ros-planning/moveit/issues/1372>`_)
* Contributors: Robert Haschke

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* Contributors: Keerthana Subramanian Manivannan, Robert Haschke

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
* [capability] Added plan_only flags to pick and place (`#862 <https://github.com/ros-planning/moveit/issues/862>`_)
* [maintenance] Python3 support (`#1103 <https://github.com/ros-planning/moveit/issues/1103>`_, `#1054 <https://github.com/ros-planning/moveit/issues/1054>`_)
* Contributors: David Watkins, Michael Görner, d-walsh, mike lautman

0.10.1 (2018-05-25)
-------------------
* Get robot markers from state (`#836 <https://github.com/ros-planning/moveit/issues/836>`_)
* Add namespace capabilities to moveit_commander (`#835 <https://github.com/ros-planning/moveit/issues/835>`_)
* Constrained Cartesian planning using moveit commander (`#805 <https://github.com/ros-planning/moveit/issues/805>`_)
* Handle robot_description parameter in RobotCommander (`#782 <https://github.com/ros-planning/moveit/issues/782>`_)
* support TrajectoryConstraints in MoveGroupInterface + MoveitCommander (`#793 <https://github.com/ros-planning/moveit/issues/793>`_)
* API to get planner_id (`#788 <https://github.com/ros-planning/moveit/issues/788>`_)
* Contributors: Akiyoshi Ochiai, Bence Magyar, Bryce Willey, Dave Coleman, Michael Görner, Ryan Keating, Will Baker

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [fix] Bugs in moveit_commander/robot.py (`#621 <https://github.com/ros-planning/moveit/issues/621>`_)
* [fix] pyassimp regression workaround  (`#581 <https://github.com/ros-planning/moveit/issues/581>`_)
* Contributors: Kei Okada, Konstantin Selyunin

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
* [fix] Regression on Ubuntu Xenial; numpy.ndarray indices bug (from `#86 <https://github.com/ros-planning/moveit/issues/86>`_) (`#450 <https://github.com/ros-planning/moveit/issues/450>`_).
* [doc][moveit_commander] added description for set_start_state (`#447 <https://github.com/ros-planning/moveit/issues/447>`_)
* Contributors: Adam Allevato, Ravi Prakash Joshi

0.9.4 (2017-02-06)
------------------
* [fix] issue `#373 <https://github.com/ros-planning/moveit/issues/373>`_ for Kinetic (`#377 <https://github.com/ros-planning/moveit/issues/377>`_) (`#385 <https://github.com/ros-planning/moveit/issues/385>`_)
* [fix] typo in moveit_commander (`#376 <https://github.com/ros-planning/moveit/issues/376>`_)
* Contributors: Dave Coleman, Shingo Kitagawa

0.9.3 (2016-11-16)
------------------
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------

0.6.1 (2016-04-28)
------------------
* [feat] Add the possibility to choose description file `#43 <https://github.com/ros-planning/moveit_commander/issues/43>`_
* [improve] support pyassimp 3.2. Looks like they changed their import path. robot_description should not be hardcoded to allow changing the name of the description file. This is usefull when working with several robots that do not share the same description file. `#45 <https://github.com/ros-planning/moveit_commander/issues/45>`_
* [improve] add queue_size option in planning_scene_interface.py `#41 <https://github.com/ros-planning/moveit_commander/issues/41>`_
* Contributors: Dave Coleman, Isaac I.Y. Saito, Kei Okada, Michael Görner, buschbapti

0.6.0 (2016-01-30)
------------------
* Merge pull request #38  from 130s/doc/python_if
  [RobotCommander] Fill in in-code document where missing.
* [moveit_commander/robot.py] Code cleaning; semi-PEP8.
* Merge pull request #35  from MichaelStevens/set_num_planning_attempts
  adding set_num_planning_attempts to commander interface
* Merge pull request #30 from ymollard/indigo-devel
  Planning scene improvements +  added python wrapper for MoveGroup.asyncExecute()
* Added python wrapper for MoveGroup.asyncExecute()
* Allow to clean all objects in a row
* Allow to attash an existing object without recreating the whole CollisionObject
* Merge pull request #24  from ymollard/hydro-devel
  Allowed user to change the scale of a mesh
* Merge pull request #23  from HumaRobotics/hydro-devel
  Fixed arguments removal in python roscpp_initializer
* Merge pull request #26  from corot/hydro-devel
  Add missing variants of place (PlaceLocation, place anywhere)
* Added a way to change the size of a mesh when grasping
* Allowed user to change the scale of a mesh
* Fixed arguments removal in python roscpp_initializer
* Contributors: Dave Coleman, Ioan A Sucan, Isaac I.Y. Saito, Michael Stevens, Philippe Capdepuy, Yoan Mollard, corot

0.5.7 (2014-07-05)
------------------
* Merge pull request `#21 <https://github.com/ros-planning/moveit_commander/issues/21>` from pirobot/hydro-devel
  Added set_support_surface_name function to move_group.py
* Added set_support_surface_name function to move_group.py
* Contributors: Patrick Goebel, Sachin Chitta

0.5.6 (2014-03-24)
------------------
* Added the calls necessary to manage path constraints.
* fix joint and link acces on __getattr__  when trying to acces a joint and its paramaters throught
* Contributors: Acorn, Emili Boronat, Sachin Chitta

0.5.5 (2014-02-27)
------------------
* adding get for active joints
* Contributors: Acorn, Sachin Chitta

0.5.4 (2014-02-06)
------------------

* Install moveit_commander_cmdline.py into package specific directory, not to global bin.
* Fix typos in comments

0.5.3 (2014-01-03)
------------------
* work around name bug
  move group interface python programs cannot be launched from launch files if
  the __name:= argument is used.  This works around the problem and allows using
  launch files to launch python moveit programs.
* Added Travis Continuous Integration

0.5.2 (2013-09-23)
------------------
* add support for setting joint targets from approximate IK
* no longer depend on manipulation_msgs
* expand functionality of MoveGroupInterface

0.5.1 (2013-08-13)
------------------
* make pick() more general
* use msg serialization
* use new attach / detach operations
* fix header for demo code
* Duration class bug fixed in commander conversion.

0.5.0 (2013-07-18)
------------------
* move msgs to common_msgs
* fixed ground command
