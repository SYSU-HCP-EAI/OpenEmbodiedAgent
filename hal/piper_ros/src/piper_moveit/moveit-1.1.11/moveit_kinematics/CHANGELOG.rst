^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.11 (2022-12-21)
-------------------
* kinematics: add ``program_options`` as required boost component (`#3269 <https://github.com/ros-planning/moveit/issues/3269>`_)
* Contributors: Jochen Sprickerhof

1.1.10 (2022-09-13)
-------------------
* Merge PR `#3172 <https://github.com/ros-planning/moveit/issues/3172>`_: Fix CI
* Fix test_ikfast_plugins.sh
* auto_create_ikfast_moveit_plugin.sh: allow xacro input
* Switch to hpp headers of pluginlib
* Replace bind() with lambdas (`#3106 <https://github.com/ros-planning/moveit/issues/3106>`_)
* Contributors: Jochen Sprickerhof, Michael Görner, Robert Haschke

1.1.9 (2022-03-06)
------------------

1.1.8 (2022-01-30)
------------------

1.1.7 (2021-12-31)
------------------
* ``round_collada_numbers.py``: python 2/3 compatibility (`#2983 <https://github.com/ros-planning/moveit/issues/2983>`_)
* Switch to ``std::bind`` (`#2967 <https://github.com/ros-planning/moveit/issues/2967>`_)
* Contributors: Jochen Sprickerhof, Tomislav Bazina

1.1.6 (2021-11-06)
------------------
* Use newly introduced cmake macro ``moveit_build_options()`` from ``moveit_core``
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: Robert Haschke, pvanlaar

1.1.5 (2021-05-23)
------------------

1.1.4 (2021-05-12)
------------------
* Fix ikfast script: install sympy 0.7.1 from git (`#2650 <https://github.com/ros-planning/moveit/issues/2650>`_)
* Contributors: ags-dy

1.1.3 (2021-04-29)
------------------

1.1.2 (2021-04-08)
------------------
* Fix formatting errors
* Replaced eigen+kdl conversions with tf2_eigen + tf2_kdl (`#2472 <https://github.com/ros-planning/moveit/issues/2472>`_)
* Python3 compatibility for ikfast's round_collada_numbers.py (`#2473 <https://github.com/ros-planning/moveit/issues/2473>`_)
* Contributors: Tobias Fischer, Tyler Weaver, petkovich

1.1.1 (2020-10-13)
------------------
* [fix] various issues with Noetic build (`#2327 <https://github.com/ros-planning/moveit/issues/2327>`_)
* [fix] python3 issues (`#2323 <https://github.com/ros-planning/moveit/issues/2323>`_)
* Contributors: G.A. vd. Hoorn, Michael Görner, Robert Haschke

1.1.0 (2020-09-04)
------------------
* [feature] Implementation of parameter TranslationXY2D IKFast (`#1949 <https://github.com/ros-planning/moveit/issues/1949>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Delete IKCache copy constructor (`#1750 <https://github.com/ros-planning/moveit/issues/1750>`_)
* [maint] Move NOLINT instructions to intended positions (`#2058 <https://github.com/ros-planning/moveit/issues/2058>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_) (`#2004 <https://github.com/ros-planning/moveit/issues/2004>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] Fix various build issues on Windows (`#1880 <https://github.com/ros-planning/moveit/issues/1880>`_)
* [maint] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 for portability (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* [maint] Relax dependencies of moveit_kinematics (`#1529 <https://github.com/ros-planning/moveit/issues/1529>`_)
* Contributors: Ayush Garg, Christian Henkel, Dave Coleman, Henning Kayser, Immanuel Martini, Jonathan Binney, Markus Vieth, Martin Günther, Michael Ferguson, Michael Görner, Robert Haschke, Sean Yen, Tyler Weaver, Yu, Yan, edetleon, jschleicher, v4hn

1.0.6 (2020-08-19)
------------------
* [maint] Adapt repository for splitted moveit_resources layout (`#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint] Migrate to clang-format-10
* [maint] Optimize includes (`#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* Contributors: Markus Vieth, Robert Haschke, Michael Görner

1.0.5 (2020-07-08)
------------------

1.0.4 (2020-05-30)
------------------
* Fix broken IKFast generator (`#2116 <https://github.com/ros-planning/moveit/issues/2116>`_)
* Contributors: Robert Haschke

1.0.3 (2020-04-26)
------------------
* [feature] KDL IK: constrain wiggled joints to limits (`#1953 <https://github.com/ros-planning/moveit/issues/1953>`_)
* [feature] IKFast: optional prefix for link names (`#1599 <https://github.com/ros-planning/moveit/issues/1599>`_)
  If you pass a `link_prefix` parameter in your `kinematics.yaml`, this string is prepended to the base and tip links.
  It allows multi-robot setups (e.g. dual-arm) and still instantiate the same solver for both manipulators.
* [feature] IKFast: increase verbosity of generated script (`#1434 <https://github.com/ros-planning/moveit/issues/1434>`_)
* [maint]   Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint]   Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint]   Windows build: Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [feature] IKFast: implement `Translation*AxisAngle4D` IK type (`#1823 <https://github.com/ros-planning/moveit/issues/1823>`_)
* [fix]     Fix possible division-by-zero (`#1809 <https://github.com/ros-planning/moveit/issues/1809>`_)
* Contributors: Christian Henkel, Martin Günther, Max Krichenbauer, Michael Görner, Robert Haschke, Sean Yen, Yu, Yan, jschleicher

1.0.2 (2019-06-28)
------------------
* [fix] KDL IK solver: fix handling of mimic joints (`#1490 <https://github.com/ros-planning/moveit/issues/1490>`_)
* [fix] Fix ROS apt-key in OpenRAVE docker image (`#1503 <https://github.com/ros-planning/moveit/issues/1503>`_)
* [fix] Fix ikfast plugin-generator script (`#1492 <https://github.com/ros-planning/moveit/issues/1492>`_, `#1449 <https://github.com/ros-planning/moveit/issues/1449>`_)
* Contributors: Immanuel Martini, Michael Görner, Robert Haschke

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Robert Haschke, Yu, Yan

1.0.0 (2019-02-24)
------------------
* [fix] catkin_lint issues (`#1341 <https://github.com/ros-planning/moveit/issues/1341>`_)
* [capability] Adapt ikfast plugin to new KinematicsBase API. `#1320 <https://github.com/ros-planning/moveit/issues/1320>`_
* [improve] cleanup LMA kinematics solver `#1318 <https://github.com/ros-planning/moveit/issues/1318>`_
* [improve] KDL IK solver improvements (`#1321 <https://github.com/ros-planning/moveit/issues/1321>`_)
* [improve] Kinematics tests, kdl cleanup `#1272 <https://github.com/ros-planning/moveit/issues/1272>`_, `#1294 <https://github.com/ros-planning/moveit/issues/1294>`_
* Contributors: Dave Coleman, Jorge Nicho, Mike Lautman, Robert Haschke

0.10.8 (2018-12-24)
-------------------

0.10.7 (2018-12-13)
-------------------

0.10.6 (2018-12-09)
-------------------
* [enhancement] Pass RobotModel to IK, avoiding multiple loading (`#1166 <https://github.com/ros-planning/moveit/issues/1166>`_)
  See `MIGRATION notes <https://github.com/ros-planning/moveit/blob/melodic-devel/MIGRATION.md>`_ for API changes in IK plugins,
  kdl, srv, or cached_ik for examples.
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* [maintenance] Use C++14 (`#1146 <https://github.com/ros-planning/moveit/issues/1146>`_)
* Contributors: Alex Moriarty, Michael Görner, Robert Haschke

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------

0.10.2 (2018-10-24)
-------------------
* [capability] add IKP_Translation{X,Y,Z}AxisAngle4D to the cpp template, see https://github.com/ros-planning/moveit/issues/548#issuecomment-316298918
* [maintenance] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* Contributors: Kei Okada, Mikael Arguedas, Mohmmad Ayman, Robert Haschke, mike lautman, v4hn

0.10.1 (2018-05-25)
-------------------
* migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* switch to ROS_LOGGER from CONSOLE_BRIDGE (`#874 <https://github.com/ros-planning/moveit/issues/874>`_)
* fixes to ikfast kinematics plugin (`#808 <https://github.com/ros-planning/moveit/issues/808>`_)
* Cached ik kinematics plugin (`#612 <https://github.com/ros-planning/moveit/issues/612>`_)
  add caching wrapper for IK solvers
* Contributors: Ian McMahon, Mark Moll, Mikael Arguedas, Robert Haschke, Xiaojian Ma

0.9.11 (2017-12-25)
-------------------
* Merge pull request `#714 <https://github.com/ros-planning/moveit/issues/714>`_ from henhenhen/kinetic-devel_lookup-param
  Use lookupParam() in kinematics plugins
* Replace param() with lookupParam() in srv_kinematics_plugin
* Replace param() with lookupParam() in lma_kinematics_plugin
* Replace param() with lookupParam() in kdl_kinematics_plugin
* Replace param() with lookupParam() in ikfast_kinematics_plugin
* Remove redundant parameter query
* Contributors: Henning Kayser, Isaac I.Y. Saito

0.9.10 (2017-12-09)
-------------------
* [fix][kinetic onward] Fix create_ikfast_moveit_plugin to comply with format 2 of the package.xml. Remove collada_urdf dependency `#666 <https://github.com/ros-planning/moveit/pull/666>`_
* [fix] create_ikfast_moveit_plugin: fixed directory variable for templates that were moved to ikfast_kinematics_plugin `#620 <https://github.com/ros-planning/moveit/issues/620>`_
* [improve] IKFastTemplate: Expand solutions to full joint range in searchPositionIK `#598 <https://github.com/ros-planning/moveit/issues/598>`_
* [improve] IKFastTemplate: searchPositionIK now returns collision-free solution which is nearest to seed state. (`#585 <https://github.com/ros-planning/moveit/issues/585>`_)
* Contributors: Dennis Hartmann, G.A. vd. Hoorn, Michael Görner, fsuarez6

0.9.9 (2017-08-06)
------------------
* [improve] Modify ikfast_template for getPositionIK single solution results (`#537 <https://github.com/ros-planning/moveit/issues/537>`_)
* Contributors: nsnitish

0.9.8 (2017-06-21)
------------------
* [build] ikfast_kinematics_plugin: Write XML files as UTF-8 (`#514 <https://github.com/ros-planning/moveit/issues/514>`_)
* [build] adjust cmake_minimum_required for add_compile_options (`#521 <https://github.com/ros-planning/moveit/issues/521>`_)
* [build] ikfast_kinematics_plugin: Add c++11 compile option. This is required for Kinetic.
* Contributors: Martin Guenther, Michael Goerner

0.9.7 (2017-06-05)
------------------
* [fix][Kinetic+] ikfast_kinematics_plugin: Add c++11 compile option `#515 <https://github.com/ros-planning/moveit/pull/515>`_
* [fix][Indigo] moveit_kinematics Eigen3 dependency (`#470 <https://github.com/ros-planning/moveit/issues/470>`_)
* Contributors: Martin Guenther, YuehChuan

0.9.6 (2017-04-12)
------------------

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
* [fix] Replace unused service dependency with msg dep (`#361 <https://github.com/ros-planning/moveit/issues/361>`_)
* [maintenance] Updated package.xml maintainers and author emails `#330 <https://github.com/ros-planning/moveit/issues/330>`_
* Contributors: Dave Coleman, Ian McMahon

0.9.2 (2016-11-05)
------------------
* [Maintenance] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman

0.9.0 (2016-10-19)
------------------
* Add dependency on new moveit_kinematics package
* Move moveit_ikfast into moveit_kinematics
* Moved kinematics plugins to new pkg moveit_kinematics
* Contributors: Dave Coleman

0.8.3 (2016-08-21)
------------------
