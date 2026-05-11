^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_ros_perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.11 (2022-12-21)
-------------------

1.1.10 (2022-09-13)
-------------------
* Replace bind() with lambdas (`#3106 <https://github.com/ros-planning/moveit/issues/3106>`_)
* Contributors: Michael Görner, Robert Haschke

1.1.9 (2022-03-06)
------------------
* Use GLEW::GLEW link target (`#3079 <https://github.com/ros-planning/moveit/issues/3079>`_)
* Fix use of std::bind (`#3048 <https://github.com/ros-planning/moveit/issues/3048>`_)
* Contributors: Michael Görner, Tobias Fischer

1.1.8 (2022-01-30)
------------------

1.1.7 (2021-12-31)
------------------
* Switch to ``std::bind`` (`#2967 <https://github.com/ros-planning/moveit/issues/2967>`_)
* Add ns for depth image & pointcloud octomap updaters (`#2916 <https://github.com/ros-planning/moveit/issues/2916>`_)
* Contributors: Jochen Sprickerhof, Tim Redick

1.1.6 (2021-11-06)
------------------
* Use newly introduced cmake macro ``moveit_build_options()`` from ``moveit_core``
* Fix clipping of points: only considered points up to ``max_range`` (`#2848 <https://github.com/ros-planning/moveit/issues/2848>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Move ``OccMapTree`` to ``moveit_core/collision_detection`` (`#2684 <https://github.com/ros-planning/moveit/issues/2684>`_)
* Contributors: Michael Görner, Robert Haschke, Simon Schmeisser, Tyler Weaver, pvanlaar

1.1.5 (2021-05-23)
------------------
* Revert "Lock the octomap/octree while collision checking (`#2683 <https://github.com/ros-planning/moveit/issues/2683>`_)
* Contributors: Michael Görner

1.1.4 (2021-05-12)
------------------
* Lock the octomap/octree while collision checking (`#2596 <https://github.com/ros-planning/moveit/issues/2596>`_)
* Contributors: Simon Schmeisser

1.1.3 (2021-04-29)
------------------
* Document solution in ROS_ERROR on failed self-filtering (`#2627 <https://github.com/ros-planning/moveit/issues/2627>`_)
* Contributors: Michael Görner

1.1.2 (2021-04-08)
------------------
* Fixed flood of errors on startup for `mesh_filter` (`#2550 <https://github.com/ros-planning/moveit/issues/2550>`_)
* Enable mesh filter (`#2448 <https://github.com/ros-planning/moveit/issues/2448>`_)
* Contributors: Jafar Abdi, John Stechschulte

1.1.1 (2020-10-13)
------------------
* [fix] various issues with Noetic build (`#2327 <https://github.com/ros-planning/moveit/issues/2327>`_)
* [maint] Cleanup MSA includes (`#2351 <https://github.com/ros-planning/moveit/issues/2351>`_)
* [maint] Add comment to MOVEIT_CLASS_FORWARD (`#2315 <https://github.com/ros-planning/moveit/issues/2315>`_)
* Contributors: Felix von Drigalski, G.A. vd. Hoorn, Robert Haschke

1.1.0 (2020-09-04)
------------------
* [feature] Utilize new geometric_shapes functions to improve performance (`#2038 <https://github.com/ros-planning/moveit/issues/2038>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [fix] Fix getTransform() (`#2113 <https://github.com/ros-planning/moveit/issues/2113>`_)
* [fix] depth_image_octomap_updater: correctly set properties of debug images (`#1653 <https://github.com/ros-planning/moveit/issues/1653>`_)
* [maint] Optional cpp version setting (`#2166 <https://github.com/ros-planning/moveit/issues/2166>`_)
* [maint] clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_, `#2004 <https://github.com/ros-planning/moveit/issues/2004>`_, `#1419 <https://github.com/ros-planning/moveit/issues/1419>`_)
* [maint] Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint] Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [maint] NAMED logging for moveit_ros_perception (`#1897 <https://github.com/ros-planning/moveit/issues/1897>`_)
* [maint] Fix various build issues on Windows (`#1880 <https://github.com/ros-planning/moveit/issues/1880>`_)
* [maint] Fix compiler warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint] Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint] Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* Contributors: Ayush Garg, Bjar Ne, Dale Koenig, Dave Coleman, Henning Kayser, Jonathan Binney, Mahmoud Ahmed Selim, Markus Vieth, Martin Pecka, Matthias Nieuwenhuisen, Michael Görner, Robert Haschke, Sean Yen, Tyler Weaver, Yu, Yan, jschleicher

1.0.6 (2020-08-19)
------------------
* [maint] Migrate to clang-format-10
* [maint] Optimize includes (`#2229 <https://github.com/ros-planning/moveit/issues/2229>`_)
* [maint] Further increase acceptance threshold for mesh-filter test
* [maint] Prefer vendor-specific OpenGL library
* Contributors: Markus Vieth, Robert Haschke

1.0.5 (2020-07-08)
------------------
* [maint] Fix mesh_filter test (`#2044 <https://github.com/ros-planning/moveit/issues/2044>`_)
* Contributors: Bjar Ne

1.0.4 (2020-05-30)
------------------

1.0.3 (2020-04-26)
------------------
* [maint] Apply clang-tidy fix to entire code base (`#1394 <https://github.com/ros-planning/moveit/issues/1394>`_)
* [maint] Fix errors: catkin_lint 1.6.7 (`#1987 <https://github.com/ros-planning/moveit/issues/1987>`_)
* [maint] Windows build fixes
  * Fix header inclusion and other MSVC build errors (`#1636 <https://github.com/ros-planning/moveit/issues/1636>`_)
  * Fix binary artifact install locations. (`#1575 <https://github.com/ros-planning/moveit/issues/1575>`_)
* [maint] Use CMAKE_CXX_STANDARD to enforce c++14 (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [maint] Allow subclassing of point_containment_filter::ShapeMask. (`#1457 <https://github.com/ros-planning/moveit/issues/1457>`_)
* [fix]   `depth_image_octomap_updater`: reset depth transfer function to standard values (`#1661 <https://github.com/ros-planning/moveit/issues/1661>`_)
* [fix]   `depth_image_octomap_updater`: correctly set properties of debug images (`#1652 <https://github.com/ros-planning/moveit/issues/1652>`_)
* [maint] Move `occupancy_map_monitor` into its own package (`#1533 <https://github.com/ros-planning/moveit/issues/1533>`_)
* Contributors: Martin Pecka, Matthias Nieuwenhuisen, Robert Haschke, Sean Yen, Yu, Yan, jschleicher

1.0.2 (2019-06-28)
------------------
* [maintenance] Removed unnecessary null pointer checks on deletion (`#1410 <https://github.com/ros-planning/moveit/issues/1410>`_)
* Contributors: Mahmoud Ahmed Selim

1.0.1 (2019-03-08)
------------------
* [improve] Apply clang tidy fix to entire code base (Part 1) (`#1366 <https://github.com/ros-planning/moveit/issues/1366>`_)
* Contributors: Yu, Yan

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
* [maintenance] Use createUniqueInstance() (`#1104 <https://github.com/ros-planning/moveit/issues/1104>`_)
* [maintenance] Enforce OpenMP support for perception (`#1234 <https://github.com/ros-planning/moveit/issues/1234>`_)
* [maintenance] Replaced Eigen::Affine3d -> Eigen::Isometry3d (`#1096 <https://github.com/ros-planning/moveit/issues/1096>`_)
* [maintenance] Use C++14 (`#1146 <https://github.com/ros-planning/moveit/issues/1146>`_)
* Contributors: Alex Moriarty, Michael Görner, Robert Haschke

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29)
-------------------

0.10.3 (2018-10-29)
-------------------
* [fix] compiler warnings (`#1089 <https://github.com/ros-planning/moveit/issues/1089>`_)
* Contributors: Robert Haschke

0.10.2 (2018-10-24)
-------------------
* [fix] Eigen alignment issuses due to missing aligned allocation (`#1039 <https://github.com/ros-planning/moveit/issues/1039>`_)
* [fix] DepthImageOctomapUpdater not found error (`#954 <https://github.com/ros-planning/moveit/issues/954>`_)
* [fix] planning scene lock when octomap updates too quickly (`#920 <https://github.com/ros-planning/moveit/issues/920>`_)
* [enhancement] error message in shape_mask (`#828 <https://github.com/ros-planning/moveit/issues/828>`_)
* [maintenance] various compiler warnings (`#1038 <https://github.com/ros-planning/moveit/issues/1038>`_)
* [maintenance] disable unittests for moveit_ros_perception ... due to broken Mesa OpenGL (since version 17.x?) (`#982 <https://github.com/ros-planning/moveit/issues/982>`_)
* [maintenance] add minimum required pluginlib version (`#927 <https://github.com/ros-planning/moveit/issues/927>`_)
* Contributors: Adrian Zwiener, Martin Günther, Michael Görner, Mikael Arguedas, Mohmmad Ayman, Ridhwan Luthra, Robert Haschke, mike lautman

0.10.1 (2018-05-25)
-------------------
* boost::shared_ptr -> std::shared_ptr
* migration from tf to tf2 API (`#830 <https://github.com/ros-planning/moveit/issues/830>`_)
* [fix] make OpenGL parts optional (`#698 <https://github.com/ros-planning/moveit/issues/698>`_)
* Contributors: Bence Magyar, Ian McMahon, Lukas Bulwahn, Michael Görner, Mikael Arguedas, Robert Haschke

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------
* [improve] removed deprecated pluginlib macro (`#677 <https://github.com/ros-planning/moveit/issues/677>`_)
* Contributors: Mikael Arguedas

0.9.9 (2017-08-06)
------------------

0.9.8 (2017-06-21)
------------------

0.9.7 (2017-06-05)
------------------

0.9.6 (2017-04-12)
------------------
* [fix][moveit_ros_robot_interaction] `catkin_make -DCMAKE_ENABLE_TESTING=0` failure (`#478 <https://github.com/ros-planning/moveit/issues/478>`_)
* Contributors: Michael Goerner

0.9.5 (2017-03-08)
------------------
* [fix][moveit_ros_warehouse] gcc6 build error `#423 <https://github.com/ros-planning/moveit/pull/423>`_
* [enhancement] Remove "catch (...)" instances, catch std::exception instead of std::runtime_error (`#445 <https://github.com/ros-planning/moveit/issues/445>`_)
* Contributors: Bence Magyar, Dave Coleman

0.9.4 (2017-02-06)
------------------
* [maintenance] Remove custom cmake modules (`#418 <https://github.com/ros-planning/moveit/issues/418>`_)
* [maintenance] clang-format upgraded to 3.8 (`#367 <https://github.com/ros-planning/moveit/issues/367>`_)
* Contributors: Dave Coleman, Jochen Sprickerhof

0.9.3 (2016-11-16)
------------------

0.9.2 (2016-11-05)
------------------
* [Maintenace] Auto format codebase using clang-format (`#284 <https://github.com/ros-planning/moveit/issues/284>`_)
* Contributors: Dave Coleman

0.6.6 (2016-06-08)
------------------
* replaced cmake_modules dependency with eigen
* [jade] eigen3 adjustment
* remove unknown dependency sensor_msgs_generate_cpp
  dependencies are pulled in via ${catkin_LIBRARIES}
* Find X11 for build on OS X 10.11
* set empty display function for glut window
  With freeglut 3.0 moveit aborts over here, printing
  > ERROR: No display callback registered for window 1
  According to https://sourceforge.net/p/freeglut/bugs/229/
  and https://www.opengl.org/resources/libraries/glut/spec3/node46.html
  a callback *must* be registered for each window.
  With this patch moveit starts up as expected.
* Remove OpenMP parallelization, fixes `#563 <https://github.com/ros-planning/moveit_ros/issues/563>`_
* Removed trailing whitespace from entire repository
* last comment
* Added missing dependency on moveit_msgs package
* Contributors: Andriy Petlovanyy, Dave Coleman, Isaac I.Y. Saito, Kentaro Wada, Robert Haschke, Stefan Kohlbrecher, dg, v4hn

0.6.5 (2015-01-24)
------------------
* update maintainers
* adding RAII-based locking for OccMapTree
* moving lazy_free_space_updater into it's own library
* Contributors: Jonathan Bohren, Michael Ferguson

0.6.4 (2014-12-20)
------------------

0.6.3 (2014-12-03)
------------------
* port `moveit_ros#445 <https://github.com/ros-planning/moveit_ros/issues/445>`_ to indigo
* disable test that needs display when no display defined
* GL_TYPE() is a function in newer versions of OpenGL, this fixes tests on Ubuntu 14.04
* Contributors: Michael Ferguson

0.6.2 (2014-10-31)
------------------

0.6.1 (2014-10-31)
------------------
* fix linking error on OSX
* Contributors: Michael Ferguson

0.6.0 (2014-10-27)
------------------
* Fixing invalid iterators if filtered_cloud_topic is not set.
  Adding missing dependency on sensor_msgs.
  Fixing indentation, whitespace, and tabs.
  Incrementing PointCloud2Iterator pixel-at-a-time, not byte-at-a-time.
* remove PCL dependency
* Fixed issue with unordered_map and libc++ (LLVM, Mac OS X Mavericks)
  libc++ doesn't have std::tr1::unordered_map, just std::unordered_map
* Fixing OpenGL gl.h and glu.h inclusion on Mac OS X
* Contributors: Jason Ziglar, Marco Esposito, Sachin Chitta, Vincent Rabaud

0.5.19 (2014-06-23)
-------------------
* Fix [-Wreorder] warning.
* Address [cppcheck: duplicateExpression] error.
  The existing check for NaNs is in fact correct for IEEE-compliant floating
  numbers, i.e., if (a == a) then a is not a NaN, but confuses static code
  analyzers. This fix instead uses the isnan(a) macro from <cmath>.
* Prevent future conflicts between STL and Boost.
  mesh_filter_base.cpp was doing:
  using namespace std;
  using namespace boost;
  Considering that Boost is a testing ground for future standard additions,
  bringing the two namespaces into scope in the same translation unit is not
  the best idea. In this particular file, there's a potential conflict between
  C++'s and Boost's shared_ptr implementation.
* Make creation of std::pairs future-compiler-proof.
  Details:
  http://stackoverflow.com/questions/14623958/breaking-change-in-c11-with-make-pair-ty1-val1-const-ty2-val2
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.5.18 (2014-03-23)
-------------------

0.5.17 (2014-03-22)
-------------------
* update build system for ROS indigo
* update maintainer e-mail
* Contributors: Ioan Sucan

0.5.16 (2014-02-27)
-------------------

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
* comply to the new Table.msg
* Contributors: Vincent Rabaud

0.5.9 (2013-12-03)
------------------
* fix cloud offset

0.5.8 (2013-10-11)
------------------
* adds compliance for mesa versions <9.2

0.5.7 (2013-10-01)
------------------

0.5.6 (2013-09-26)
------------------
* fix `#320 <https://github.com/ros-planning/moveit_ros/issues/320>`_.
* fix `#318 <https://github.com/ros-planning/moveit_ros/issues/318>`_.

0.5.5 (2013-09-23)
------------------
* remove dep on pcl (pcl_conversions is sufficient)

0.5.4 (2013-08-14)
------------------
* add dependency on OpenCV2
* Pointcloud_octomap_updater compilation flags fixed

0.5.2 (2013-07-15)
------------------

0.5.1 (2013-07-14)
------------------
* find PCL separately

0.5.0 (2013-07-12)
------------------
* use pcl_conversions instead of pcl_ros
* white space fixes (tabs are now spaces)

0.4.5 (2013-07-03)
------------------

0.4.4 (2013-06-26)
------------------
* Fixes linkedit error on OS X
