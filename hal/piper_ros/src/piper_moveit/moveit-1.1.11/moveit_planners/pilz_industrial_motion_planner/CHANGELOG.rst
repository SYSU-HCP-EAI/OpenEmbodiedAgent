^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pilz_industrial_motion_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.11 (2022-12-21)
-------------------
* Merge fixes+improvements to ``PlanningScene`` editing in rviz: `#3263 <https://github.com/ros-planning/moveit/issues/3263>`_, `#3264 <https://github.com/ros-planning/moveit/issues/3264>`_, `#3296 <https://github.com/ros-planning/moveit/issues/3296>`_
* Contributors: Robert Haschke

1.1.10 (2022-09-13)
-------------------
* Add missing header for std::unique_ptr (`#3180 <https://github.com/ros-planning/moveit/issues/3180>`_)
* Switch to hpp headers of pluginlib
* Replace bind() with lambdas (`#3106 <https://github.com/ros-planning/moveit/issues/3106>`_)
* Contributors: Jochen Sprickerhof, Michael Görner

1.1.9 (2022-03-06)
------------------

1.1.8 (2022-01-30)
------------------
* Avoid downgrading default C++ standard (`#3043 <https://github.com/ros-planning/moveit/issues/3043>`_)
* Resolve ambiguous function specification (`#3040 <https://github.com/ros-planning/moveit/issues/3040>`_)
* Contributors: Jochen Sprickerhof

1.1.7 (2021-12-31)
------------------
* Pilz planner: improve reporting of invalid start joints (`#3000 <https://github.com/ros-planning/moveit/issues/3000>`_)
* Switch to ``std::bind`` (`#2967 <https://github.com/ros-planning/moveit/issues/2967>`_)
* Contributors: Jochen Sprickerhof, Robert Haschke, v4hn

1.1.6 (2021-11-06)
------------------
* Fix calculation of subframe offset (`#2890 <https://github.com/ros-planning/moveit/issues/2890>`_)
* Remove unused moveit_planning_execution.launch
* Use test_environment.launch in unittests (`#2949 <https://github.com/ros-planning/moveit/issues/2949>`_)
* Rename launch argument execution_type -> fake_execution_type
* Improve error messages (`#2940 <https://github.com/ros-planning/moveit/issues/2940>`_)
  * Remove deprecated xacro --inorder
  * Don't complain about missing limits for irrelevant JMGs
  * Avoid duplicate error messages
  * Downgrade ERROR to WARN when checking joint limits, report affected joint name
  * Quote (possibly empty) planner id
* Consider attached bodies for planning (`#2773 <https://github.com/ros-planning/moveit/issues/2773>`_, `#2824 <https://github.com/ros-planning/moveit/issues/2824>`_, `#2878 <https://github.com/ros-planning/moveit/issues/2878>`_)
* Fix collision detection: consider current PlanningScene (`#2803 <https://github.com/ros-planning/moveit/issues/2803>`_)
* Add planning_pipeline_id to MotionSequence action + service (`#2755 <https://github.com/ros-planning/moveit/issues/2755>`_)
* clang-tidy: modernize-make-shared, modernize-make-unique (`#2762 <https://github.com/ros-planning/moveit/issues/2762>`_)
* Contributors: Felix von Drigalski, Leroy Rügemer, Michael Görner, Robert Haschke, Tom Noble, aa-tom, cambel, pvanlaar

1.1.5 (2021-05-23)
------------------
* Allow selecting planning pipeline in MotionSequenceAction (`#2657 <https://github.com/ros-planning/moveit/issues/2657>`_)
* Contributors: Felix von Drigalski

1.1.4 (2021-05-12)
------------------

1.1.3 (2021-04-29)
------------------
* Skip fixed joints when checking velocity limits (`#2610 <https://github.com/ros-planning/moveit/issues/2610>`_)
* Contributors: Christian Landgraf

1.1.2 (2021-04-08)
------------------
* Fix formatting errors
* Replaced eigen+kdl conversions with tf2_eigen + tf2_kdl (`#2472 <https://github.com/ros-planning/moveit/issues/2472>`_)
* Add missing dependency on joint_limits_interface (`#2487 <https://github.com/ros-planning/moveit/issues/2487>`_)
* Use kinematics solver timeout if not specified otherwise (`#2489 <https://github.com/ros-planning/moveit/issues/2489>`_)
* pilz planner: add string includes (`#2483 <https://github.com/ros-planning/moveit/issues/2483>`_)
* Upgrade cmake_minimum_required to 3.1 (`#2453 <https://github.com/ros-planning/moveit/issues/2453>`_)
* Add Pilz industrial motion planner (`#1893 <https://github.com/ros-planning/moveit/issues/1893>`_)
* Contributors: Pilz GmbH and Co. KG, Christian Henkel, Immanuel Martini, Michael Görner, Robert Haschke, Tyler Weaver, petkovich
