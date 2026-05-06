^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_servo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Backport position limit enforcement from MoveIt2 (`#2898 <https://github.com/ros-planning/moveit/issues/2898>`_)
* Use newly introduced cmake macro ``moveit_build_options()`` from ``moveit_core``
* Minor fixups (`#2759 <https://github.com/ros-planning/moveit/issues/2759>`_)
* Remove gtest include from non-testing source (`#2747 <https://github.com/ros-planning/moveit/issues/2747>`_)
* Fix an off-by-one error in servo_calcs.cpp (`#2740 <https://github.com/ros-planning/moveit/issues/2740>`_)
* Refactor ``moveit_servo::LowPassFilter`` to be assignable (`#2722 <https://github.com/ros-planning/moveit/issues/2722>`_)
* Contributors: Griswald Brooks, Michael Görner, Michael Wiznitzer, Robert Haschke, luisrayas3, toru-kuga

1.1.5 (2021-05-23)
------------------

1.1.4 (2021-05-12)
------------------

1.1.3 (2021-04-29)
------------------
* Avoid joint jump when SuddenHalt() is called in velocity mode (`#2594 <https://github.com/ros-planning/moveit/issues/2594>`_)
* Contributors: Nathan Brooks

1.1.2 (2021-04-08)
------------------
* Fix formatting errors
* Halt Servo command on Pose Tracking stop (`#2501 <https://github.com/ros-planning/moveit/issues/2501>`_)
* stop_requested\_ flag clearing fix (`#2537 <https://github.com/ros-planning/moveit/issues/2537>`_)
* Add missing include (`#2519 <https://github.com/ros-planning/moveit/issues/2519>`_)
* Refactor Servo velocity bounds enforcement (`#2471 <https://github.com/ros-planning/moveit/issues/2471>`_)
* Protect paused\_ flag, for thread safety (`#2494 <https://github.com/ros-planning/moveit/issues/2494>`_)
* Do not break out of loop -- need to update low pass filters (`#2496 <https://github.com/ros-planning/moveit/issues/2496>`_)
* Fix initial angle error is always 0 (`#2464 <https://github.com/ros-planning/moveit/issues/2464>`_)
* Add an important sleep in Servo pose tracking (`#2463 <https://github.com/ros-planning/moveit/issues/2463>`_)
* Prevent moveit_servo transforms between fixed frames from causing timeout (`#2418 <https://github.com/ros-planning/moveit/issues/2418>`_)
* Low latency mode (`#2401 <https://github.com/ros-planning/moveit/issues/2401>`_)
* Move timer initialization down to fix potential race condition
* Fix pose tracking race condition (`#2395 <https://github.com/ros-planning/moveit/issues/2395>`_)
* Cleanup current state handling in servo (`#2372 <https://github.com/ros-planning/moveit/issues/2372>`_)
* Change servo namespacing logic (`#2354 <https://github.com/ros-planning/moveit/issues/2354>`_)
* Fix servo trajectory point timestamping (`#2375 <https://github.com/ros-planning/moveit/issues/2375>`_)
* Fix ordering of windup args to control_toolbox::Pid (`#2370 <https://github.com/ros-planning/moveit/issues/2370>`_)
* Contributors: AdamPettinger, AndyZe, Jere Liukkonen, Michael Görner, Nathan Brooks, Tyler Weaver, parunapu

1.1.1 (2020-10-13)
------------------
* [feature] A library for servoing toward a moving pose (`#2203 <https://github.com/ros-planning/moveit/issues/2203>`_)
* [feature] Refactor velocity limit enforcement and add a unit test (`#2260 <https://github.com/ros-planning/moveit/issues/2260>`_)
* [fix] Servo thread interruption (`#2314 <https://github.com/ros-planning/moveit/issues/2314>`_)
* [fix] Servo heap-buffer-overflow bug (`#2307 <https://github.com/ros-planning/moveit/issues/2307>`_)
* [maint] Cleanup MSA includes (`#2351 <https://github.com/ros-planning/moveit/issues/2351>`_)
* Contributors: AndyZe, Robert Haschke, Tyler Weaver

1.1.0 (2020-09-04)
------------------
* [feature] Update last_sent_command\_ at ServoCalcs start (`#2249 <https://github.com/ros-planning/moveit/issues/2249>`_)
* [feature] Add a utility to print collision pairs (`#2275 <https://github.com/ros-planning/moveit/issues/2275>`_)
* [fix] Various fixes for upcoming Noetic release (`#2180 <https://github.com/ros-planning/moveit/issues/2180>`_)
* [maint] add soname version to moveit_servo (`#2266 <https://github.com/ros-planning/moveit/issues/2266>`_)
* [maint] delete python integration tests (`#2186 <https://github.com/ros-planning/moveit/issues/2186>`_)
* Contributors: AdamPettinger, AndyZe, Robert Haschke, Ruofan Xu, Tyler Weaver, v4hn

1.0.6 (2020-08-19)
------------------
* [feature] A ROS service to reset the Servo status (`#2246 <https://github.com/ros-planning/moveit/issues/2246>`_)
* [feature] Check collisions during joint motions, too (`#2204 <https://github.com/ros-planning/moveit/issues/2204>`_)
* [fix]     Correctly set velocities to zero when stale (`#2255 <https://github.com/ros-planning/moveit/issues/2255>`_)
* [maint]   Remove unused yaml param (`#2232 <https://github.com/ros-planning/moveit/issues/2232>`_)
* [maint]   Adapt repository for splitted moveit_resources layout (`#2199 <https://github.com/ros-planning/moveit/issues/2199>`_)
* [maint]   Migrate to clang-format-10
* Contributors: AndyZe, Robert Haschke, Ruofan Xu, Michael Görner

1.0.5 (2020-07-08)
------------------
* [maint]   Minor moveit_servo header cleanup (`#2173 <https://github.com/ros-planning/moveit/issues/2173>`_)
* [maint]   Move and rename to moveit_ros/moveit_servo (`#2165 <https://github.com/ros-planning/moveit/issues/2165>`_)
* [maint]   Changes before porting to ROS2 (`#2151 <https://github.com/ros-planning/moveit/issues/2151>`_)
  * throttle warning logs
  * ROS1 Basic improvements and changes
  * Fixes to drift dimensions, singularity velocity scaling
  * tf name changes, const fixes, slight logic changes
  * Move ROS_LOG_THROTTLE_PERIOD to cpp files
  * Track staleness of joint and twist seperately
  * Ensure joint_trajectory output is always populated with something, even when no jog
  * Fix joint trajectory redundant points for gazebo pub
  * Fix crazy joint jog from bad Eigen init
  * Fix variable type in addJointIncrements()
  * Initialize last sent command in constructor
  * More explicit joint_jog_cmd\ and twist_stamped_cmd\ names
  * Add comment clarying transform calculation / use
* [fix]     Fix access past end of array bug (`#2155 <https://github.com/ros-planning/moveit/issues/2155>`_)
* [maint]   Remove duplicate line (`#2154 <https://github.com/ros-planning/moveit/issues/2154>`_)
* [maint]   pragma once in jog_arm.h (`#2152 <https://github.com/ros-planning/moveit/issues/2152>`_)
* [feature] Simplify communication between threads (`#2103 <https://github.com/ros-planning/moveit/issues/2103>`_)
  * get latest joint state c++ api
  * throttle warning logs
  * publish from jog calcs timer, removing redundant timer and internal messaging to main timer
  * outgoing message as pool allocated shared pointer for zero copy
  * replace jog_arm shared variables with ros pub/sub
  * use built in zero copy message passing instead of spsc_queues
  * use ros timers instead of threads in jog_arm
* [feature] Added throttle to jogarm accel limit warning (`#2141 <https://github.com/ros-planning/moveit/issues/2141>`_)
* [feature] Time-based collision avoidance (`#2100 <https://github.com/ros-planning/moveit/issues/2100>`_)
* [fix]     Fix crash on empty jog msgs (`#2094 <https://github.com/ros-planning/moveit/issues/2094>`_)
* [feature] Jog arm dimensions (`#1724 <https://github.com/ros-planning/moveit/issues/1724>`_)
* [maint]   Clang-tidy fixes (`#2050 <https://github.com/ros-planning/moveit/issues/2050>`_)
* [feature] Keep updating joints, even while waiting for a valid command (`#2027 <https://github.com/ros-planning/moveit/issues/2027>`_)
* [fix]     Fix param logic bug for self- and scene-collision proximity thresholds (`#2022 <https://github.com/ros-planning/moveit/issues/2022>`_)
* [feature] Split collision proximity threshold (`#2008 <https://github.com/ros-planning/moveit/issues/2008>`_)
  * separate proximity threshold values for self-collisions and scene collisions
  * increase default value of scene collision proximity threshold
  * deprecate old parameters
* [fix]     Fix valid command flags (`#2013 <https://github.com/ros-planning/moveit/issues/2013>`_)
  * Rename the 'zero command flag' variables for readability
  * Reset flags when incoming commands timeout
  * Remove debug line, clang format
* [maint]   Use default move constructor + assignment operators for MoveItCpp. (`#2004 <https://github.com/ros-planning/moveit/issues/2004>`_)
* [fix]     Fix low-pass filter initialization (`#1982 <https://github.com/ros-planning/moveit/issues/1982>`_)
  * Pause/stop JogArm threads using shared atomic bool variables
  * Add pause/unpause flags for jog thread
  * Verify valid joints by filtering for active joint models only
  * Remove redundant joint state increments
  * Wait for initial jog commands in main loop
* [fix]     Remove duplicate collision check in JogArm (`#1986 <https://github.com/ros-planning/moveit/issues/1986>`_)
* [feature] Add a binary collision check (`#1978 <https://github.com/ros-planning/moveit/issues/1978>`_)
* [feature] Publish more detailed warnings (`#1915 <https://github.com/ros-planning/moveit/issues/1915>`_)
* [feature] Use wait_for_service() to fix flaky tests (`#1946 <https://github.com/ros-planning/moveit/issues/1946>`_)
* [maint]   Fix versioning (`#1948 <https://github.com/ros-planning/moveit/issues/1948>`_)
* [feature] SRDF velocity and acceleration limit enforcement (`#1863 <https://github.com/ros-planning/moveit/issues/1863>`_)
* [maint]   Replace namespaces robot_state and robot_model with moveit::core (`#1924 <https://github.com/ros-planning/moveit/issues/1924>`_)
* [fix]     JogArm C++ API fixes (`#1911 <https://github.com/ros-planning/moveit/issues/1911>`_)
* [feature] A ROS service to enable task redundancy (`#1855 <https://github.com/ros-planning/moveit/issues/1855>`_)
* [fix]     Fix segfault with uninitialized JogArm thread (`#1882 <https://github.com/ros-planning/moveit/issues/1882>`_)
* [feature] Add warnings to moveit_jog_arm low pass filter (`#1872 <https://github.com/ros-planning/moveit/issues/1872>`_)
* [maint]   Use CMAKE_CXX_STANDARD to enforce c++14 for portability (`#1607 <https://github.com/ros-planning/moveit/issues/1607>`_)
* [fix]     Fix initial end effector transform jump (`#1871 <https://github.com/ros-planning/moveit/issues/1871>`_)
* [feature] Rework the halt msg functionality (`#1868 <https://github.com/ros-planning/moveit/issues/1868>`_)
* [fix]     Various small fixes (`#1859 <https://github.com/ros-planning/moveit/issues/1859>`_)
* [maint]   Improve formatting in comments
* [fix]     Prevent a crash at velocity limit (`#1837 <https://github.com/ros-planning/moveit/issues/1837>`_)
* [feature] Remove scale/joint parameter (`#1838 <https://github.com/ros-planning/moveit/issues/1838>`_)
* [feature] Pass planning scene monitor into cpp interface (`#1849 <https://github.com/ros-planning/moveit/issues/1849>`_)
* [maint]   Move attribution below license file, standardize with MoveIt (`#1847 <https://github.com/ros-planning/moveit/issues/1847>`_)
* [maint]   Reduce console output warnings (`#1845 <https://github.com/ros-planning/moveit/issues/1845>`_)
* [fix]     Fix command frame transform computation (`#1842 <https://github.com/ros-planning/moveit/issues/1842>`_)
* [maint]   Fix dependencies + catkin_lint issues
* [feature] Update link transforms before calling checkCollision on robot state in jog_arm (`#1825 <https://github.com/ros-planning/moveit/issues/1825>`_)
* [feature] Add atomic bool flags for terminating JogArm threads gracefully (`#1816 <https://github.com/ros-planning/moveit/issues/1816>`_)
* [feature] Get transforms from RobotState instead of TF (`#1803 <https://github.com/ros-planning/moveit/issues/1803>`_)
* [feature] Add a C++ API (`#1763 <https://github.com/ros-planning/moveit/issues/1763>`_)
* [maint]   Fix unused parameter warnings (`#1773 <https://github.com/ros-planning/moveit/issues/1773>`_)
* [maint]   Update license formatting (`#1764 <https://github.com/ros-planning/moveit/issues/1764>`_)
* [maint]   Unify jog_arm package to be C++14 (`#1762 <https://github.com/ros-planning/moveit/issues/1762>`_)
* [fix]     Fix jog_arm segfault (`#1692 <https://github.com/ros-planning/moveit/issues/1692>`_)
* [fix]     Fix double mutex unlock (`#1672 <https://github.com/ros-planning/moveit/issues/1672>`_)
* [maint]   Rename jog_arm->moveit_jog_arm (`#1663 <https://github.com/ros-planning/moveit/issues/1663>`_)
* [feature] Do not wait for command msg to start spinning (`#1603 <https://github.com/ros-planning/moveit/issues/1603>`_)
* [maint]   Update jog_arm README with rviz config (`#1614 <https://github.com/ros-planning/moveit/issues/1614>`_)
* [maint]   Switch from include guards to pragma once (`#1615 <https://github.com/ros-planning/moveit/issues/1615>`_)
* [maint]   Separate moveit_experimental packages (`#1606 <https://github.com/ros-planning/moveit/issues/1606>`_)
* [feature] Use UR5 example (`#1605 <https://github.com/ros-planning/moveit/issues/1605>`_)
* [feature] Sudden stop for critical issues, filtered deceleration otherwise (`#1468 <https://github.com/ros-planning/moveit/issues/1468>`_)
* [feature] Change 2nd order Butterworth low pass filter to 1st order (`#1483 <https://github.com/ros-planning/moveit/issues/1483>`_)
* [maint]   Remove ! from MoveIt name (`#1590 <https://github.com/ros-planning/moveit/issues/1590>`_)
* [feature] JogArm: Remove dependency on move_group node (`#1569 <https://github.com/ros-planning/moveit/issues/1569>`_)
* [fix]     Fix jog arm CI integration test (`#1466 <https://github.com/ros-planning/moveit/issues/1466>`_)
* [feature] A jogging PR for Melodic. (`#1360 <https://github.com/ros-planning/moveit/issues/1360>`_)
  * Allow for joints in the msg that are not part of the MoveGroup.
  * Switching to the Panda robot model for tests.
  * Blacklist the test as I can't get it to pass Travis (fine locally).
  * Throttling all warnings. Fix build warning re. unit vs int comparison.
  * Continue to publish commands even if stationary
  * Scale for 'unitless' commands is not tied to publish_period.
  * New function name for checkIfJointsWithinBounds()
  * Configure the number of msgs to publish when stationary.
  * Run jog_calcs at the same rate as the publishing thread.
  * Better comments in config file, add spacenav_node dependency
  * Add spacenav_node to CMakeLists.
* Contributors: AdamPettinger, AndyZe, Ayush Garg, Dale Koenig, Dave Coleman, Jonathan Binney, Paul Verhoeckx, Henning Kayser, Jafar Abdi, John Stechschulte, Mike Lautman, Robert Haschke, SansoneG, jschleicher, Tyler Weaver, rfeistenauer

1.0.1 (2019-03-08)
------------------

1.0.0 (2019-02-24)
------------------

0.10.8 (2018-12-24)
-------------------

0.10.5 (2018-11-01)
-------------------

0.10.4 (2018-10-29 19:44)
-------------------------

0.10.3 (2018-10-29 04:12)
-------------------------

0.10.2 (2018-10-24)
-------------------

0.10.1 (2018-05-25)
-------------------

0.10.0 (2018-05-22)
-------------------

0.9.11 (2017-12-25)
-------------------

0.9.10 (2017-12-09)
-------------------

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

0.9.4 (2017-02-06)
------------------

0.9.3 (2016-11-16)
------------------

0.9.2 (2016-11-05)
------------------

0.9.1 (2016-10-21)
------------------
