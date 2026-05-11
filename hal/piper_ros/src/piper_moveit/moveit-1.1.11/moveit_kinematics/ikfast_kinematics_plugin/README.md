MoveIt IKFast
==========
* Authors: Dave Coleman, CU Boulder; Jeremy Zoss, SwRI; David Butterworth, KAIST, Mathias Lüdtke, Fraunhofer IPA
* Date: 5/24/2013

Generates a IKFast kinematics plugin for MoveIt using [OpenRave](http://openrave.org/) generated cpp files.

Tested on ROS Hydro and greater with Catkin using OpenRave 0.8 with a 6dof and 7dof robot arm manipulator. Does not work with greater than 7dof.

[Documentation on docs.ros.org](http://docs.ros.org/indigo/api/moveit_ikfast/html/doc/ikfast_tutorial.html)

## Build Status

indigo-devel branch: [![Build Status](https://travis-ci.org/ros-planning/moveit_ikfast.png?branch=indigo-devel)](https://travis-ci.org/ros-planning/moveit_ikfast)
