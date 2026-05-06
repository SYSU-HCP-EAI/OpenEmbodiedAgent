# Cached IK Kinematics Plugin

* Author: Mark Moll, Rice University

The Cached IK Kinematics Plugin creates a persistent cache of IK solutions. This cache is then used to speed up any other IK solver. A call to an IK solver will use a similar state in the cache as a seed for the IK solver. If that fails to return a solution, the IK solver is called again with the user-specified seed state. New IK solutions that are sufficiently different from states in the cache are added to the cache. Periodically, the cache is saved to disk.

## Basic Usage

To use the Cached IK Kinematics Plugin, you need to modify the file `kinematics.yaml` for your robot. Change lines like these:

    manipulator:
      kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin

to this:

    manipulator:
      kinematics_solver: cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin
      # optional parameters for caching:
      max_cache_size: 10000
      min_pose_distance: 1
      min_joint_config_distance: 4

The cache size can be controlled with an absolute cap (`max_cache_size`) or with a distance threshold on the end effector pose (`min_pose_distance`) or robot joint state (`min_joint_config_distance`). Normally, the cache files are saved to the current working directory (which is usually `${HOME}/.ros`, not the directory where you ran `roslaunch`), in a subdirectory for each robot. Possible values for `kinematics_solver` are:

- `cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin`: a wrapper for the default KDL IK solver.
- `cached_ik_kinematics_plugin/CachedSrvKinematicsPlugin`: a wrapper for the solver that uses ROS service calls to communicate with external IK solvers.
- `cached_ik_kinematics_plugin/CachedTRACKinematicsPlugin`: a wrapper for the TRAC IK solver. This solver is only available if the TRAC IK kinematics plugin is detected at compile time.
- `cached_ik_kinematics_plugin/CachedUR5KinematicsPlugin`: a wrapper for the analytic IK solver for the UR5 arm (similar solvers exist for the UR3 and UR10). This is only for illustrative purposes; the caching just adds extra overhead to the solver.

## Measuring IK Solver Performance

To evaluate IK solver performance and to facilitate tuning of the caching parameters there is a program called `measure_ik_call_cost`. This program can be run like so:

    roslaunch moveit_kinematics measure_ik_call_cost.launch robot:=fetch

This will look for the `fetch_moveit_config` package. The IK solver is then called 10,000 times for each planning group. At each iteration the IK solver will attempt to recover a randomly sampled configuration given a corresponding end effector position. The default joint positions are used as the seed state. By changing the parameters in `kinematics.yaml` you can create different cache files (the cache file names include the parameter settings) and measure the performance. The performance is measured by the average time to call the IK solver and the number of times the IK solver failed to return a solution. It will also report the percentage of random configurations that are in self-collision. Those configurations are skipped for IK solver calls.

For the Fetch robot we get the following numbers for average time per IK call, measured over 10,000 calls:

|                | KDL, no caching | KDL, cached | TRAC-IK, no caching | TRAC-IK, cached |
|----------------|-----------------|-------------|---------------------|-----------------|
| arm            | 0.00396232      | 0.00292258  | 0.000577042         | 0.000583098     |
| arm with torso | 0.0130856       | 0.00846116  | 0.00133652          | 0.00124825      |

This illustrates two points: (1) if the IK solver is slow, significant improvements can be obtained with caching, and (2) if the IK solver is fast, then caching doesn't improve performance much and can actually make it worse. Note that these numbers were obtained with the default cache parameters for `min_pose_distance` and `min_joint_config_distance`; performance can potentially be improved by tuning these parameters for a given robot.

Below is a complete list of all arguments:

- `robot`: the name of the corresponding MoveIt config package
- `group`: the joint group to measure (by default performance is reported for all joint groups)
- `tip`: the name of the end effector (by default the default end effectors are used)
- `num`: the number of IK calls per joint group
- `reset_to_default`: whether to reset to default values before calling IK (rather than seed the solver with the correct IK solution). By default this parameter is set to `true`. Set to `false` to speed up filling the cache (but performance numbers are meaningless in this case).

## Advanced Usage: Creating Wrappers for Other IK Solvers

The Cached IK Kinematics Plugin is implemented as a wrapper around classed derived from the [`kinematics::KinematicsBase` abstract base class](http://docs.ros.org/latest-lts/api/moveit_core/html/classkinematics_1_1KinematicsBase.html). Wrappers for the `kdl_kinematics_plugin::KDLKinematicsPlugin` and `srv_kinematics_plugin::SrvKinematicsPlugin` classes are already included in the plugin. For any other solver, you can create a new kinematics plugin. The C++ code for doing so is extremely simple; here is the code to create a wrapper for the KDL solver:

    #include "cached_ik_kinematics_plugin.h"
    #include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
    #include <pluginlib/class_list_macros.hpp>
    PLUGINLIB_EXPORT_CLASS(cached_ik_kinematics_plugin::CachedIKKinematicsPlugin<kdl_kinematics_plugin::KDLKinematicsPlugin>, kinematics::KinematicsBase);

In the catkin `package.xml` file for your plugin, you add these lines just before `</package>`:

    <export>
      <moveit_core plugin="${prefix}/my_ik_plugin.xml"/>
    </export>

Next, create the file my_ik_plugin.xml with the following contents:

    <library path="lib/libmy_ik_plugin">
      <class name="cached_ik_kinematics_plugin/CachedMyKinematicsPlugin" type="cached_ik_kinematics_plugin::CachedIKKinematicsPlugin<my_kinematics_plugin::MyKinematicsPlugin>" base_class_type="kinematics::KinematicsBase">
        <description>
          A kinematics plugin for persistently caching IK solutions computed with the KDL kinematics plugin.
        </description>
      </class>
    </library>

**Note:** For IK solvers that implement the multi-tip API, use the `cached_ik_kinematics_plugin::CachedMultiTipIKKinematicsPlugin` wrapper class instead of `cached_ik_kinematics_plugin::CachedIKKinematicsPlugin`.
