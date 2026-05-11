# MoveIt ROS Control Plugin

This package provides plugins of base class `moveit_controller_manager::MoveItControllerManager` and a new plugin base class for `moveit_controller_manager::MoveItControllerHandle` allocators.
The allocator class is necessary because `moveit_controller_manager::MoveItControllerHandle` needs a name passed to the constructor.
Two variantes are provided, `moveit_ros_control_interface::MoveItControllerManager` for interfacing a single ros_control node and `moveit_ros_control_interface::MoveItMultiControllerManager` for seamless integration with any number of ros_control nodes.


## moveit_ros_control_interface::MoveItControllerManager
This plugin interfaces a single ros_control-driven node in the namespace given in the `~ros_control_namespace` ROS parameter.
It polls all controllers via the `list_controllers` service and passes their properties to MoveIt.
The polling is throttled to 1 Hertz.

### Handle plugins
The actual handle creation is delegated to allocator plugins of base class `moveit_ros_control_interface::ControllerHandleAllocator`.
These plugins should be registered with lookup names that match the corresponding controller types.

Currently plugins for `position_controllers/JointTrajectoryController`, `velocity_controllers/JointTrajectoryController` and `effort_controllers/JointTrajectoryController` are available, which simply wrap `moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle` instances.

### Setup
In your MoveIt launch file (e.g. `ROBOT_moveit_config/launch/ROBOT_moveit_controller_manager.launch.xml`) set the `moveit_controller_manager` parameter:
```
<param name="moveit_controller_manager" value="moveit_ros_control_interface::MoveItControllerManager" />
```

And make sure to set the `ros_control_namespace` parameter to the namespace (without the /controller_manager/ part) of the ros_control-based node you like to interface.
If you are using the `moveit_setup_assistent` you can add it to `ROBOT_moveit_config/config/controllers.yaml`, e.g.:
```
ros_control_namespace: /ROS_CONTROL_NODE
controller_list:
  - name: /ROS_CONTROL_NODE/position_trajectory_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - joint_a1
      - joint_a2
      - joint_a3
      - joint_a4
      - joint_a5
      - joint_a6
      - joint_a7
```

### Controller switching
MoveIt can decide which controllers have to be started and stopped.
Since only controller names with registered allocator plugins are handed over to MoveIt, this implementation takes care of stopping other conflicting controllers based on their claimed resources and the resources for the to-be-started controllers.

### Namespaces
All controller names get prefixed by the namespace of the ros_control node.
For this to work the controller names should not contain slashes. This is a strict requirement if the ros_control  namespace is `/`.

## moveit_ros_control_interface::MoveItMultiControllerManager

This plugin does not need further configuration. It polls the ROS master for services and identifies ros_control nodes automatically.
It spawns `moveit_ros_control_interface::MoveItControllerManager` instances with their namespace and takes cares of proper delegation.


### Setup
Just set the `moveit_controller_manager` parameter in your MoveIt launch file (e.g. `ROBOT_moveit_config/launch/ROBOT_moveit_controller_manager.launch.xml`)
```
<param name="moveit_controller_manager" value="moveit_ros_control_interface::MoveItMultiControllerManager" />
```
