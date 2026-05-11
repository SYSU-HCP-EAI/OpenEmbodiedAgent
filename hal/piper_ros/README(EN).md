# AGX Robotic Arm

[中文](README.MD)

![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-noetic-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

```shell
git clone https://github.com/agilexrobotics/piper_ros.git
```

```shell
cd piper_ros
git checkout noetic
```

|Description | Document|
|---|---|
|Moveit|[Moveit README](src/piper_moveit/README(EN).md)|
|Simulation|[Simulation README](src/piper_sim/README(EN).md)|

## 0 Note the URDF version

DH parameters before S-V1.6-3 firmware version are obtained by taking the contact limit of each joint as the initial coordinate

DH parameters after S-V1.6-3 firmware version offset the coordinate system of j2 and j3 by 2 degrees

The default URDF is now the latter

|firmware version |urdf|
|---|---|
|current version < S-V1.6-3|`piper_description_old.urdf`|
|current version >= S-V1.6-3|`piper_description.urdf`|

## 1 Installation Method

### 1.1 Install Dependencies

```shell
pip3 install python-can
```

```shell
pip3 install piper_sdk
```

```shell
source /opt/ros/noetic/setup.bash
sudo apt install python3-wstool python3-catkin-tools python3-rosdep ros-noetic-ruckig
sudo apt-get install ros-noetic-eigen-stl-containers ros-noetic-geometric-shapes ros-noetic-moveit-msgs ros-noetic-srdfdom ros-noetic-pybind11-catkin
sudo apt-get install ros-noetic-moveit-resources-panda-moveit-config ros-noetic-ompl ros-noetic-warehouse-ros ros-noetic-eigenpy ros-noetic-rosparam-shortcuts
```

**NOTE**

If you encounter compilation errors related to MoveIt that cannot be resolved, you can directly install the official MoveIt package by running:sudo apt install ros-$ROS_DISTRO-moveitAfter installation, either delete the `src/piper_moveit/moveit-1.1.11` directory, or create an empty file named `CATKIN_IGNORE` inside that directory to make ROS ignore this package during compilation. Then, delete the `build/` and `devel/` directories and recompile.

```shell
cd piper_ros
catkin_make
```

### 2.1 Enable CAN Module

First, set the shell script parameters.

#### 2.1.1 Single Robotic Arm

##### 1) PC with only one USB-to-CAN module plugged in

- **Use the `can_activate.sh` script here**

Simply execute

```bash
bash can_activate.sh can0 1000000
```

##### 2) PC with multiple USB-to-CAN modules plugged in

- **Use the `can_activate.sh` script here**

Unplug all CAN modules.

Only insert the CAN module connected to the robotic arm into the PC and execute

```shell
sudo ethtool -i can0 | grep bus
```

Record the `bus-info` value, for example, `1-2:1.0`.

ps: **Typically, the first inserted CAN module will default to can0. If no CAN is detected, use `bash find_all_can_port.sh` to check which CAN name corresponds to the USB address.**

Assuming the above operation records `bus-info` as `1-2:1.0`, execute the following command to check if the CAN device was successfully activated:

```bash
bash can_activate.sh can_piper 1000000 "1-2:1.0"
```

ps: **This means the CAN device inserted into the `1-2:1.0` USB port is renamed to `can_piper`, with a baud rate of 1000000, and it is activated.**

Then execute `ifconfig` to check if `can_piper` appears. If so, the CAN module has been successfully configured.

### 2.2 Running the Node

#### 2.2.1 Single Robotic Arm

Node name: `piper_ctrl_single_node.py`

Parameters:

```shell
can_port: the name of the CAN route to open
auto_enable: whether to enable automatically; True enables as soon as the program starts
# Note that if set to False, after interrupting the program and restarting the node, the arm will retain the last state
# If the arm was enabled last time, it remains enabled after restarting
# If the arm was disabled last time, it remains disabled after restarting
gripper_exist: whether there is an end effector, set to True if present
rviz_ctrl_flag: whether to use RViz to send joint angle messages; True if RViz sends joint angle messages
# Since the range of joint7 in RViz is [0, 0.04], but the actual gripper range is 0.08m, enabling RViz control will double the joint7 value
```

`start_single_piper.launch`:

```xml
<launch>
  <arg name="can_port" default="can0" />
  <arg name="auto_enable" default="true" />
  <arg name="gripper_val_mutiple" default="1" />
  <!-- <include file="$(find piper_description)/launch/display_xacro.launch"/> -->
  <!-- Start the robotic arm node -->
  <node name="piper_ctrl_single_node" pkg="piper" type="piper_ctrl_single_node.py" output="screen">
    <param name="can_port" value="$(arg can_port)" />
    <param name="auto_enable" value="$(arg auto_enable)" />
    <param name="gripper_val_mutiple" value="$(arg gripper_val_mutiple)" />
    <!-- <param name="rviz_ctrl_flag" value="true" /> -->
    <param name="girpper_exist" value="true" />
    <remap from="joint_ctrl_single" to="/joint_states" />
  </node>
</launch>
```

`start_single_piper_rviz.launch`:

```xml
<launch>
  <arg name="can_port" default="can0" />
  <arg name="auto_enable" default="true" />
  <include file="$(find piper_description)/launch/piper_with_gripper/display_xacro.launch"/>
  <!-- Start the robotic arm node -->
  <node name="piper_ctrl_single_node" pkg="piper" type="piper_ctrl_single_node.py" output="screen">
    <param name="can_port" value="$(arg can_port)" />
    <param name="auto_enable" value="$(arg auto_enable)" />
    <param name="gripper_val_mutiple" value="2" />
    <!-- <param name="rviz_ctrl_flag" value="true" /> -->
    <param name="girpper_exist" value="true" />
    <remap from="joint_ctrl_single" to="/joint_states" />
  </node>
</launch>
```

##### (1) Start the Control Node

There are several ways to start the same node:

```shell
# Start the node
roscore
rosrun piper piper_ctrl_single_node.py _can_port:=can0 _mode:=0
# Or, use launch to start the node
roslaunch piper start_single_piper.launch can_port:=can0 auto_enable:=true
# Or, directly run launch with default parameters
roslaunch piper start_single_piper.launch
# RViz can also be used to start control; the parameters to change are as mentioned above
roslaunch piper start_single_piper_rviz.launch
```

If only the control node is started, without RViz:

`rostopic list`

```shell
/arm_status # Robotic arm status, see below
/enable_flag # Enable flag sent to the node, send true to enable
/end_pose # Robotic arm end effector pose feedback in quaternion
/end_pose_euler # Robotic arm end effector pose feedback in Euler angles (custom message)
/joint_states # Subscribe to joint messages, sending joint positions will control the arm's movement
/joint_states_single # Robotic arm joint status feedback
/pos_cmd # End effector control messages
```

`rosservice list`

```shell
/enable_srv # Robotic arm enable service
/go_zero_srv # Robotic arm zeroing service
/gripper_srv # Robotic arm gripper control service
/reset_srv # Robotic arm reset service
/stop_srv # Robotic arm stop service
```

##### (2) Enable the Robotic Arm

```shell
# Call the service
rosservice call /enable_srv "enable_request: true"
# Publish to the topic
rostopic pub /enable_flag std_msgs/Bool "data: true"
```

##### (3) Disable the Robotic Arm

```shell
# Call the service
rosservice call /enable_srv "enable_request: false"
# Publish to the topic
rostopic pub /enable_flag std_msgs/Bool "data: false"
```

##### (4) Publish Joint Messages

Note that the robotic arm will lift, ensure there are no obstacles in the working range.

```shell
rostopic pub /joint_states sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['']
position: [0.2,0.2,-0.2,0.3,-0.2,0.5,0.01]
velocity: [0,0,0,0,0,0,10]
effort: [0,0,0,0,0,0,0.5]" 
```

##### (5) Stop the Robotic Arm (Note: It will fall with a constant damping)

```shell
rosservice call /stop_srv
```

##### (6) Reset the Robotic Arm (Note: It will immediately lose power and fall)

```shell
rosservice call /reset_srv
```

##### (7) Zero the Robotic Arm

- If the arm is in MIT mode, set `is_mit_mode` to `true`
- If the arm is not in MIT mode (position and velocity control mode), set `is_mit_mode` to `false`

```shell
rosservice call /go_zero_srv "is_mit_mode: false"
rosservice call /go_zero_srv "is_mit_mode: true"
```

### 2.3 Piper Custom Messages

ROS package `piper_msgs`

The robotic arm's own status feedback message, corresponding to the feedback message with `id=0x2A1` in the CAN protocol.
Description

`PiperStatusMsg.msg`

```c
uint8 ctrl_mode
/*
0x00 Standby Mode  
0x01 CAN Command Control Mode
0x02 Teach Mode
0x03 Ethernet Control Mode
0x04 WiFi Control Mode
0x05 Remote Control Mode
0x06 Interactive Teach Input Mode
0x07 Offline Trajectory Mode*/
uint8 arm_status
/*
0x00 Normal
0x01 Emergency Stop
0x02 No Solution
0x03 Singularity
0x04 Target Angle Exceeds Limit
0x05 Joint Communication Error
0x06 Joint Brake Not Released
0x07 Arm Collision
0x08 Exceed Speed During Teach Mode
0x09 Joint Status Error
0x0A Other Error  
0x0B Teach Record
0x0C Teach Execution
0x0D Teach Pause
0x0E Master Control NTC Over Temperature
0x0F Release Resistor NTC Over Temperature*/
uint8 mode_feedback
/*
0x00 MOVE P
0x01 MOVE J
0x02 MOVE L
0x03 MOVE C*/
uint8 teach_status
/*
0x00 Off
0x01 Start Teach Recording (Enter Drag Teach Mode)
0x02 End Teach Recording (Exit Drag Teach Mode)
0x03 Execute Teach Trajectory (Reproduce Drag Teach Trajectory)
0x04 Pause Execution
0x05 Continue Execution (Resume Trajectory Reproduction)
0x06 Terminate Execution
0x07 Move to Trajectory Start Point*/
uint8 motion_status
/*
0x00 Reached Target Position
0x01 Not Reached Target Position*/
uint8 trajectory_num
/*0~255 (Feedback in Offline Trajectory Mode)*/
int64 err_code // Fault code
bool joint_1_angle_limit // Joint 1 Communication Error (0: Normal, 1: Error)
bool joint_2_angle_limit // Joint 2 Communication Error (0: Normal, 1: Error)
bool joint_3_angle_limit // Joint 3 Communication Error (0: Normal, 1: Error)
bool joint_4_angle_limit // Joint 4 Communication Error (0: Normal, 1: Error)
bool joint_5_angle_limit // Joint 5 Communication Error (0: Normal, 1: Error)
bool joint_6_angle_limit // Joint 6 Communication Error (0: Normal, 1: Error)
bool communication_status_joint_1 // Joint 1 Angle Over Limit (0: Normal, 1: Error)
bool communication_status_joint_2 // Joint 2 Angle Over Limit (0: Normal, 1: Error)
bool communication_status_joint_3 // Joint 3 Angle Over Limit (0: Normal, 1: Error)
bool communication_status_joint_4 // Joint 4 Angle Over Limit (0: Normal, 1: Error)
bool communication_status_joint_5 // Joint 5 Angle Over Limit (0: Normal, 1: Error)
bool communication_status_joint_6 // Joint 6 Angle Over Limit (0: Normal, 1: Error)
```

End Effector Pose Control, Note: Some singularities cannot be reached.

`PosCmd.msg`

```c
// Units: meters
float64 x
float64 y
float64 z
// Units: radians
float64 roll
float64 pitch
float64 yaw
float64 gripper
// Temporarily invalid parameters
int32 mode1
int32 mode2
```

## 3 Attention

- You need to activate the CAN device and set the correct baud rate before reading or controlling the robotic arm.
- If you see:

  ```shell
  Enable Status: False
  <class 'can.exceptions.CanError'> Message NOT sent
  <class 'can.exceptions.CanError'> Message NOT sent
  ```

  It indicates that the robotic arm is not connected to the CAN module. After unplugging and re-plugging the USB, restart the robotic arm, re-activate the CAN module, and then try to restart the node again.

- If automatic enable is turned on, the program will automatically exit after 5 seconds if enable is not successful.

## Q&A

[Q&A](Q&A.MD)
