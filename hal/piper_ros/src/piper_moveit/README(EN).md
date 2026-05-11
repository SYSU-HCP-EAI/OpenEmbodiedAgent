# Piper_Moveit

[中文](README.md)

![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-noetic-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 1 Install Moveit Environment

> Note: Moveit 1.1.11 is included in the src, no need to download separately.

Source installation requires wstool and catkin_tools:

```bash
sudo apt install python3-wstool python3-catkin-tools python3-rosdep
```

## 2 Usage`

### 2.1 Run Moveit

Enter the workspace:

```bash
cd ~/piper_ros
source devel/setup.bash
```

#### 2.1.1 Run (with gripper)

Start the ROS control node (Gripper control value doubled):

```bash
roslaunch piper start_single_piper.launch gripper_val_mutiple:=2
```

> The process is successful when the enable confirmation appears.

Start the moveit

```bash
roslaunch piper_with_gripper_moveit demo.launch
```

If you don't want to start rviz, run:

```bash
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false
```

> The gripper mode is divided into two control groups:
>
>- **Arm control group** contains joints joint1 to joint6
>- **Gripper control group** contains joints joint7 and joint8. The gripper control group uses joint7 for active control, and joint8 for passive control
>- **Piper control group** contains joints joint1 and joint6, joint7 for gripper control
> The gripper control range is from 0 to 0.035 meters. For the actual gripper open/close distance, multiply by 2, i.e., 0 to 0.07.

|joint_name|     limit(rad)     |    limit(angle)    |     limit(rad/s)   |   limit(rad/s^2)   |
|----------|     ----------     |     ----------     |     ----------     |     ----------     |
|joint1    |   [-2.618, 2.618]  |    [-150.0, 150.0] |      [0, 3.0]      |      [0, 5.0]      |
|joint2    |   [0, 3.14]        |    [0, 180.0]      |      [0, 3.0]      |      [0, 5.0]      |
|joint3    |   [-2.967, 0]      |    [-170, 0]       |      [0, 3.0]      |      [0, 5.0]      |
|joint4    |   [-1.745, 1.745]  |    [-100.0, 100.0] |      [0, 3.0]      |      [0, 5.0]      |
|joint5    |   [-1.22, 1.22]    |    [-70.0, 70.0]   |      [0, 3.0]      |      [0, 5.0]      |
|joint6    |   [-2.0944, 2.0944]|    [-120.0, 120.0] |      [0, 3.0]      |      [0, 5.0]      |

The control information node is /joint_states:

```bash
rostopic echo /joint_states
```

>- The first 6 values are for arm position control
>- The 7th value is for gripper position control
>- The 8th value is 0 and does not participate in control

#### 2.2.2 Run (without gripper)

Start the ROS control node (Gripper control value doubled):

```bash
roslaunch piper start_single_piper.launch gripper_val_mutiple:=2
```

> The process is successful when the enable confirmation appears.

Start the moveit

```bash
roslaunch piper_no_gripper_moveit demo.launch
```

If you don't want to start rviz, run:

```bash
roslaunch piper_no_gripper_moveit demo.launch use_rviz:=false
```

### 2.3 Plan Trajectory and Move

#### 2.3.1 Teach by dragging

![piper_moveit](../../asserts/pictures/piper_moveit.png)

After adjusting the position, click "Plan & Execute" in the left "MotionPlanning" panel to start planning and moving.

#### 2.3.2 Server-side Control (Joint Angle Control)

Control the arm (terminal input):

```bash
cd piper_ros
source devel/setup.bash
```

Arm joint angle control:

```bash
rosservice call /joint_moveit_ctrl_arm "joint_states: [0.2,0.2,-0.2,0.3,-0.2,0.5]
max_velocity: 0.5
max_acceleration: 0.5" 
```

Arm end pose control:

```bash
rosservice call /joint_moveit_ctrl_endpose "joint_endpose: [0.099091, 0.008422, 0.246447, -0.09079689034052749, 0.7663049838381912, -0.02157924359457128, 0.6356625934370577]
max_velocity: 0.5
max_acceleration: 0.5" 
```

Gripper control:

```bash
rosservice call /joint_moveit_ctrl_gripper "gripper: 0.035
max_velocity: 0.5
max_acceleration: 0.5" 
```

Control the arm and gripper in joint motion:

```bash
rosservice call /joint_moveit_ctrl_piper "joint_states: [0.2,0.2,-0.2,0.3,-0.2,0.5]
gripper: 0.035
max_velocity: 0.5
max_acceleration: 0.5" 
```

#### 2.3.3 Client-side Control (terminal input)

```bash
cd piper_ros
source devel/setup.bash
rosrun moveit_ctrl joint_moveit_ctrl.py
```

> Speed and acceleration are controlled using percentage speed, ranging from 0 to 1, with the default value being 0.5. The maximum arm speed is 3(rad/s). You can change the `max_velocity=0.5`, `max_acceleration=0.5` parameters in the [joint_moveit_ctrl](../piper_moveit/moveit_ctrl/scripts/joint_moveit_ctrl.py) control function.
> Modify `arm_position`, `gripper_position` in [joint_moveit_ctrl](../piper_moveit/moveit_ctrl/scripts/joint_moveit_ctrl.py) to control joint movement, in radians.

#### 2.3.4 Moveit Class Control (Joint Angle Control)

> This part can be added in the [joint_moveit_ctrl](../piper_moveit/moveit_ctrl/scripts/joint_moveit_ctrl.py) and applied.

```python
#!/usr/bin/env python3

import rospy
import moveit_commander

def move_robot():
    # Initialize MoveIt! components
    moveit_commander.roscpp_initialize([])
    move_group = moveit_commander.MoveGroupCommander("gripper")  # Modify to "arm", "gripper", or "piper" as needed

    # Get current joint values
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.0  # For arm, joint_goal has length 6, for gripper, it has length 1, for piper, it has length 7
    # joint_goal[1] = 0.0
    # joint_goal[2] = 0.0
    # joint_goal[3] = 0.0
    # joint_goal[4] = 0.0
    # joint_goal[5] = 0.0
    # joint_goal[6] = 0.0 # For Piper planning group, this is for gripper control

    # Set and execute the target
    move_group.set_joint_value_target(joint_goal)
    success = move_group.go(wait=True)
    rospy.loginfo(f"Movement success: {success}")
    rospy.loginfo(f"joint_value: {move_group.get_current_joint_values()}")

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
```

## 3 Moveit Control for Simulated Arm

Note: **The gripper version must correspond.**

### 3.1 Gazebo

#### 3.1.1 Run Gazebo

See [piper_gazebo](../piper_sim/README(EN).md#21-gazebo-simulation)

#### 3.1.2 Moveit Control

Same as [2.2 Run Moveit](#22-run-moveit)

### 3.2 Mujoco

#### 3.2.1 Run Mujoco

See [piper_mujoco](../piper_sim/README(EN).md#22-mujoco-simulation)

#### 3.2.2 Moveit Control

Same as [2.2 Run Moveit](#22-run-moveit)
