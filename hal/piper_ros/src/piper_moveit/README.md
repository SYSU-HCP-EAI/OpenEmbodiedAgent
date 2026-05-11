# Piper_Moveit

[EN](README(EN).md)

![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-noetic-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 1 安装Moveit环境

> 注:moveit 1.1.11包含在src中，无需单独下载，只是包moveit-1.1.11路径下存放了CATKIN_IGNORE文件，默认不会编译它
> 如果解决moveit的编译问题太多，可以直接通过`sudo apt install ros-$ROS_DISTRO-moveit`来安装官方源的moeit

源安装需要 wstool,catkin_tools:

```bash
sudo apt install python3-wstool python3-catkin-tools python3-rosdep
```

## 2 使用方法

### 2.1 运行moveit

进入工作空间

```bash
cd ~/piper_ros
source devel/setup.bash
```

#### 2.1.1 运行(有夹爪)

开启ros控制节点(夹爪控制值二倍)

```bash
roslaunch piper start_single_piper.launch gripper_val_mutiple:=2
```

>出现使能成功即可

运行moveit

```bash
roslaunch piper_with_gripper_moveit demo.launch
```

若希望不启动rviz,运行

```bash
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false
```

>夹爪模式分为两个控制组:
>
>- **机械臂控制组** 包含关节 joint1 至 joint6
>- **夹爪控制组** 包含关节 joint7 和 joint8, 夹爪控制组采用 joint7 进行主动控制,而 joint8 为被动控制
>- **piper控制组** 包含关节 joint1 和 joint6, joint7为夹爪控制
> 夹爪控制值范围为0到0.035, 单位为m, 对应到实际夹爪张合距离需乘2,即0到0.07

|joint_name|     limit(rad)     |    limit(angle)    |     limit(rad/s)   |   limit(rad/s^2)   |
|----------|     ----------     |     ----------     |     ----------     |     ----------     |
|joint1    |   [-2.618, 2.618]  |    [-150.0, 150.0] |      [0, 3.0]      |      [0, 5.0]      |
|joint2    |   [0, 3.14]        |    [0, 180.0]      |      [0, 3.0]      |      [0, 5.0]      |
|joint3    |   [-2.967, 0]      |    [-170, 0]       |      [0, 3.0]      |      [0, 5.0]      |
|joint4    |   [-1.745, 1.745]  |    [-100.0, 100.0] |      [0, 3.0]      |      [0, 5.0]      |
|joint5    |   [-1.22, 1.22]    |    [-70.0, 70.0]   |      [0, 3.0]      |      [0, 5.0]      |
|joint6    |   [-2.0944, 2.0944]|    [-120.0, 120.0] |      [0, 3.0]      |      [0, 5.0]      |

控制信息节点为/joint_states

```bash
rostopic echo /joint_states
```

>- 其中前6个值为机械臂位置控制
>- 第7个值为夹爪位置控制
>- 第8个值为0不参与控制

#### 2.1.2 运行(无夹爪)

开启ros控制节点(夹爪控制值二倍)

```bash
roslaunch piper start_single_piper.launch gripper_val_mutiple:=2
```

>出现使能成功即可

```bash
roslaunch piper_no_gripper_moveit demo.launch
```

若希望不启动rviz,运行

```bash
roslaunch piper_no_gripper_moveit demo.launch use_rviz:=false
```

### 2.2 规划轨迹并运动

#### 2.2.1 拖动示教

![piper_moveit](../../asserts/pictures/piper_moveit.png)

调整好位置后点击左侧MotionPlanning中Planning的Plan&Execute即可开始规划并运动

#### 2.2.2 服务端控制(关节弧度控制)

控制机械臂 (终端输入)

```bash
cd piper_ros
source devel/setup.bash
```

机械臂关节弧度控制

```bash
rosservice call /joint_moveit_ctrl_arm "joint_states: [0.2,0.2,-0.2,0.3,-0.2,0.5]
max_velocity: 0.5
max_acceleration: 0.5" 

```

机械臂末端位置控制

```bash
rosservice call /joint_moveit_ctrl_endpose "joint_endpose: [0.099091, 0.008422, 0.246447, -0.09079689034052749, 0.7663049838381912, -0.02157924359457128, 0.6356625934370577]
max_velocity: 0.5
max_acceleration: 0.5" 

```

控制夹爪 (终端输入)

```bash
rosservice call /joint_moveit_ctrl_gripper "gripper: 0.035
max_velocity: 0.5
max_acceleration: 0.5" 
```

控制机械臂和夹爪联合运动

```bash
rosservice call /joint_moveit_ctrl_piper "joint_states: [0.2,0.2,-0.2,0.3,-0.2,0.5]
gripper: 0.035
max_velocity: 0.5
max_acceleration: 0.5" 
```

#### 2.2.3 客户端控制 (终端输入)

```bash
cd piper_ros
source devel/setup.bash
rosrun moveit_ctrl joint_moveit_ctrl.py
```

> 速度和加速度使用百分比控速,范围为(0-1),默认为0.5,机械臂最大速度为3(rad/s).可在[joint_moveit_ctrl](../piper_moveit/moveit_ctrl/scripts/joint_moveit_ctrl.py)中的控制函数中的max_velocity=0.5, max_acceleration=0.5参数更改
> 更改 [joint_moveit_ctrl](../piper_moveit/moveit_ctrl/scripts/joint_moveit_ctrl.py)中的 arm_position, gripper_position 控制关节运动,单位为弧度

#### 2.2.4 moveit类控制 (关节弧度控制)

> 此部分可加在 [joint_moveit_ctrl](../piper_moveit/moveit_ctrl/scripts/joint_moveit_ctrl.py) 中应用

```python
#!/usr/bin/env python3

import rospy
import moveit_commander

def move_robot():
    # 初始化 MoveIt! 相关组件
    moveit_commander.roscpp_initialize([])
    move_group = moveit_commander.MoveGroupCommander("gripper")  # 可以根据需要修改为 "arm"、"gripper"或"piper"

    # 获取当前关节值
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.0  # arm使用joint_goal长度为6,gripper使用joint_goal长度为1,piper使用joint_goal长度为7
    # joint_goal[1] = 0.0
    # joint_goal[2] = 0.0
    # joint_goal[3] = 0.0
    # joint_goal[4] = 0.0
    # joint_goal[5] = 0.0
    # joint_goal[6] = 0.0 # 使用piper规划组是，这一位是夹爪控制

    # 设置并执行目标
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

## 3 moveit控制仿真机械臂

注：**有无夹爪版本需要对应**

### 3.1 gazebo

#### 3.1.1 运行gazebo

见 [piper_gazebo](../piper_sim/README.md#21-gazebo仿真)

#### 4.1.2 moveit控制

同 [2.2 运行moveit](#22-运行moveit)

### 4.2 mujoco

#### 4.2.1 运行mujoco

见 [piper_mujoco](../piper_sim/README.md#22-mujoco仿真)

#### 4.2.2 moveit控制

同 [2.2 运行moveit](#22-运行moveit)
