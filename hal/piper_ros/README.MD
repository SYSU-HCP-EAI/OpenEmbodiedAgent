# Piper 机械臂ROS(noetic)

[EN](README(EN).md)

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

|说明 |文档|
|---|---|
|Moveit|[Moveit README](src/piper_moveit/README.md)|
|仿真|[仿真 README](src/piper_sim/README.md)|

## 0 注意URDF版本

S-V1.6-3固件版本以前的DH参数为各关节接触限位处为初始坐标得到

S-V1.6-3固件版本以后的DH参数将j2和j3的坐标系偏移了2度

现默认URDF为后者

|firmware version |urdf|
|---|---|
|current version < S-V1.6-3|`piper_description_old.urdf`|
|current version >= S-V1.6-3|`piper_description.urdf`|

## 1 安装方法

### 1.1 安装依赖

```shell
pip3 install python-can
```

```shell
pip3 install piper_sdk
```

```shell
source /opt/ros/noetic/setup.bash
sudo apt-get install python3-wstool python3-catkin-tools python3-rosdep ros-noetic-ruckig
sudo apt-get install ros-noetic-eigen-stl-containers ros-noetic-geometric-shapes ros-noetic-pybind11-catkin
sudo apt-get install ros-noetic-moveit-resources-panda-moveit-config ros-noetic-ompl ros-noetic-warehouse-ros ros-noetic-eigenpy ros-noetic-rosparam-shortcuts
sudo apt-get install ros-noetic-moveit-msgs ros-noetic-srdfdom
```

编译

注意：如果由于moveit编译报错无法解决，可以直接通过`sudo apt install ros-$ROS_DISTRO-moveit`来安装官方源的moveit，然后删掉`src/piper_moveit/moveit-1.1.11`这个目录，或者在该路径下创建名字为`CATKIN_IGNORE`的空文件即可让ros在编译期间忽略这个包，然后删掉`build/`和`devel/`路径重新编译。

```shell
cd piper_ros
catkin_make
```

## 2 快速使用

### 2.1 使能can模块

首先需要设置好shell脚本参数

#### 2.1.1 单条机械臂

##### 1）pc只插入一个usb转can模块

- **此处使用`can_activate.sh`脚本**

直接执行

```bash
bash can_activate.sh can0 1000000
```

##### 2）pc插入多个usb转can模块

- **此处使用`can_activate.sh`脚本**

拔掉所有can模块

只将连接到机械臂的can模块插入PC，执行

```shell
sudo ethtool -i can0 | grep bus
```

并记录下`bus-info`的数值例如`1-2:1.0`

ps:**一般第一个插入的can模块会默认是can0，如果没有查询到can可以使用`bash find_all_can_port.sh`来查看刚才usb地址对应的can名称**

假设上面的操作记录的`bus-info`数值为`1-2:1.0`

然后执行,查看can设备是否激活成功

```bash
bash can_activate.sh can_piper 1000000 "1-2:1.0"
```

ps:**此处的意思是，1-2:1.0硬件编码的usb端口插入的can设备，名字被重命名为can_piper,波特率为1000000，并激活**

然后执行`ifconfig`查看是否有`can_piper`，如果有则can模块设置成功

### 2.2 运行节点

#### 2.2.1 单机械臂

节点名`piper_ctrl_single_node.py`

param

```shell
can_port:要打开的can路由名字
auto_enable:是否自动使能，True则开启程序就自动使能
# 注意这个设置为False，中断程序后再启动节点，机械臂会保持上次启动程序的状态
# 若上次启动程序机械臂状态为使能，则中断程序后再启动节点，机械臂仍为使能
# 若上次启动程序机械臂状态为失能，则中断程序后再启动节点，机械臂仍为失能
girpper_exist:是否有末端夹爪，True则说明有末端夹爪，会开启夹爪控制
rviz_ctrl_flag:是否使用rviz来发送关节角消息，True则接收rviz发送的关节角消息
#由于rviz中的joint7范围是[0,0.04]，而真实夹爪行程为0.08m，打开rviz控制后会将rviz发出的joint7乘2倍
```

`start_single_piper.launch`如下:

```xml
<launch>
  <arg name="can_port" default="can0" />
  <arg name="auto_enable" default="true" />
  <arg name="gripper_val_mutiple" default="1" />
  <!-- <include file="$(find piper_description)/launch/display_xacro.launch"/> -->
  <!-- 启动机械臂节点 -->
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

`start_single_piper_rviz.launch`如下:

```xml
<launch>
  <arg name="can_port" default="can0" />
  <arg name="auto_enable" default="true" />
  <include file="$(find piper_description)/launch/piper_with_gripper/display_xacro.launch"/>
  <!-- 启动机械臂节点 -->
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

##### （1）启动控制节点

提供几种不同的启动方式，启动的都是相同节点

```shell
# 启动节点
roscore
rosrun piper piper_ctrl_single_node.py _can_port:=can0 _mode:=0
# 或，使用launch 启动节点
roslaunch piper start_single_piper.launch can_port:=can0 auto_enable:=true
# 或，直接运行launch，会以默认参数运行
roslaunch piper start_single_piper.launch
# 也可以用rviz开启控制,需要更改的参数如上述
roslaunch piper start_single_piper_rviz.launch
```

如果只是单独启动了控制节点，没有启动rviz

`rostopic list`

```shell
/arm_status #机械臂状态，详见下文
/enable_flag #使能标志位，发送给节点，发送true用来使能
/end_pose #机械臂末端位姿状态反馈，四元数
/end_pose_euler #机械臂末端位姿状态反馈，欧拉角(为自定义消息)
/joint_states #订阅关节消息，给这个消息发送关节位置能控制机械臂运动
/joint_states_single #机械臂关节状态反馈
/pos_cmd #末端控制消息
```

`rosservice list`

```shell
/enable_srv #机械臂使能服务端
/go_zero_srv #机械臂归零服务
/gripper_srv #机械臂夹爪控制服务
/reset_srv #机械臂重置服务
/stop_srv #机械臂停止服务
```

##### （2）使能机械臂

```shell
# call 服务端
rosservice call /enable_srv "enable_request: true"
# pub topic
rostopic pub /enable_flag std_msgs/Bool "data: true"
```

##### （3）失能机械臂

```shell
# call 服务端
rosservice call /enable_srv "enable_request: false"
# pub topic
rostopic pub /enable_flag std_msgs/Bool "data: false"
```

##### （4）发布关节消息

注意，机械臂会抬起，请确保机械臂工作范围内无障碍

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

##### （5）停止机械臂，注意机械臂会以一个恒定阻尼落下

```shell
rosservice call /stop_srv
```

##### （6）重置机械臂，注意机械臂会立刻掉电落下

```shell
rosservice call /reset_srv
```

##### （7）令机械臂归零

- 如果机械臂为mit模式，将`is_mit_mode`置为`true`
- 如果机械臂不为mit模式(位置速度控制模式)，将`is_mit_mode`置为`false`

```shell
rosservice call /go_zero_srv "is_mit_mode: false"
rosservice call /go_zero_srv "is_mit_mode: true"
```

### 2.3 piper自定义消息

ros功能包`piper_msgs`

机械臂自身状态反馈消息，对应can协议中`id=0x2A1`的反馈消息
说明

`PiperStatusMsg.msg`

```c
uint8 ctrl_mode
/*
0x00 待机模式  
0x01 CAN指令控制模式
0x02 示教模式
0x03 以太网控制模式
0x04 wifi控制模式
0x05 遥控器控制模式
0x06 联动示教输入模式
0x07 离线轨迹模式*/
uint8 arm_status
/*
0x00 正常
0x01 急停
0x02 无解
0x03 奇异点
0x04 目标角度超过限
0x05 关节通信异常
0x06 关节抱闸未打开 
0x07 机械臂发生碰撞
0x08 拖动示教时超速
0x09 关节状态异常
0x0A 其它异常  
0x0B 示教记录
0x0C 示教执行
0x0D 示教暂停
0x0E 主控NTC过温
0x0F 释放电阻NTC过温*/
uint8 mode_feedback
/*
0x00 MOVE P
0x01 MOVE J
0x02 MOVE L
0x03 MOVE C*/
uint8 teach_status
/*
0x00 关闭
0x01 开始示教记录（进入拖动示教模式）
0x02 结束示教记录（退出拖动示教模式）
0x03 执行示教轨迹（拖动示教轨迹复现）
0x04 暂停执行
0x05 继续执行（轨迹复现继续）
0x06 终止执行
0x07 运动到轨迹起点*/
uint8 motion_status
/*
0x00 到达指定点位
0x01 未到达指定点位*/
uint8 trajectory_num
/*0~255 (离线轨迹模式下反馈)*/
int64 err_code//故障码
bool joint_1_angle_limit//1号关节通信异常（0：正常 1：异常）
bool joint_2_angle_limit//2号关节通信异常（0：正常 1：异常）
bool joint_3_angle_limit//3号关节通信异常（0：正常 1：异常）
bool joint_4_angle_limit//4号关节通信异常（0：正常 1：异常）
bool joint_5_angle_limit//5号关节通信异常（0：正常 1：异常）
bool joint_6_angle_limit//6号关节通信异常（0：正常 1：异常）
bool communication_status_joint_1//1号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_2//2号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_3//3号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_4//4号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_5//5号关节角度超限位（0：正常 1：异常）
bool communication_status_joint_6//6号关节角度超限位（0：正常 1：异常）
```

机械臂末端位姿控制，注意：有些奇异点无法到达

`PosCmd.msg`

```c
// 单位：m
float64 x
float64 y
float64 z
// 单位：rad
float64 roll
float64 pitch
float64 yaw
float64 gripper
// 暂为无效参数
int32 mode1
int32 mode2
```

## 3 注意事项

- 需要先激活can设备，并且设置正确的波特率，才可以读取机械臂消息或者控制机械臂
- 如果出现

  ```shell
  使能状态: False
  <class 'can.exceptions.CanError'> Message NOT sent
  <class 'can.exceptions.CanError'> Message NOT sent
  ```

  说明机械臂没有与can模块连通，检查绿色端子是否紧连，拔插can模块usb，重新上电机械臂，再激活can模块，然后尝试重新启动节点

- 如果打开了自动使能，尝试使能5s没有成功后，会自动退出程序

## Q&A

[Q&A](Q&A.MD)
