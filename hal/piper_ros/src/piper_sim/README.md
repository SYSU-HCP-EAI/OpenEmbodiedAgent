# piper_sim

[EN](README(EN).md)

![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-noetic-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 1 gazebo仿真

### 1.1 piper gazebo仿真(有夹爪)

gazebo仿真运行

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_gazebo piper_gazebo.launch
```

通过rviz_gui控制有夹爪机械臂(新终端运行)

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_description display_urdf.launch
```

### 1.2 piper gazebo仿真(无夹爪)

gazebo仿真运行

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_gazebo piper_no_gripper_gazebo.launch
```

通过rviz_gui控制无夹爪机械臂(新终端运行)

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_description display_no_gripper_urdf.launch
```

### piper_gazebo中,可以通过以下话题控制机械臂运动

其中/joint_states为多关节同时控制，其他话题直接控制单关节

原理：通过joint_states_ctrl节点将/joint_states控制信息转换为/piper_description/joint1_position_controller/command等八个关节的控制信息，实现gazebo仿真机械臂控制
注：/joint_states的joint7直接联动控制joint7和joint8，所以/joint_states的joint8不参与控制

```text
/joint_states
/gazebo/joint1_position_controller/command
/gazebo/joint2_position_controller/command
/gazebo/joint3_position_controller/command
/gazebo/joint4_position_controller/command
/gazebo/joint5_position_controller/command
/gazebo/joint6_position_controller/command
/gazebo/joint7_position_controller/command
/gazebo/joint8_position_controller/command
```

## 2 mujoco仿真

### 2.1 mujoco210和mujoco-py的安装

#### 2.1.1 安装mujoco

1、[下载mujoco210](https://github.com/google-deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz)

2、解压

```bash
mkdir ~/.mujoco
cd (压缩包所在目录)
tar -zxvf mujoco210-linux-x86_64.tar.gz -C ~/.mujoco
```

3、添加环境变量

```bash
echo "export LD_LIBRARY_PATH=~/.mujoco/mujoco210/bin:\$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc
```

4、测试

```bash
cd ~/.mujoco/mujoco210/bin
./simulate ../model/humanoid.xml
```

#### 2.1.2 安装mujoco-py

1、下载源码

```bash
git clone https://github.com/openai/mujoco-py.git
```

2、安装(这一步可以在conda环境中进行)

```bash
cd ~/mujoco-py
pip3 install -U 'mujoco-py<2.2,>=2.1'
pip3 install -r requirements.txt
pip3 install -r requirements.dev.txt
python3 setup.py install
sudo apt install libosmesa6-dev
sudo apt install patchelf
```

3、添加环境变量

```bash
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia" >> ~/.bashrc
source ~/.bashrc
```

4、测试

```python
import mujoco_py
import os
mj_path = mujoco_py.utils.discover_mujoco()
xml_path = os.path.join(mj_path, 'model', 'humanoid.xml')
model = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(model)
print(sim.data.qpos)
sim.step()
print(sim.data.qpos)
```

### 2.2 piper mujoco仿真(有夹爪)

mujoco仿真运行

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_mujoco piper_mujoco.launch
```

通过rviz_gui控制有夹爪机械臂(新终端运行)

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_description display_gripper_urdf.launch
```

### 2.3 piper mujoco仿真(无夹爪)

mujoco仿真运行

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_mujoco piper_no_gripper_mujoco.launch
```

通过rviz_gui控制无夹爪机械臂(新终端运行)

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_description display_no_gripper_urdf.launch
```

### 控制参数介绍

[有夹爪控制参数](../piper_description/mujoco_model/piper_description.xml)

[无夹爪控制参数](../piper_description/mujoco_model/piper_no_gripper_description.xml)

- damping="100 更改关节阻尼

- kp="10000" 更改关节控制增益

- forcerange="-100 100" 更改关节控制力矩 (将forcelimited="false"调整为true 然后在后面添加forcerange="-100 100")

[PID控制demo](piper_mujoco/scripts/piper_mujoco_pid.py)
