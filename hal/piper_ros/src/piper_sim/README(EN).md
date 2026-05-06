# piper_sim

[中文](README.md)

![ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)

|ROS |STATE|
|---|---|
|![ros](https://img.shields.io/badge/ROS-noetic-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 1 Gazebo Simulation

### 1.1 Piper Gazebo Simulation (With Gripper)

Run the Gazebo simulation

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_gazebo piper_gazebo.launch
```

Control the gripper arm via RViz GUI (Run in a new terminal)

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_description display_gripper_urdf.launch
```

### 1.2 Piper Gazebo Simulation (Without Gripper)

Run the Gazebo simulation

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_gazebo piper_no_gripper_gazebo.launch
```

Control the arm without the gripper via RViz GUI (Run in a new terminal)

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_description display_no_gripper_urdf.launch
```

### Control the arm in Piper Gazebo through the following topics

The `/joint_states` topic controls multiple joints simultaneously, while other topics control individual joints.

Principle: The `joint_states_ctrl` node converts `/joint_states` control information into control information for each joint, such as `/piper_description/joint1_position_controller/command`, to control the arm in Gazebo simulation.
Note: Joint 7 directly controls both joint 7 and joint 8, so joint 8 is not involved in the control of `/joint_states`.

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

## 2 Mujoco Simulation

### 2.1 Installing Mujoco210 and mujoco-py

#### 2.1.1 Install Mujoco

1. [Download Mujoco210](https://github.com/google-deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz)

2. Extract the files

    ```bash
    mkdir ~/.mujoco
    cd (Directory containing the package)
    tar -zxvf mujoco210-linux-x86_64.tar.gz -C ~/.mujoco
    ```

3. Add the environment variable

    ```bash
    echo "export LD_LIBRARY_PATH=~/.mujoco/mujoco210/bin:\$LD_LIBRARY_PATH" >> ~/.bashrc
    source ~/.bashrc
    ```

4. Test the installation

    ```bash
    cd ~/.mujoco/mujoco210/bin
    ./simulate ../model/humanoid.xml
    ```

#### 2.1.2 Install mujoco-py

1. Download the source code

    ```bash
    git clone https://github.com/openai/mujoco-py.git
    ```

2. Install (This step can be done within a conda environment)

    ```bash
    cd ~/mujoco-py
    pip3 install -U 'mujoco-py<2.2,>=2.1'
    pip3 install -r requirements.txt
    pip3 install -r requirements.dev.txt
    python3 setup.py install
    sudo apt install libosmesa6-dev
    sudo apt install patchelf
    ```

3. Add the environment variable

    ```bash
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia" >> ~/.bashrc
    source ~/.bashrc
    ```

4. Test the installation

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

### 2.2 Piper Mujoco Simulation (With Gripper)

Run the Mujoco simulation

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_mujoco piper_mujoco.launch
```

Control the gripper arm via RViz GUI (Run in a new terminal)

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_description display_gripper_urdf.launch
```

### 2.3 Piper Mujoco Simulation (Without Gripper)

Run the Mujoco simulation

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_mujoco piper_no_gripper_mujoco.launch
```

Control the arm without the gripper via RViz GUI (Run in a new terminal)

```bash
cd piper_ros
source devel/setup.bash
```

```bash
roslaunch piper_description display_no_gripper_urdf.launch
```

### Control Parameter Explanation

[Control parameters for the gripper](../piper_description/mujoco_model/piper_description.xml)

[Control parameters for without gripper](../piper_description/mujoco_model/piper_no_gripper_description.xml)

- damping="100" Adjust joint damping

- kp="10000" Adjust joint control gain

- forcerange="-100 100" Adjust joint torque control (Set `forcelimited="false"` to `true`, then add `forcerange="-100 100"`)

[PID Control demo](piper_mujoco/scripts/piper_mujoco_pid.py)
