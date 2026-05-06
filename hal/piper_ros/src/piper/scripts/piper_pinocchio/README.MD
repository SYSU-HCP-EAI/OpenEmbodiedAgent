# piper 匹诺曹逆解

## conda环境搭建

代码只使用了pinocchio=3.6.0=py39hfd66edf_0版本的pinocchio测试

```shell
conda create -n tv python=3.9
conda activate tv
conda install pinocchio=3.6.0 -c conda-forge
pip install meshcat
pip install casadi
```

## 运行

在一个单独终端中运行

```bash
roscore
```

另起新终端运行

运行的是piper单臂控制节点(也可以不执行这一步，单独运行逆解，这一步是用来控制真实机械臂的)

```bash
roslaunch piper start_single_piper.launch
```

直接使用conda的python运行文件

```bash
source /{你的路径}/piper_ros/devel/setup.sh
conda activate tv
python /{你的路径}/piper_ros/src/piper/scripts/piper_pinocchio/piper_pinocchio.py
```

运行成功后会自动弹出浏览器的可视化页面，并创建topic，这里只显示`piper_pinocchio`节点创建的topic

```bash
rostopic list
```

```bash
/joint_states #pub
/pin_pos_cmd #sub
```

查看`/pin_pos_cmd`详情

```bash
rostopic info /pin_pos_cmd
```

可以看到这个消息为自定义消息，被该节点所订阅，需要给该topic发送消息才能控制机械臂

```bash
Type: piper_msgs/PosCmd

Publishers: None

Subscribers: 
 * /inverse_solution_node_xxxxxxxxx
```

发布末端消息

```bash
source /{你的路径}/piper_ros/devel/setup.sh
# 发送零点
rostopic pub /pin_pos_cmd piper_msgs/PosCmd "{x: 0.056218, y: 0.0, z: 0.213266, roll: 0.0, pitch: 1.48352986, yaw: 0.0, gripper: 0.0, mode1: 0, mode2: 0}"
# 令z轴抬起0.1m
rostopic pub /pin_pos_cmd piper_msgs/PosCmd "{x: 0.056218, y: 0.0, z: 0.313266, roll: 0.0, pitch: 1.48352986, yaw: 0.0, gripper: 0.0, mode1: 0, mode2: 0}"
```

## Q&A

### 1

安装了`pinocchio`但是却出现

```bash
ImportError: cannot import name 'casadi' from 'pinocchio:
```

请先查看pinocchio版本

```bash
# 激活环境
conda activate vt
# 查询包
conda list | grep pinocchio
# 或
conda env export | grep pinocchio
```

该问题是由于casadi没有内置在pinocchio中导致

在其工程的changelog中记录到3.0.0之后的版本内置了casadi，但是3.7.0 drop了20.04的支持

<https://github.com/stack-of-tasks/pinocchio/blob/devel/CHANGELOG.md#300---2024-05-27>

使用以下指令查看可安装的pinocchio版本

```bash
conda search pinocchio -c conda-forge
```

查看是否有本文推荐的版本

### 2

运行会出现没有`rospkg`的问题(ros1-noetic)

```bash
conda activate tv
pip install rospkg
```

再运行节点即可
