import math
import time
import mujoco_py
from mujoco_py import load_model_from_path, MjSim, MjViewer
import glfw  # 用于检查窗口关闭事件
import os

# 加载模型
current_path = os.path.dirname(os.path.realpath(__file__))
model_path = os.path.join(current_path, '..', '..', 'piper_description', 'mujoco_model', 'piper_description.xml')
model = load_model_from_path(model_path)
sim = MjSim(model)
viewer = MjViewer(sim)

# 定义PID控制器类
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_error = 0
        self.prev_error = 0
        self.dt = 0.01  # 控制周期，需要与模拟步长相匹配

    def calculate(self, target, current):
        error = target - current
        self.integral_error += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = (self.Kp * error) + (self.Ki * self.integral_error) + (self.Kd * derivative)
        self.prev_error = error
        return output

# 定义六自由度机械臂角度控制类
class ArmControl:
    def __init__(self, sim):
        self.sim = sim
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]  # 假设机械臂有6个关节
        self.pid_controllers = [PIDController(1.1, 0, 0) for _ in range(6)]  # 为每个关节创建PID控制器
        self.target_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 目标角度

    def set_target_angles(self, target_angles):
        self.target_angles = target_angles  # 设置目标角度

    def get_current_angles(self):
        current_angles = []
        for joint_name in self.joint_names:
            joint_id = self.sim.model.joint_name2id(joint_name)
            current_angles.append(self.sim.data.qpos[joint_id])  # 获取每个关节的当前角度
        return current_angles

    def control(self):
        current_angles = self.get_current_angles()
        control_signals = []
        for i in range(6):
            control_signal = self.pid_controllers[i].calculate(self.target_angles[i], current_angles[i])
            control_signals.append(control_signal)
            joint_id = self.sim.model.joint_name2id(self.joint_names[i])
            self.sim.data.ctrl[joint_id] = control_signal  # 通过控制信号调整关节角度
        return control_signals

# 主循环
arm_control = ArmControl(sim)

# 设置目标角度



while True:
    # sim.step()  # 模拟一步

    # target_angles = [0.5, 0.5, -0.5, 0.5, -0.5, 0.5]  # 你可以设置任何期望的角度
    # arm_control.set_target_angles(target_angles)
    # arm_control.control()  # 控制每个关节的角度

    count = 0
    while 1:
        if(count > 0 and count < 300):
            target_angles = [0.5, 0.5, -0.5, 0.5, -0.5, 0.5]
            arm_control.set_target_angles(target_angles)
            arm_control.control()
        if(count > 200 and count < 400):
            target_angles = [0.5, 0.5, -0.5, 0.5, -0.5, 0.5]
            arm_control.set_target_angles(target_angles)
            arm_control.control()
        if(count > 400):
            count = 0
        sim.step()
        viewer.render()
        time.sleep(0.01)
        count = count + 1
    
    viewer.render()  # 渲染视图
    if glfw.window_should_close(viewer.window):
        break  # 如果窗口关闭则退出循环

    time.sleep(0.01)  # 控制周期
