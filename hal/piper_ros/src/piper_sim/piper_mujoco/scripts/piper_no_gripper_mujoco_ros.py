#!/usr/bin/env python3
# coding=utf-8

import rospy
import mujoco_py
import os
import time
import glfw
import subprocess
from std_msgs.msg import Header
from mujoco_py import MjSim, MjViewer
from mujoco_py import GlfwContext
from sensor_msgs.msg import JointState

# 如果你希望显示渲染窗口，可以注释掉这一行
# GlfwContext(offscreen=True)

class Mujoco_Model():
    def __init__(self):
        rospy.init_node("mujoco_piper_no_gripper_controller_node", anonymous=True)
        rospy.Subscriber("/mujoco_joint_states_ctrl", JointState, self.joint_state_callback)

        # 初始化 joint_targets 字典
        self.joint_targets = {}  # ← 添加这行，防止 AttributeError

        self.joint_state_pub = rospy.Publisher("/mujoco_joint_states_pub", JointState, queue_size=1,tcp_nodelay=True)

        package_path = subprocess.check_output('rospack find piper_description', shell=True).strip().decode('utf-8')
        model_path = os.path.join(package_path, 'mujoco_model', 'piper_no_gripper_description.xml')
        model_path = os.path.abspath(model_path)
        
        print("The model path is: ", model_path)

        model = mujoco_py.load_model_from_path(model_path)
        self.sim = MjSim(model)
        self.viewer = MjViewer(self.sim)


    def joint_state_callback(self, msg):
        """ 从 ROS /joint_states 话题获取关节角度 """
        for i, name in enumerate(msg.name):
            self.joint_targets[name] = msg.position[i]

    def pos_ctrl(self, joint_name, target_angle):
        """ 控制 MuJoCo 关节角度 """
        if joint_name not in self.sim.model.joint_names:
            rospy.logwarn(f"Joint {joint_name} not found in Mujoco model.")
            return

        try:
            joint_id = self.sim.model.get_joint_qpos_addr(joint_name)
            current_angle = self.sim.data.qpos[joint_id]
            actuator_id = self.sim.model.actuator_name2id(joint_name)
            self.sim.data.ctrl[actuator_id] = target_angle
        except Exception as e:
            rospy.logerr(f"Error controlling joint {joint_name}: {e}")

    def publish_joint_states(self):
        """发布当前 MuJoCo 关节状态"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.sim.model.joint_names
        msg.position = [self.sim.data.qpos[self.sim.model.get_joint_qpos_addr(j)] for j in msg.name]
        self.joint_state_pub.publish(msg)


    def control_loop(self):
        """ 让 MuJoCo 机械臂跟随 ROS 关节状态 """
        rate = rospy.Rate(100)  # 100Hz 控制循环
        tolerance = 0.05  # 角度误差容忍度

        while not rospy.is_shutdown() and not glfw.window_should_close(self.viewer.window):
            all_reached = True

            for joint, target_angle in self.joint_targets.items():
                
                if joint in self.sim.model.joint_names:
                    joint_id = self.sim.model.get_joint_qpos_addr(joint)
                    current_angle = self.sim.data.qpos[joint_id]

                    if abs(current_angle - target_angle) > tolerance:
                        all_reached = False  # 关节未达到目标

                    self.pos_ctrl(joint, target_angle)

            self.sim.step()
            self.viewer.render()
            self.publish_joint_states()
            rate.sleep()

def main():
    test = Mujoco_Model()
    test.control_loop()

if __name__ == "__main__":  
    main()
