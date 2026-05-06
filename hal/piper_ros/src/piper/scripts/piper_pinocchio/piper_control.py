#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from piper_msgs.msg import PosCmd
from std_msgs.msg import Header

# float64 x
# float64 y
# float64 z
# float64 roll
# float64 pitch
# float64 yaw
# float64 gripper    # 单位：米    范围：0 ~ 0.08米
# int32 mode1
# int32 mode2

class PIPER:
    def __init__(self):
        # 发布控制piper机械臂话题
        self.pub_joint = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # self.rate = rospy.Rate(80) # 10hz
    
    def joint_control_piper(self,j1,j2,j3,j4,j5,j6,gripper):
        joint_states_msgs = JointState()
        joint_states_msgs.header = Header()
        joint_states_msgs.header.stamp = rospy.Time.now()
        joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
        joint_states_msgs.position.append(j1)
        joint_states_msgs.position.append(j2)
        joint_states_msgs.position.append(j3)
        joint_states_msgs.position.append(j4)
        joint_states_msgs.position.append(j5)
        joint_states_msgs.position.append(j6)
        joint_states_msgs.position.append(gripper)
        self.pub_joint.publish(joint_states_msgs)
        # self.rate.sleep()
        print("send joint control piper command")
    
    # 使用线性插值实现平滑过渡到初始位置
    def init_pose(self):
        # 目标关节位置
        target_joint_state = self.target_joint_state
        
        # 获取当前关节位置
        # 如果已经接收到关节位置数据，使用实际的当前位置
        # 否则会一步调整到位
        if self.joint_positions_received:
            current_positions = self.current_joint_positions
            rospy.loginfo(f"使用实际的当前关节位置: {current_positions}")
            
            # 设置过渡时间和控制频率
            duration = 0.5  # 过渡持续时间(秒)
            rate = 50  # 控制频率(Hz)
            
            # 计算总步数
            steps = int(duration * rate)
            
            # 计算每一步的增量
            increments = [(target - current) / steps for current, target in zip(current_positions, target_joint_state)]
            
            # 创建ROS的Rate对象控制循环频率
            rate_obj = rospy.Rate(rate)
            
            # 记录开始时间（用于日志）
            start_time = rospy.Time.now()
            
            # 逐步移动到目标位置
            for step in range(steps + 1):
                # 计算当前步骤的位置
                interpolated_positions = [current + increment * step for current, increment in zip(current_positions, increments)]
                
                # 发布关节状态消息
                joint_states_msgs = JointState()
                joint_states_msgs.header = Header()
                joint_states_msgs.header.stamp = rospy.Time.now()
                joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
                joint_states_msgs.position = interpolated_positions
                
                # 发布消息
                self.arm_joint_state_publisher.publish(joint_states_msgs)
                
                # 按照指定频率控制循环
                rate_obj.sleep()
            
            # 确保最后一帧是精确的目标位置
            joint_states_msgs = JointState()
            joint_states_msgs.header = Header()
            joint_states_msgs.header.stamp = rospy.Time.now()
            joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
            joint_states_msgs.position = target_joint_state
            self.arm_joint_state_publisher.publish(joint_states_msgs)
            
            # 计算实际用时
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            # print(f"平滑移动到初始位置完成，用时: {elapsed_time:.2f}秒")
            
        else:
            start_time = rospy.Time.now()  # 获取当前时间
            while (rospy.Time.now() - start_time).to_sec() < 0.5:  # 持续发送0.5秒
                joint_states_msgs = JointState()
                joint_states_msgs.header = Header()
                joint_states_msgs.header.stamp = rospy.Time.now()
                joint_states_msgs.name = [f'joint{i+1}' for i in range(7)]
                joint_states_msgs.position = target_joint_state
                self.arm_joint_state_publisher.publish(joint_states_msgs)
            # print("send joint control piper command for 2 seconds")
            # 使用默认的非零位置作为起始点
            # current_positions = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            # rospy.loginfo("未接收到当前关节位置，使用默认初始位置")
     
# test code
if __name__ == '__main__':
    # piper = PIPER() 
    rospy.init_node('control_piper_node', anonymous=True)
    # piper.control_piper(0.0,0.0,0.0,0.0,0.0,0.0,0.05)
    # 保持节点运行并监听外部程序的调用
    rospy.spin()
