#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# 关节列表
joint_names = [
    "joint1", "joint2", "joint3", "joint4",
    "joint5", "joint6", "joint7", "joint8"
]

# 预创建 Publisher
publishers = {
    name: rospy.Publisher(f"/gazebo/{name}_position_controller/command", Float64, queue_size=10)
    for name in joint_names
}

# 缓存上一次的关节位置
last_positions = {name: None for name in joint_names}

# 处理 /joint_states 话题的回调函数
def joint_state_callback(msg):
    global last_positions
    joint_positions = dict(zip(msg.name, msg.position))  # 关节名 -> 位置

    for joint_name in joint_names:
        # 直接处理 joint8，设置它的值为 joint7 的负值
        if joint_name == "joint8":
            if "joint7" in joint_positions:
                joint_positions["joint8"] = -joint_positions["joint7"]
            else:
                joint_positions["joint8"] = 0  # 如果没有joint7的位置，可以默认给它0
        
        # 发布其他关节的值
        if joint_name in joint_positions:
            position = joint_positions[joint_name]

            # 仅在关节位置发生变化时发布
            if last_positions[joint_name] is None or abs(position - last_positions[joint_name]) > 1e-5:
                publishers[joint_name].publish(Float64(position))
                last_positions[joint_name] = position  # 更新缓存

def main():
    rospy.init_node('joint_states_ctrl', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, joint_state_callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    main()
