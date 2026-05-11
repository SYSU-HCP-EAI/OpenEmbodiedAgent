#!/usr/bin/env python

import rospy
import time
import random
from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlRequest
from tf.transformations import quaternion_from_euler

def call_joint_moveit_ctrl_arm(joint_states, max_velocity=0.5, max_acceleration=0.5):
    rospy.wait_for_service("joint_moveit_ctrl_arm")
    try:
        moveit_service = rospy.ServiceProxy("joint_moveit_ctrl_arm", JointMoveitCtrl)
        request = JointMoveitCtrlRequest()
        request.joint_states = joint_states
        request.gripper = 0.0
        request.max_velocity = max_velocity
        request.max_acceleration = max_acceleration

        response = moveit_service(request)
        if response.status:
            rospy.loginfo("Successfully executed joint_moveit_ctrl_arm")
        else:
            rospy.logwarn(f"Failed to execute joint_moveit_ctrl_arm, error code: {response.error_code}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {str(e)}")

def call_joint_moveit_ctrl_gripper(gripper_position, max_velocity=0.5, max_acceleration=0.5):
    rospy.wait_for_service("joint_moveit_ctrl_gripper")
    try:
        moveit_service = rospy.ServiceProxy("joint_moveit_ctrl_gripper", JointMoveitCtrl)
        request = JointMoveitCtrlRequest()
        request.joint_states = [0.0] * 6
        request.gripper = gripper_position
        request.max_velocity = max_velocity
        request.max_acceleration = max_acceleration

        response = moveit_service(request)
        if response.status:
            rospy.loginfo("Successfully executed joint_moveit_ctrl_gripper")
        else:
            rospy.logwarn(f"Failed to execute joint_moveit_ctrl_gripper, error code: {response.error_code}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {str(e)}")

def call_joint_moveit_ctrl_piper(joint_states, gripper_position, max_velocity=0.5, max_acceleration=0.5):
    rospy.wait_for_service("joint_moveit_ctrl_piper")
    try:
        moveit_service = rospy.ServiceProxy("joint_moveit_ctrl_piper", JointMoveitCtrl)
        request = JointMoveitCtrlRequest()
        request.joint_states = joint_states
        request.gripper = gripper_position
        request.max_velocity = max_velocity
        request.max_acceleration = max_acceleration

        response = moveit_service(request)
        if response.status:
            rospy.loginfo("Successfully executed joint_moveit_ctrl_piper")
        else:
            rospy.logwarn(f"Failed to execute joint_moveit_ctrl_piper, error code: {response.error_code}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {str(e)}")

def convert_endpose(endpose):
    if len(endpose) == 6:
        x, y, z, roll, pitch, yaw = endpose
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        return [x, y, z, qx, qy, qz, qw]

    elif len(endpose) == 7:
        return endpose  # 直接返回四元数

    else:
        raise ValueError("Invalid endpose format! Must be 6 (Euler) or 7 (Quaternion) values.")

def call_joint_moveit_ctrl_endpose(endpose, max_velocity=0.5, max_acceleration=0.5):
    rospy.wait_for_service("joint_moveit_ctrl_endpose")
    try:
        moveit_service = rospy.ServiceProxy("joint_moveit_ctrl_endpose", JointMoveitCtrl)
        request = JointMoveitCtrlRequest()
        
        request.joint_states = [0.0] * 6  # 填充6个关节状态
        request.gripper = 0.0
        request.max_velocity = max_velocity
        request.max_acceleration = max_acceleration
        request.joint_endpose = convert_endpose(endpose)  # 自动转换

        response = moveit_service(request)
        if response.status:
            rospy.loginfo("Successfully executed joint_moveit_ctrl_endpose")
        else:
            rospy.logwarn(f"Failed to execute joint_moveit_ctrl_endpose, error code: {response.error_code}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {str(e)}")

# 此处关节限制仅为测试使用，实际关节限制以READEME中为准
def randomval():
    arm_position = [
        random.uniform(-0.2, 0.2),  # 关节1
        random.uniform(0, 0.5),  # 关节2
        random.uniform(-0.5, 0),  # 关节3
        random.uniform(-0.2, 0.2),  # 关节4
        random.uniform(-0.2, 0.2),  # 关节5
        random.uniform(-0.2, 0.2)   # 关节6
    ]
    gripper_position = random.uniform(0, 0.035)

    return arm_position, gripper_position

def main():
    rospy.init_node("test_joint_moveit_ctrl", anonymous=True)
    arm_position, gripper_position = [], 0
    for i in range(10): 
        # arm_position, _ = randomval()  # 机械臂控制
        # call_joint_moveit_ctrl_arm(arm_position, max_velocity=0.5, max_acceleration=0.5)
        # time.sleep(1)
        # _, gripper_position = randomval()  # 夹爪控制
        # call_joint_moveit_ctrl_gripper(gripper_position)
        # time.sleep(1)
        # arm_position, gripper_position = randomval()
        # call_joint_moveit_ctrl_piper(arm_position, gripper_position)  # 机械臂夹爪联合控制
        # time.sleep(1)
        # endpose_euler = [0.531014, -0.133376, 0.418909, -0.6052452780065936, 1.2265301318390152, -0.9107128036906411]
        # call_joint_moveit_ctrl_endpose(endpose_euler)  # 末端位置控制(欧拉角)
        # time.sleep(1)
        # endpose_quaternion = [0.531014, -0.133376, 0.418909, 0.02272779901175584, 0.6005891177332143, -0.18925185045722595, 0.7765049233012219]
        # call_joint_moveit_ctrl_endpose(endpose_quaternion)  # 末端位置控制(四元数)
        # time.sleep(1)
        arm_position = [0, 0, 0, 0, 0, 0]
        call_joint_moveit_ctrl_arm(arm_position, max_velocity=0.5, max_acceleration=0.5) # 回零
        time.sleep(1)
if __name__ == "__main__":
    main()
