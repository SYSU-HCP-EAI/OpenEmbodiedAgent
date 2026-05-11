#!/usr/bin/env python

import rospy
from moveit_commander import *
from moveit_ctrl.srv import JointMoveitCtrl, JointMoveitCtrlResponse
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

class JointMoveitCtrlServer:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('joint_moveit_ctrl_server')

        # 初始化 MoveIt
        roscpp_initialize([])
        self.robot = RobotCommander()

        # 获取 MoveIt 规划组列表
        available_groups = self.robot.get_group_names()
        rospy.loginfo(f"Available MoveIt groups: {available_groups}")

        # 仅实例化存在的规划组
        self.arm_move_group = None
        self.gripper_move_group = None
        self.piper_move_group = None

        if "arm" in available_groups:
            self.arm_move_group = MoveGroupCommander("arm")
            rospy.loginfo("Initialized arm move group.")
        
        if "gripper" in available_groups:
            self.gripper_move_group = MoveGroupCommander("gripper")
            rospy.loginfo("Initialized gripper move group.")
        
        if "piper" in available_groups:
            self.piper_move_group = MoveGroupCommander("piper")
            rospy.loginfo("Initialized piper move group.")

        # 创建关节运动控制服务
        self.arm_srv = rospy.Service('joint_moveit_ctrl_arm', JointMoveitCtrl, self.handle_joint_moveit_ctrl_arm)
        self.gripper_srv = rospy.Service('joint_moveit_ctrl_gripper', JointMoveitCtrl, self.handle_joint_moveit_ctrl_gripper)
        self.piper_srv = rospy.Service('joint_moveit_ctrl_piper', JointMoveitCtrl, self.handle_joint_moveit_ctrl_piper)
        self.endpose_srv = rospy.Service('joint_moveit_ctrl_endpose', JointMoveitCtrl, self.handle_joint_moveit_ctrl_endpose)

        rospy.loginfo("Joint MoveIt Control Services Ready.")

    def handle_joint_moveit_ctrl_arm(self, request):
        rospy.loginfo("Received arm joint movement request.")

        try:
            if self.arm_move_group:
                arm_joint_goal = request.joint_states[:6]
                self.arm_move_group.set_joint_value_target(arm_joint_goal)
                max_velocity = max(1e-6, min(1-1e-6, request.max_velocity))
                max_acceleration = max(1e-6, min(1-1e-6, request.max_acceleration))
                self.arm_move_group.set_max_velocity_scaling_factor(max_velocity)
                self.arm_move_group.set_max_acceleration_scaling_factor(max_acceleration)
                rospy.loginfo(f"max_velocity: {max_velocity} max_acceleration: {max_acceleration}")
                self.arm_move_group.go(wait=True)
                rospy.loginfo("Arm movement executed successfully.")
            else:
                rospy.logerr("Arm move group is not initialized.")
        except Exception as e:
            rospy.logerr(f"Exception during arm movement: {str(e)}")

        return JointMoveitCtrlResponse(status=True, error_code=0)

    def handle_joint_moveit_ctrl_gripper(self, request):
        rospy.loginfo("Received gripper joint movement request.")

        try:
            if self.gripper_move_group:
                gripper_goal = [request.gripper]
                self.gripper_move_group.set_joint_value_target(gripper_goal)
                self.gripper_move_group.go(wait=True)
                rospy.loginfo("Gripper movement executed successfully.")
            else:
                rospy.logerr("Gripper move group is not initialized.")
        except Exception as e:
            rospy.logerr(f"Exception during gripper movement: {str(e)}")

        return JointMoveitCtrlResponse(status=True, error_code=0)

    def handle_joint_moveit_ctrl_piper(self, request):
        rospy.loginfo("Received piper joint movement request.")

        try:
            if self.piper_move_group:
                piper_joint_goal = list(request.joint_states[:6]) + [request.gripper]
                self.piper_move_group.set_joint_value_target(piper_joint_goal)
                max_velocity = max(1e-6, min(1-1e-6, request.max_velocity))
                max_acceleration = max(1e-6, min(1-1e-6, request.max_acceleration))
                self.piper_move_group.set_max_velocity_scaling_factor(max_velocity)
                self.piper_move_group.set_max_acceleration_scaling_factor(max_acceleration)
                rospy.loginfo(f"max_velocity: {max_velocity} max_acceleration: {max_acceleration}")
                self.piper_move_group.go(wait=True)
                rospy.loginfo("Piper movement executed successfully.")
            else:
                rospy.logerr("Piper move group is not initialized.")
        except Exception as e:
            rospy.logerr(f"Exception during piper movement: {str(e)}")
        
        return JointMoveitCtrlResponse(status=True, error_code=0)

    def handle_joint_moveit_ctrl_endpose(self, request):
        rospy.loginfo("Received endpose movement request.")

        try:
            if self.arm_move_group:
                position = request.joint_endpose[:3]
                if len(request.joint_endpose) == 7:
                    # 四元数 [qx, qy, qz, qw]
                    quaternion = request.joint_endpose[3:]
                    rospy.loginfo("Using Quaternion for orientation: (qx, qy, qz, qw) -> %f, %f, %f, %f", *quaternion)
                else:
                    rospy.logerr("Invalid joint_endpose size. It must be 7 (Quaternion).")
                    return JointMoveitCtrlResponse(status=False, error_code=1)
                
                target_pose = Pose()
                target_pose.position.x = position[0]
                target_pose.position.y = position[1]
                target_pose.position.z = position[2]
                target_pose.orientation.x = quaternion[0]
                target_pose.orientation.y = quaternion[1]
                target_pose.orientation.z = quaternion[2]
                target_pose.orientation.w = quaternion[3]

                self.arm_move_group.set_pose_target(target_pose)
                max_velocity = max(1e-6, min(1-1e-6, request.max_velocity))
                max_acceleration = max(1e-6, min(1-1e-6, request.max_acceleration))
                self.arm_move_group.set_max_velocity_scaling_factor(max_velocity)
                self.arm_move_group.set_max_acceleration_scaling_factor(max_acceleration)
                rospy.loginfo(f"max_velocity: {max_velocity} max_acceleration: {max_acceleration}")
                self.arm_move_group.go(wait=True)
                rospy.loginfo("Endpose movement executed successfully.")
            else:
                rospy.logerr("Arm move group is not initialized.")
        except Exception as e:
            rospy.logerr(f"Exception during endpose movement: {str(e)}")

        return JointMoveitCtrlResponse(status=True, error_code=0)

if __name__ == '__main__':
    JointMoveitCtrlServer()
    rospy.spin()
