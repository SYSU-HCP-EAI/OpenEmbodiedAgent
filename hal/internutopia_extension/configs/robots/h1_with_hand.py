from typing import Optional

from internutopia.core.config import RobotCfg
from internutopia.macros import gm
from internutopia_extension.configs.controllers import (
    H1MoveBySpeedControllerCfg,
    InverseKinematicsControllerCfg,
    JointControllerCfg,
    MoveAlongPathPointsControllerCfg,
    MoveToPointBySpeedControllerCfg,
    RecoverControllerCfg,
    RotateControllerCfg,
)
from internutopia_extension.configs.sensors import RepCameraCfg

joint_controller = JointControllerCfg(
    name='joint_controller',
    joint_names=[
        'left_hip_yaw_joint',
        'right_hip_yaw_joint',
        'torso_joint',
        'left_hip_roll_joint',
        'right_hip_roll_joint',
        'left_shoulder_pitch_joint',
        'right_shoulder_pitch_joint',
        'left_hip_pitch_joint',
        'right_hip_pitch_joint',
        'left_shoulder_roll_joint',
        'right_shoulder_roll_joint',
        'left_knee_joint',
        'right_knee_joint',
        'left_shoulder_yaw_joint',
        'right_shoulder_yaw_joint',
        'left_ankle_joint',
        'right_ankle_joint',
        'left_elbow_joint',
        'right_elbow_joint',
    ],
)

move_by_speed_cfg = H1MoveBySpeedControllerCfg(
    name='move_by_speed',
    policy_weights_path=gm.ASSET_PATH + '/robots/h1/policy/move_by_speed/h1_loco_model_20000.pt',
    joint_names=[
        'left_hip_yaw_joint',
        'right_hip_yaw_joint',
        'torso_joint',
        'left_hip_roll_joint',
        'right_hip_roll_joint',
        'left_shoulder_pitch_joint',
        'right_shoulder_pitch_joint',
        'left_hip_pitch_joint',
        'right_hip_pitch_joint',
        'left_shoulder_roll_joint',
        'right_shoulder_roll_joint',
        'left_knee_joint',
        'right_knee_joint',
        'left_shoulder_yaw_joint',
        'right_shoulder_yaw_joint',
        'left_ankle_joint',
        'right_ankle_joint',
        'left_elbow_joint',
        'right_elbow_joint',
    ],
)

move_to_point_cfg = MoveToPointBySpeedControllerCfg(
    name='move_to_point',
    forward_speed=1.0,
    rotation_speed=4.0,
    threshold=0.05,
    sub_controllers=[move_by_speed_cfg],
)

move_along_path_cfg = MoveAlongPathPointsControllerCfg(
    name='move_along_path',
    forward_speed=1.0,
    rotation_speed=4.0,
    threshold=0.1,
    sub_controllers=[move_to_point_cfg],
)

rotate_cfg = RotateControllerCfg(
    name='rotate',
    rotation_speed=2.0,
    threshold=0.05,
    sub_controllers=[move_by_speed_cfg],
)

recover_cfg = RecoverControllerCfg(
    name='recover',
    recover_height=1.0,
    sub_controllers=[joint_controller],
)

right_arm_joint_controller_cfg = JointControllerCfg(
    name='right_arm_joint_controller',
    joint_names=[
        'right_shoulder_pitch_joint',
        'right_shoulder_roll_joint',
        'right_shoulder_yaw_joint',
        'right_elbow_joint',
    ],
)

right_arm_ik_controller_cfg = InverseKinematicsControllerCfg(
    name='right_arm_ik_controller',
    robot_description_path=gm.ASSET_PATH + '/robots/h1_with_hand/right_arm_descriptor.yaml',
    robot_urdf_path=gm.ASSET_PATH + '/robots/h1_with_hand/h1_with_hand.urdf',
    end_effector_frame_name='right_hand_link',
    threshold=0.01,
)

h1_with_hand_camera_cfg = RepCameraCfg(name='camera', prim_path='logo_link/Camera', resolution=(640, 480), depth=True)

h1_with_hand_tp_camera_cfg = RepCameraCfg(
    name='tp_camera', prim_path='torso_link/TPCamera', resolution=(640, 480), depth=True
)


class H1WithHandRobotCfg(RobotCfg):
    # meta info
    name: Optional[str] = 'h1_with_hand'
    type: Optional[str] = 'H1WithHandRobot'
    prim_path: Optional[str] = '/h1_with_hand'
    usd_path: Optional[str] = gm.ASSET_PATH + '/robots/h1_with_hand/h1_with_hand_rt_v2.usd'
