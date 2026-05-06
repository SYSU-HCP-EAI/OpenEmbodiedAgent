from typing import Optional

from internutopia.core.config import RobotCfg
from internutopia.macros import gm
from internutopia_extension.configs.controllers import (
    GR1MoveBySpeedControllerCfg,
    GR1TeleOpControllerCfg,
    MoveAlongPathPointsControllerCfg,
    MoveToPointBySpeedControllerCfg,
    RotateControllerCfg,
)
from internutopia_extension.configs.sensors import RepCameraCfg

# controllers

move_by_speed_cfg = GR1MoveBySpeedControllerCfg(
    name='move_by_speed',
    policy_weights_path=gm.ASSET_PATH + '/robots/gr1/policy/move_by_speed/gr1_policy.onnx',
    joint_names=[
        'left_hip_roll_joint',
        'left_hip_yaw_joint',
        'left_hip_pitch_joint',
        'left_knee_pitch_joint',
        'left_ankle_pitch_joint',
        'left_ankle_roll_joint',
        'right_hip_roll_joint',
        'right_hip_yaw_joint',
        'right_hip_pitch_joint',
        'right_knee_pitch_joint',
        'right_ankle_pitch_joint',
        'right_ankle_roll_joint',
        'waist_yaw_joint',
        'waist_pitch_joint',
        'waist_roll_joint',
        'head_roll_joint',
        'head_pitch_joint',
        'head_yaw_joint',
        'left_shoulder_pitch_joint',
        'left_shoulder_roll_joint',
        'left_shoulder_yaw_joint',
        'left_elbow_pitch_joint',
        'left_wrist_yaw_joint',
        'left_wrist_roll_joint',
        'left_wrist_pitch_joint',
        'L_index_proximal_joint',
        'L_index_intermediate_joint',
        'L_middle_proximal_joint',
        'L_middle_intermediate_joint',
        'L_pinky_proximal_joint',
        'L_pinky_intermediate_joint',
        'L_ring_proximal_joint',
        'L_ring_intermediate_joint',
        'L_thumb_proximal_yaw_joint',
        'L_thumb_proximal_pitch_joint',
        # "L_thumb_intermediate_joint",
        'L_thumb_distal_joint',
        'right_shoulder_pitch_joint',
        'right_shoulder_roll_joint',
        'right_shoulder_yaw_joint',
        'right_elbow_pitch_joint',
        'right_wrist_yaw_joint',
        'right_wrist_roll_joint',
        'right_wrist_pitch_joint',
        'R_index_proximal_joint',
        'R_index_intermediate_joint',
        'R_middle_proximal_joint',
        'R_middle_intermediate_joint',
        'R_pinky_proximal_joint',
        'R_pinky_intermediate_joint',
        'R_ring_proximal_joint',
        'R_ring_intermediate_joint',
        'R_thumb_proximal_yaw_joint',
        'R_thumb_proximal_pitch_joint',
        # "R_thumb_intermediate_joint",
        'R_thumb_distal_joint',
    ],
)

move_to_point_cfg = MoveToPointBySpeedControllerCfg(
    name='move_to_point',
    forward_speed=1.0,
    rotation_speed=4.0,
    threshold=0.05,
    sub_controllers=[move_by_speed_cfg],
)

teleop_cfg = GR1TeleOpControllerCfg(
    name='teleop',
    type='GR1TeleOpController',
    joint_names=[
        'left_hip_roll_joint',
        'right_hip_roll_joint',
        'waist_yaw_joint',
        'left_hip_yaw_joint',
        'right_hip_yaw_joint',
        'waist_pitch_joint',
        'left_hip_pitch_joint',
        'right_hip_pitch_joint',
        'waist_roll_joint',
        'left_knee_pitch_joint',
        'right_knee_pitch_joint',
        'left_ankle_pitch_joint',
        'right_ankle_pitch_joint',
        'head_roll_joint',
        'left_shoulder_pitch_joint',
        'right_shoulder_pitch_joint',
        'left_ankle_roll_joint',
        'right_ankle_roll_joint',
        'head_pitch_joint',
        'left_shoulder_roll_joint',
        'right_shoulder_roll_joint',
        'head_yaw_joint',
        'left_shoulder_yaw_joint',
        'right_shoulder_yaw_joint',
        'left_elbow_pitch_joint',
        'right_elbow_pitch_joint',
        'left_wrist_yaw_joint',
        'right_wrist_yaw_joint',
        'left_wrist_roll_joint',
        'right_wrist_roll_joint',
        'left_wrist_pitch_joint',
        'right_wrist_pitch_joint',
        'L_index_proximal_joint',
        'L_middle_proximal_joint',
        'L_pinky_proximal_joint',
        'L_ring_proximal_joint',
        'L_thumb_proximal_yaw_joint',
        'R_index_proximal_joint',
        'R_middle_proximal_joint',
        'R_pinky_proximal_joint',
        'R_ring_proximal_joint',
        'R_thumb_proximal_yaw_joint',
        'L_index_intermediate_joint',
        'L_middle_intermediate_joint',
        'L_pinky_intermediate_joint',
        'L_ring_intermediate_joint',
        'L_thumb_proximal_pitch_joint',
        'R_index_intermediate_joint',
        'R_middle_intermediate_joint',
        'R_pinky_intermediate_joint',
        'R_ring_intermediate_joint',
        'R_thumb_proximal_pitch_joint',
        'L_thumb_distal_joint',
        'R_thumb_distal_joint',
    ],
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
    rotation_speed=4.0,
    threshold=0.05,
    sub_controllers=[move_by_speed_cfg],
)

# sensors
camera_left_cfg = RepCameraCfg(
    name='camera_left',
    prim_path='head_yaw_link/CameraLeft',
    resolution=(1280, 720),
)

camera_right_cfg = RepCameraCfg(
    name='camera_right',
    prim_path='head_yaw_link/CameraRight',
    resolution=(1280, 720),
)


class GR1RobotCfg(RobotCfg):
    # meta info
    name: Optional[str] = 'gr1'
    type: Optional[str] = 'GR1Robot'
    prim_path: Optional[str] = '/gr1'
    usd_path: Optional[str] = gm.ASSET_PATH + '/robots/gr1/gr1.usd'
