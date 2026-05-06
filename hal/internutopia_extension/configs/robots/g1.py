from typing import Optional

from internutopia.core.config import RobotCfg
from internutopia.macros import gm
from internutopia_extension.configs.controllers import (
    G1MoveBySpeedControllerCfg,
    MoveAlongPathPointsControllerCfg,
    MoveToPointBySpeedControllerCfg,
    RotateControllerCfg,
)

move_by_speed_cfg = G1MoveBySpeedControllerCfg(
    name='move_by_speed',
    policy_weights_path=gm.ASSET_PATH + '/robots/g1/policy/move_by_speed/g1_15000.onnx',
    joint_names=[
        'left_hip_pitch_joint',
        'right_hip_pitch_joint',
        'waist_yaw_joint',
        'left_hip_roll_joint',
        'right_hip_roll_joint',
        'left_hip_yaw_joint',
        'right_hip_yaw_joint',
        'left_knee_joint',
        'right_knee_joint',
        'left_shoulder_pitch_joint',
        'right_shoulder_pitch_joint',
        'left_ankle_pitch_joint',
        'right_ankle_pitch_joint',
        'left_shoulder_roll_joint',
        'right_shoulder_roll_joint',
        'left_ankle_roll_joint',
        'right_ankle_roll_joint',
        'left_shoulder_yaw_joint',
        'right_shoulder_yaw_joint',
        'left_elbow_joint',
        'right_elbow_joint',
        'left_wrist_roll_joint',
        'right_wrist_roll_joint',
        'left_wrist_pitch_joint',
        'right_wrist_pitch_joint',
        'left_wrist_yaw_joint',
        'right_wrist_yaw_joint',
    ],
)

move_to_point_cfg = MoveToPointBySpeedControllerCfg(
    name='move_to_point',
    forward_speed=1.0,
    rotation_speed=2.0,
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
    rotation_speed=4.0,
    threshold=0.1,
    sub_controllers=[move_by_speed_cfg],
)


class G1RobotCfg(RobotCfg):
    # meta info
    name: Optional[str] = 'g1'
    type: Optional[str] = 'G1Robot'
    prim_path: Optional[str] = '/g1'
    usd_path: Optional[str] = gm.ASSET_PATH + '/robots/g1/g1_29dof_color.usd'
