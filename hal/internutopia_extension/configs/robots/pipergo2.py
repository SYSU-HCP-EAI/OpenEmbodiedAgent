from pathlib import Path
from typing import Optional

from internutopia.core.config import RobotCfg
from internutopia.macros import gm
from internutopia_extension.configs.controllers import (
    AliengoMoveBySpeedControllerCfg,
    MoveAlongPathPointsControllerCfg,
    MoveToPointBySpeedControllerCfg,
    RotateControllerCfg,
)

_REPO_ROOT = Path(__file__).resolve().parents[4]
_PG2 = _REPO_ROOT / "asserts" / "robots" / "pipergo2" / "pipergo2.usd"
PIPERGO2_USD_PATH = str(
    _PG2
    if _PG2.is_file()
    else _REPO_ROOT / "examples" / "robots" / "pipergo2" / "pipergo2.usd"
)

move_by_speed_cfg = AliengoMoveBySpeedControllerCfg(
    name='move_by_speed',
    policy_weights_path=gm.ASSET_PATH + "/robots/aliengo/policy/move_by_speed/aliengo_loco_model_4000.pt",
    joint_names=[
        'FL_hip_joint',
        'FR_hip_joint',
        'RL_hip_joint',
        'RR_hip_joint',
        'FL_thigh_joint',
        'FR_thigh_joint',
        'RL_thigh_joint',
        'RR_thigh_joint',
        'FL_calf_joint',
        'FR_calf_joint',
        'RL_calf_joint',
        'RR_calf_joint',
    ],
)

move_to_point_cfg = MoveToPointBySpeedControllerCfg(
    name='move_to_point',
    forward_speed=0.8,
    rotation_speed=3.0,
    threshold=0.08,
    sub_controllers=[move_by_speed_cfg],
)

move_along_path_cfg = MoveAlongPathPointsControllerCfg(
    name='move_along_path',
    forward_speed=0.8,
    rotation_speed=3.0,
    threshold=0.12,
    sub_controllers=[move_to_point_cfg],
)

rotate_cfg = RotateControllerCfg(
    name='rotate',
    rotation_speed=2.0,
    threshold=0.08,
    sub_controllers=[move_by_speed_cfg],
)


class PiperGo2RobotCfg(RobotCfg):
    name: Optional[str] = 'pipergo2'
    type: Optional[str] = 'PiperGo2Robot'
    prim_path: Optional[str] = '/pipergo2'
    usd_path: Optional[str] = PIPERGO2_USD_PATH
    base_link_name: Optional[str] = 'trunk'
    arm_mass_scale: Optional[float] = 0.35
