from pathlib import Path
from typing import List, Optional, Tuple

from pydantic import Field

from internutopia.core.config.robot import ControllerCfg, RobotCfg
from internutopia_extension.configs.controllers.holonomic_planar_move_to_point_controller import (
    HolonomicPlanarMoveToPointControllerCfg,
)

_REPO_ROOT = Path(__file__).resolve().parents[3]
XLEROBOT_USD_PATH = str((_REPO_ROOT / "xlerobot_isaaclab-" / "assets" / "robots" / "xlerobot" / "xlerobot.usd").resolve())

# DoF names from xlerobot Isaac Lab stack (see xlerobot_isaaclab-/.../utils/constant.py).
XLEROBOT_EXPECTED_DOF_NAMES = [
    "Rotation",
    "Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
    "Jaw",
    "Rotation_2",
    "Pitch_2",
    "Elbow_2",
    "Wrist_Pitch_2",
    "Wrist_Roll_2",
    "Jaw_2",
    "head_pan_joint",
    "head_tilt_joint",
    "root_x_axis_joint",
    "root_y_axis_joint",
    "root_z_rotation_joint",
]

XLEROBOT_ARM_JOINT_NAMES = (
    "Rotation",
    "Pitch",
    "Elbow",
    "Wrist_Pitch",
    "Wrist_Roll",
    "Rotation_2",
    "Pitch_2",
    "Elbow_2",
    "Wrist_Pitch_2",
    "Wrist_Roll_2",
)

XLEROBOT_GRIPPER_JOINT_NAMES = (
    "Jaw",
    "Jaw_2",
)

XLEROBOT_HEAD_JOINT_NAMES = (
    "head_pan_joint",
    "head_tilt_joint",
)

XLEROBOT_BASE_JOINT_NAMES = (
    "root_x_axis_joint",
    "root_y_axis_joint",
    "root_z_rotation_joint",
)

XLEROBOT_DEFAULT_MOVE_TO_POINT = {
    "forward_speed": 0.11,
    "rotation_speed": 0.28,
    "threshold": 0.16,
    "stop_commanding_dist": 0.05,
    "linear_gain": 0.18,
    # 近距非 0 会在停轮阈值前持续推底盘，与「过点瞬间全零」叠加易弹跳离地；改由 xle_approach_ramp_m 软减速。
    "min_linear_speed": 0.0,
    "velocity_lpf_alpha": 0.14,
}

XLEROBOT_DEFAULT_PHYSICS = {
    "arm_joint_names": XLEROBOT_ARM_JOINT_NAMES,
    "gripper_joint_names": XLEROBOT_GRIPPER_JOINT_NAMES,
    "head_joint_names": XLEROBOT_HEAD_JOINT_NAMES,
    "base_joint_names": XLEROBOT_BASE_JOINT_NAMES,
    "arm_kps": (28.0,),
    "arm_kds": (6.0,),
    "gripper_kps": (28.0,),
    "gripper_kds": (6.0,),
    "head_kps": (28.0,),
    "head_kds": (6.0,),
    "base_kps": (0.0, 0.0, 0.0),
    # 略提高阻尼，速度指令到 0 时更快耗散关节速度，减轻「刹不住」与滑移后的竖直扰动。
    "base_kds": (72.0, 72.0, 28.0),
    "solver_position_iteration_count": 10,
    "solver_velocity_iteration_count": 4,
    "enable_self_collisions": False,
}

# 与 test3_debug 中 `move_to_point_cfg.name` 一致，便于同一套 action 字典
move_to_point_cfg = HolonomicPlanarMoveToPointControllerCfg(
    name="move_to_point",
    # 与官方键盘一档 (0.1 m/s) 同量级；上身每步位置保持后略可提高线速度
    forward_speed=XLEROBOT_DEFAULT_MOVE_TO_POINT["forward_speed"],
    rotation_speed=XLEROBOT_DEFAULT_MOVE_TO_POINT["rotation_speed"],
    threshold=XLEROBOT_DEFAULT_MOVE_TO_POINT["threshold"],
    stop_commanding_dist=XLEROBOT_DEFAULT_MOVE_TO_POINT["stop_commanding_dist"],
    linear_gain=XLEROBOT_DEFAULT_MOVE_TO_POINT["linear_gain"],
    min_linear_speed=XLEROBOT_DEFAULT_MOVE_TO_POINT["min_linear_speed"],
    velocity_lpf_alpha=XLEROBOT_DEFAULT_MOVE_TO_POINT["velocity_lpf_alpha"],
    # 仅在 XLeRobot 启用的控制补偿，不影响其他机器人。
    xle_mode=True,
    xle_goal_switch_brake_steps=0,
    # 约 0.28m 外开始软刹，与 stop_commanding_dist=0.05 衔接，避免硬切断速度。
    xle_approach_ramp_m=0.28,
    xle_disable_rotation=True,
    # 抑制速度阶跃，减少“过点后后仰摔倒”。
    max_cmd_delta_linear=0.006,
    max_cmd_delta_angular=0.030,
)


class XlerobotRobotCfg(RobotCfg):
    name: Optional[str] = "xlerobot"
    type: Optional[str] = "XlerobotRobot"
    prim_path: Optional[str] = "/xlerobot"
    usd_path: Optional[str] = XLEROBOT_USD_PATH
    # 世界位姿主刚体：与 USD 中全向关节链末端 ``base_link`` 一致（PhysX 与整机碰撞/视觉对齐）。
    base_link_name: Optional[str] = "base_link"
    arm_joint_names: Optional[Tuple[str, ...]] = XLEROBOT_DEFAULT_PHYSICS["arm_joint_names"]
    gripper_joint_names: Optional[Tuple[str, ...]] = XLEROBOT_DEFAULT_PHYSICS["gripper_joint_names"]
    head_joint_names: Optional[Tuple[str, ...]] = XLEROBOT_DEFAULT_PHYSICS["head_joint_names"]
    base_joint_names: Optional[Tuple[str, ...]] = XLEROBOT_DEFAULT_PHYSICS["base_joint_names"]
    arm_kps: Optional[Tuple[float, ...]] = XLEROBOT_DEFAULT_PHYSICS["arm_kps"]
    arm_kds: Optional[Tuple[float, ...]] = XLEROBOT_DEFAULT_PHYSICS["arm_kds"]
    gripper_kps: Optional[Tuple[float, ...]] = XLEROBOT_DEFAULT_PHYSICS["gripper_kps"]
    gripper_kds: Optional[Tuple[float, ...]] = XLEROBOT_DEFAULT_PHYSICS["gripper_kds"]
    head_kps: Optional[Tuple[float, ...]] = XLEROBOT_DEFAULT_PHYSICS["head_kps"]
    head_kds: Optional[Tuple[float, ...]] = XLEROBOT_DEFAULT_PHYSICS["head_kds"]
    base_kps: Optional[Tuple[float, ...]] = XLEROBOT_DEFAULT_PHYSICS["base_kps"]
    base_kds: Optional[Tuple[float, ...]] = XLEROBOT_DEFAULT_PHYSICS["base_kds"]
    solver_position_iteration_count: Optional[int] = XLEROBOT_DEFAULT_PHYSICS["solver_position_iteration_count"]
    solver_velocity_iteration_count: Optional[int] = XLEROBOT_DEFAULT_PHYSICS["solver_velocity_iteration_count"]
    enable_self_collisions: Optional[bool] = XLEROBOT_DEFAULT_PHYSICS["enable_self_collisions"]
    enable_manipulator_pose_hold: Optional[bool] = True
    controllers: Optional[List[ControllerCfg]] = Field(
        default_factory=lambda: [move_to_point_cfg],
    )
