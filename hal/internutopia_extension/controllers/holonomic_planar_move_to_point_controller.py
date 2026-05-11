from collections import OrderedDict
from typing import Any, List

import numpy as np

from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.robot.articulation_subset import ArticulationSubset
from internutopia.core.robot.controller import BaseController
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia.core.util import log
from internutopia_extension.configs.controllers.holonomic_planar_move_to_point_controller import (
    HolonomicPlanarMoveToPointControllerCfg,
)

_DEFAULT_BASE_JOINTS = (
    "root_x_axis_joint",
    "root_y_axis_joint",
    "root_z_rotation_joint",
)
_SAFE_MAX_LINEAR = 0.35
_SAFE_MAX_ANGULAR = 1.0
_XLE_LINEAR_SIGN_X = -1.0
_XLE_LINEAR_SIGN_Y = -1.0


@BaseController.register("HolonomicPlanarMoveToPointController")
class HolonomicPlanarMoveToPointController(BaseController):
    """底盘三关节速度跟踪 (gx, gy, *)，action 与 MoveToPointBySpeed 相同：[(gx, gy, gz)]。

    与 `xlerobot_isaaclab-/.../xlerobot_env_cfg.py` 中 `JointVelocityActionCfg` 及键盘
    `_BASE_VEL_MAPPING` 一致：``(root_x, root_y, root_z)`` 对应 **世界系** ``(vx, vy, vtheta)``，
    不做机体系旋转。
    """

    def __init__(self, config: HolonomicPlanarMoveToPointControllerCfg, robot: BaseRobot, scene: IScene) -> None:
        self.goal_position: np.ndarray | None = None
        self.last_threshold: float | None = None

        self.forward_speed = config.forward_speed if config.forward_speed is not None else 0.5
        self.rotation_speed = config.rotation_speed if config.rotation_speed is not None else 2.0
        self.threshold = config.threshold if config.threshold is not None else 0.2
        self.stop_commanding_dist = (
            config.stop_commanding_dist if getattr(config, "stop_commanding_dist", None) is not None else 0.08
        )
        self.min_linear_speed = (
            config.min_linear_speed if getattr(config, "min_linear_speed", None) is not None else 0.0
        )
        self.velocity_lpf_alpha = (
            float(config.velocity_lpf_alpha)
            if getattr(config, "velocity_lpf_alpha", None) is not None
            else 0.0
        )
        self.linear_gain = config.linear_gain if config.linear_gain is not None else 1.0
        self.max_cmd_delta_linear = (
            float(config.max_cmd_delta_linear)
            if getattr(config, "max_cmd_delta_linear", None) is not None
            else 0.03
        )
        self.max_cmd_delta_angular = (
            float(config.max_cmd_delta_angular)
            if getattr(config, "max_cmd_delta_angular", None) is not None
            else 0.08
        )
        self.fall_pitch_roll_threshold_rad = (
            float(config.fall_pitch_roll_threshold_rad)
            if getattr(config, "fall_pitch_roll_threshold_rad", None) is not None
            else 0.7
        )
        self.fall_cooldown_steps = (
            int(config.fall_cooldown_steps)
            if getattr(config, "fall_cooldown_steps", None) is not None
            else 90
        )
        self.max_base_height = (
            float(config.max_base_height)
            if getattr(config, "max_base_height", None) is not None
            else 0.35
        )
        self.xle_mode = bool(getattr(config, "xle_mode", False))
        self.xle_goal_switch_brake_steps = int(getattr(config, "xle_goal_switch_brake_steps", 0) or 0)
        self.xle_disable_rotation = bool(getattr(config, "xle_disable_rotation", False))
        self.xle_approach_ramp_m = float(getattr(config, "xle_approach_ramp_m", 0.0) or 0.0)

        names = config.base_joint_names if config.base_joint_names else list(_DEFAULT_BASE_JOINTS)
        self._base_joint_names = list(names)

        super().__init__(config=config, robot=robot, scene=scene)

        self._base_vel_filt: np.ndarray | None = None
        self._lpf_goal_xy: np.ndarray | None = None
        self._prev_cmd = np.zeros(3, dtype=float)
        self._fall_cooldown_left = 0
        self._goal_switch_brake_left = 0

        # 不在此处读取 articulation.dof_names：create_controllers 发生在 articulation
        # handles 初始化之前，dof_names 可能为 None。关节索引由 ArticulationSubset 在
        # handles_initialized 之后再解析。
        self._joint_subset = ArticulationSubset(self.robot.articulation, self._base_joint_names)
        self._logged_dof_mismatch = False

    def _maybe_log_dof_mismatch(self) -> None:
        if self._logged_dof_mismatch:
            return
        dof_names = self.robot.articulation.dof_names
        if dof_names is None:
            return
        available = set(dof_names)
        missing = [j for j in self._base_joint_names if j not in available]
        if missing:
            log.warning(
                "HolonomicPlanarMoveToPoint: configured base joints missing on articulation: %s (dof_names=%s)",
                missing,
                list(dof_names),
            )
        self._logged_dof_mismatch = True

    @staticmethod
    def _quat_wxyz_yaw(quat_wxyz: np.ndarray) -> float:
        w, x, y, z = [float(v) for v in quat_wxyz]
        return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    @staticmethod
    def _quat_wxyz_roll_pitch(quat_wxyz: np.ndarray) -> tuple[float, float]:
        w, x, y, z = [float(v) for v in quat_wxyz]
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = float(np.arctan2(sinr_cosp, cosr_cosp))
        sinp = 2.0 * (w * y - z * x)
        sinp = float(np.clip(sinp, -1.0, 1.0))
        pitch = float(np.arcsin(sinp))
        return roll, pitch

    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        assert len(action) == 1, "action must be [(gx, gy, gz)]"
        if not self.robot.articulation.handles_initialized:
            return ArticulationAction()
        if self.xle_mode and self._fall_cooldown_left > 0:
            self._fall_cooldown_left -= 1
            self._prev_cmd = np.zeros(3, dtype=float)
            zeros = np.zeros(len(self._base_joint_names), dtype=float)
            return self._joint_subset.make_articulation_action(None, zeros)
        self._maybe_log_dof_mismatch()
        if self._joint_subset.joint_indices is None:
            return ArticulationAction()

        goal = np.array(action[0], dtype=float)
        pos, quat = self.robot.get_pose()
        pos = np.array(pos, dtype=float)
        quat = np.array(quat, dtype=float)
        if not np.isfinite(pos).all() or not np.isfinite(quat).all():
            log.warning("HolonomicPlanarMoveToPoint: invalid pose (nan/inf), zeroing base command")
            self._prev_cmd = np.zeros(3, dtype=float)
            zeros = np.zeros(len(self._base_joint_names), dtype=float)
            return self._joint_subset.make_articulation_action(None, zeros)
        if self.xle_mode:
            roll, pitch = self._quat_wxyz_roll_pitch(quat)
            if (
                abs(roll) > self.fall_pitch_roll_threshold_rad
                or abs(pitch) > self.fall_pitch_roll_threshold_rad
                or float(pos[2]) > self.max_base_height
                or float(pos[2]) < -0.20
            ):
                self._fall_cooldown_left = self.fall_cooldown_steps
                self._prev_cmd = np.zeros(3, dtype=float)
                log.warning(
                    "HolonomicPlanarMoveToPoint: fall guard triggered "
                    f"(roll={roll:.3f}, pitch={pitch:.3f}, z={float(pos[2]):.3f}), braking for {self.fall_cooldown_steps} steps"
                )
                zeros = np.zeros(len(self._base_joint_names), dtype=float)
                return self._joint_subset.make_articulation_action(None, zeros)

        err_xy = goal[:2] - pos[:2]
        dist = float(np.linalg.norm(err_xy))

        self.goal_position = np.array([goal[0], goal[1], 0.0], dtype=float)
        scale = float(self.robot.get_robot_scale()[0])
        self.last_threshold = float(self.threshold * scale)
        stop_cmd = float(self.stop_commanding_dist * scale)

        # 仅在极近处停轮；「到达」半径由 last_threshold 与 get_obs 共用（须 >= demo waypoint 半径）
        if dist < stop_cmd:
            self._prev_cmd = np.zeros(3, dtype=float)
            zeros = np.zeros(len(self._base_joint_names), dtype=float)
            return self._joint_subset.make_articulation_action(None, zeros)

        # 仅 XLe 模式：目标切换时先短暂刹停，降低 waypoint 切换冲击导致的后仰。
        gxy = np.array(goal[:2], dtype=float)
        if self.xle_mode:
            if self._lpf_goal_xy is not None and float(np.linalg.norm(gxy - self._lpf_goal_xy)) > 0.10:
                self._goal_switch_brake_left = max(self._goal_switch_brake_left, self.xle_goal_switch_brake_steps)
            if self._goal_switch_brake_left > 0:
                self._goal_switch_brake_left -= 1
                self._prev_cmd = np.zeros(3, dtype=float)
                zeros = np.zeros(len(self._base_joint_names), dtype=float)
                self._lpf_goal_xy = gxy
                return self._joint_subset.make_articulation_action(None, zeros)

        dir_w = err_xy / max(dist, 1e-8)
        speed = min(self.forward_speed, self.linear_gain * dist)
        # XLe：接近目标时锥形衰减线速度，且不在锥形区内强加 min_linear_speed（否则刹不住、易弹飞）。
        ramp = float(self.xle_approach_ramp_m) if self.xle_mode else 0.0
        in_soft_approach = ramp > stop_cmd and dist > stop_cmd and dist < ramp
        if in_soft_approach:
            edge = float((dist - stop_cmd) / max(ramp - stop_cmd, 1e-6))
            edge = float(np.clip(edge, 0.0, 1.0))
            speed = float(speed) * edge
        if self.min_linear_speed > 0.0:
            if not in_soft_approach:
                speed = max(float(speed), float(self.min_linear_speed))
        sign_x = _XLE_LINEAR_SIGN_X if self.xle_mode else 1.0
        sign_y = _XLE_LINEAR_SIGN_Y if self.xle_mode else 1.0
        v_wx = float(sign_x * dir_w[0] * speed)
        v_wy = float(sign_y * dir_w[1] * speed)

        yaw = self._quat_wxyz_yaw(quat)
        yaw_des = float(np.arctan2(err_xy[1], err_xy[0]))
        yaw_err = (yaw_des - yaw + np.pi) % (2.0 * np.pi) - np.pi
        # 近目标时减弱转向，优先平移贴点（与键盘「先平移」手感一致，减少绕圈）
        yaw_wt = float(np.clip(dist / 0.85, 0.0, 1.0))
        w = float(np.clip(0.35 * yaw_err * yaw_wt, -self.rotation_speed, self.rotation_speed))
        if self.xle_mode and self.xle_disable_rotation:
            w = 0.0

        # 世界系线速度 + 绕 z 角速度，与 Isaac Lab XLeRobot 键盘/MDP 约定一致
        v_cmd = np.array([v_wx, v_wy, w], dtype=float)
        if self.xle_mode:
            lim_v = min(float(self.forward_speed), _SAFE_MAX_LINEAR)
            lim_w = min(float(self.rotation_speed), _SAFE_MAX_ANGULAR)
        else:
            lim_v = float(self.forward_speed)
            lim_w = float(self.rotation_speed)
        v_cmd[0] = float(np.clip(v_cmd[0], -lim_v, lim_v))
        v_cmd[1] = float(np.clip(v_cmd[1], -lim_v, lim_v))
        v_cmd[2] = float(np.clip(v_cmd[2], -lim_w, lim_w))

        # 目标突变时清空滤波，避免把上一段的惯性速度带到新目标
        if self._lpf_goal_xy is None or float(np.linalg.norm(gxy - self._lpf_goal_xy)) > 0.35:
            self._base_vel_filt = np.zeros(3, dtype=float)
        self._lpf_goal_xy = gxy

        a = float(self.velocity_lpf_alpha)
        if a > 0.0:
            if self._base_vel_filt is None:
                self._base_vel_filt = np.zeros(3, dtype=float)
            self._base_vel_filt = (1.0 - a) * self._base_vel_filt + a * v_cmd
            v_out = self._base_vel_filt.copy()
        else:
            v_out = v_cmd

        if self.xle_mode:
            delta = v_out - self._prev_cmd
            delta[0] = float(np.clip(delta[0], -self.max_cmd_delta_linear, self.max_cmd_delta_linear))
            delta[1] = float(np.clip(delta[1], -self.max_cmd_delta_linear, self.max_cmd_delta_linear))
            delta[2] = float(np.clip(delta[2], -self.max_cmd_delta_angular, self.max_cmd_delta_angular))
            v_out = self._prev_cmd + delta
        self._prev_cmd = v_out.copy()

        return self._joint_subset.make_articulation_action(None, v_out)

    def get_obs(self) -> OrderedDict[str, Any]:
        if self.goal_position is None or self.last_threshold is None:
            return self._make_ordered()
        pos, _ = self.robot.get_pose()
        pos = np.array(pos, dtype=float)
        pos[2] = 0.0
        g = np.array(self.goal_position, dtype=float)
        g[2] = 0.0
        dist_from_goal = float(np.linalg.norm(pos[:2] - g[:2]))
        finished = dist_from_goal < self.last_threshold
        return self._make_ordered({"finished": finished})
