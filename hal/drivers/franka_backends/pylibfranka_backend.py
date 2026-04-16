"""
hal/drivers/backends/pylibfranka_backend.py

pylibfranka 后端实现
pylibfranka Backend Implementation

通过 pylibfranka (Python bindings for libfranka) 控制 Franka 机器人。
Control Franka robot via pylibfranka (Python bindings for libfranka).

Author: PhyAgentOS Team
"""

from __future__ import annotations

import copy
import os
import time
from datetime import datetime, timezone
from typing import Any

from hal.drivers.franka_backends.franka_backend import (
    BackendCapability,
    BackendInfo,
    ConnectionStatus,
    FrankaBackend,
)

# Default collision thresholds (compatible with FR3)
_DEFAULT_LOWER_TORQUE_THRESHOLDS = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
_DEFAULT_UPPER_TORQUE_THRESHOLDS = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
_DEFAULT_LOWER_FORCE_THRESHOLDS = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
_DEFAULT_UPPER_FORCE_THRESHOLDS = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

# Default joint impedance gains
_DEFAULT_JOINT_IMPEDANCE = [3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0]
_DEFAULT_CARTESIAN_IMPEDANCE = [3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0]

# Known Franka Control Box IPs (Shop Floor network)
_KNOWN_CONTROL_BOX_IPS = [
    "172.16.0.2",   # Common Shop Floor IP (user's configuration)
    "192.168.1.1",  # Franka default
    "172.16.0.1",   # Alternative Shop Floor
    "192.168.0.1",  # Robot network (less common for direct control)
]


def _parse_float(raw: str | None, default: float) -> float:
    """解析浮点数 / Parse float value."""
    if raw in (None, ""):
        return default
    try:
        return float(raw)
    except ValueError:
        return default


class PylibfrankaBackend(FrankaBackend):
    """
    pylibfranka 后端 / pylibfranka Backend

    通过 pylibfranka 控制 Franka 机器人，支持：
    - 关节位置控制
    - 笛卡尔位置控制
    - 夹爪控制
    - 力控和阻抗控制
    - 实时控制模式 (1kHz)

    Control Franka robot via pylibfranka, supports:
    - Joint position control
    - Cartesian position control
    - Gripper control
    - Force and impedance control
    - Real-time control mode (1kHz)

    Parameters
    ----------
    ip:
        Control Box IP address (Shop Floor Interface).
    control_rate:
        Desired control loop rate in Hz. Default 250.
    robot_id:
        Logical identifier for this robot instance.
    default_gripper_force:
        Default grasping force in Newtons. Default 20.0.
    collision_thresholds:
        Custom collision thresholds.
    reconnect_policy:
        One of "auto" or "manual". Default "auto".
    realtime_mode:
        If True, enforce real-time kernel mode (1 kHz).
        If False, use kIgnore mode (development-friendly).
        Default False.
    auto_discover:
        If True and ip is not provided, try known Control Box IPs.
    """

    def __init__(
        self,
        ip: str | None = None,
        control_rate: float | None = None,
        robot_id: str = "franka_pylibfranka",
        default_gripper_force: float | None = None,
        safe_max_linear_m_s: float | None = None,
        safe_max_angular_deg_s: float | None = None,
        reconnect_policy: str = "auto",
        realtime_mode: bool = False,
        collision_thresholds: dict | None = None,
        auto_discover: bool = True,
        **kwargs: Any,
    ) -> None:
        # Resolve IP
        resolved_ip = self._resolve_ip(ip, auto_discover)

        super().__init__(
            ip=resolved_ip,
            robot_id=robot_id,
            reconnect_policy=reconnect_policy,
            **kwargs,
        )

        self.control_rate = max(
            control_rate if control_rate is not None
            else _parse_float(os.environ.get("PAOS_FRANKA_CONTROL_RATE"), 250.0),
            1.0,
        )
        self.default_gripper_force = max(
            default_gripper_force if default_gripper_force is not None
            else _parse_float(os.environ.get("PAOS_FRANKA_GRIPPER_FORCE"), 20.0),
            0.0,
        )
        self.safe_max_linear_m_s = max(
            safe_max_linear_m_s if safe_max_linear_m_s is not None
            else _parse_float(os.environ.get("PAOS_FRANKA_SAFE_MAX_LINEAR_M_S"), 0.5),
            0.0,
        )
        self.safe_max_angular_deg_s = max(
            safe_max_angular_deg_s if safe_max_angular_deg_s is not None
            else _parse_float(os.environ.get("PAOS_FRANKA_SAFE_MAX_ANGULAR_DEG_S"), 30.0),
            0.0,
        )
        self.realtime_mode = realtime_mode
        self.auto_discover = auto_discover

        # Collision thresholds
        if collision_thresholds is None:
            self._collision_thresholds = {
                "lower_torque": _DEFAULT_LOWER_TORQUE_THRESHOLDS,
                "upper_torque": _DEFAULT_UPPER_TORQUE_THRESHOLDS,
                "lower_force": _DEFAULT_LOWER_FORCE_THRESHOLDS,
                "upper_force": _DEFAULT_UPPER_FORCE_THRESHOLDS,
            }
        else:
            self._collision_thresholds = collision_thresholds

        # pylibfranka instances
        self._robot: Any | None = None
        self._gripper: Any | None = None
        self._active_control: Any | None = None

        # Internal state
        self._runtime_state: dict[str, Any] = {
            "robots": {self._robot_id: self._make_robot_state()}
        }

    def _resolve_ip(self, ip: str | None, auto_discover: bool) -> str:
        """解析机器人 IP / Resolve robot IP."""
        if ip is not None and ip.strip():
            return ip.strip()

        env_ip = os.environ.get("PAOS_FRANKA_IP", "").strip()
        if env_ip:
            return env_ip

        if auto_discover and self._try_auto_discover():
            return _KNOWN_CONTROL_BOX_IPS[0]

        return "172.16.0.2"

    def _try_auto_discover(self) -> bool:
        """尝试自动发现 Franka Control Box / Try to discover Franka Control Box."""
        import socket

        for candidate_ip in _KNOWN_CONTROL_BOX_IPS:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1.0)
                result = sock.connect_ex((candidate_ip, 7512))
                sock.close()
                if result == 0:
                    _KNOWN_CONTROL_BOX_IPS.insert(
                        0, _KNOWN_CONTROL_BOX_IPS.pop(_KNOWN_CONTROL_BOX_IPS.index(candidate_ip))
                    )
                    return True
            except Exception:
                continue
        return False

    # ── FrankaBackend 接口实现 / FrankaBackend Interface Implementation ──────

    @property
    def backend_name(self) -> str:
        return "pylibfranka"

    @property
    def backend_version(self) -> str | None:
        """获取 pylibfranka 版本 / Get pylibfranka version."""
        try:
            import pylibfranka
            return pylibfranka.__version__ if hasattr(pylibfranka, "__version__") else "unknown"
        except ImportError:
            return None

    def is_available(self) -> bool:
        """检查 pylibfranka 是否已安装 / Check if pylibfranka is installed."""
        try:
            import pylibfranka
            return True
        except ImportError:
            return False

    def can_connect(self) -> bool:
        """检查是否可以连接到机器人 / Check if can connect to robot."""
        if not self.is_available():
            self._last_error = "pylibfranka not installed"
            return False

        if self._robot is not None:
            return True

        # Try to connect without affecting state
        try:
            import pylibfranka
            from pylibfranka import RealtimeConfig

            rt_config = (
                RealtimeConfig.kEnforce if self.realtime_mode
                else RealtimeConfig.kIgnore
            )
            test_robot = pylibfranka.Robot(self._ip, rt_config)
            test_robot.stop()
            return True
        except Exception as e:
            self._last_error = str(e)
            return False

    def connect(self) -> bool:
        """连接到机器人 / Connect to robot."""
        if self._robot is not None:
            self._set_connection_status("connected", last_error=None)
            self._touch_heartbeat()
            return True

        try:
            import pylibfranka
            from pylibfranka import ControllerMode, RealtimeConfig

            rt_config = (
                RealtimeConfig.kEnforce if self.realtime_mode
                else RealtimeConfig.kIgnore
            )

            self._robot = pylibfranka.Robot(self._ip, rt_config)

            ct = self._collision_thresholds
            self._robot.set_collision_behavior(
                ct["lower_torque"],
                ct["upper_torque"],
                ct["lower_force"],
                ct["upper_force"],
            )

            if not self.realtime_mode:
                self._robot.set_joint_impedance(_DEFAULT_JOINT_IMPEDANCE)
                self._robot.set_cartesian_impedance(_DEFAULT_CARTESIAN_IMPEDANCE)

            self._gripper = pylibfranka.Gripper(self._ip)
            _ = self._robot.read_once()

            self._set_connection_status("connected", last_error=None)
            self._touch_heartbeat()
            self._set_arm_state(mode="idle", status="idle", last_error=None)
            self._update_gripper_state()
            return True

        except ImportError:
            self._set_connection_status(
                "error",
                last_error="pylibfranka not installed. Run: pip install pylibfranka",
            )
            self._robot = None
            self._gripper = None
            return False
        except Exception as exc:
            error_msg = str(exc)
            self._set_connection_status("error", last_error=error_msg)
            self._robot = None
            self._gripper = None

            if "timeout" in error_msg.lower() or "connection" in error_msg.lower():
                self._set_connection_status(
                    "error",
                    last_error=(
                        f"Cannot reach Control Box at {self._ip}. "
                        "Check: (1) physical connection, "
                        "(2) Control Box is powered on, "
                        "(3) FCI is activated in Desk, "
                        "(4) network is configured (e.g., 172.16.0.x)."
                    ),
                )
            elif "incompatible" in error_msg.lower():
                self._set_connection_status(
                    "error",
                    last_error=(
                        "libfranka version mismatch. "
                        "Ensure pylibfranka version matches robot firmware. "
                        "See compatibility table at docs/user_manual/appendix/franka_compatibility.md"
                    ),
                )
            return False

    def disconnect(self) -> None:
        """断开与机器人的连接 / Disconnect from robot."""
        if self._robot is None:
            self._set_connection_status("disconnected", last_error=None)
            self._set_arm_state(mode="idle", status="idle", last_error=None)
            return

        try:
            if self._is_connected_internal():
                self._robot.stop()
        except Exception:
            pass

        try:
            if self._gripper is not None:
                try:
                    self._gripper.stop()
                except Exception:
                    pass
        except Exception as exc:
            self._set_connection_status("error", last_error=str(exc))
        finally:
            self._robot = None
            self._gripper = None
            self._active_control = None
            self._set_connection_status("disconnected", last_error=None)
            self._set_arm_state(mode="idle", status="idle", last_error=None)

    def is_connected(self) -> bool:
        """检查是否已连接 / Check if connected."""
        return self._robot is not None

    def _is_connected_internal(self) -> bool:
        """内部连接检查 / Internal connection check."""
        if self._robot is None:
            return False
        try:
            self._robot.read_once()
            return True
        except Exception:
            return False

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        speed: float = 0.05,
        **kwargs: Any,
    ) -> str:
        """移动末端执行器到笛卡尔位置 / Move end-effector to Cartesian position."""
        if not self.is_connected():
            return self._conn_error()

        try:
            x = float(x)
            y = float(y)
            z = float(z)
            roll = float(roll)
            pitch = float(pitch)
            yaw = float(yaw)
            speed = max(0.0, min(speed, self.safe_max_linear_m_s))
            duration = float(kwargs.get("duration", 5.0))
        except (TypeError, ValueError) as exc:
            return self._error_result(f"invalid move_to parameters: {exc}")

        self._set_arm_state(
            mode="cartesian_pose",
            status="running",
            goal={"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw},
            last_error=None,
        )

        try:
            import numpy as np
            from pylibfranka import CartesianPose, ControllerMode

            self._active_control = self._robot.start_cartesian_pose_control(
                ControllerMode.JointImpedance
            )

            state, _ = self._active_control.readOnce()
            initial_pose = np.array(state.O_T_EE)

            Rx = np.array([
                [1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)],
            ])
            Ry = np.array([
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)],
            ])
            Rz = np.array([
                [np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw), np.cos(yaw), 0],
                [0, 0, 1],
            ])
            R_target = Rz @ Ry @ Rx

            target_pose = initial_pose.copy()
            target_pose[:3, :3] = R_target
            target_pose[:3, 3] = [x, y, z]
            pose_flat = target_pose.flatten("C")

            loop_dt = 1.0 / self.control_rate
            deadline = time.monotonic() + duration
            motion_finished = False

            while not motion_finished:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    motion_finished = True
                    break

                elapsed = duration - remaining
                alpha = 1.0 - np.cos(np.pi * min(elapsed / duration, 1.0)) / 2.0

                interp_pose = initial_pose + alpha * (pose_flat - initial_pose)
                command = CartesianPose(interp_pose)

                if alpha >= 1.0:
                    command.motion_finished = True
                    motion_finished = True

                self._active_control.writeOnce(command)
                time.sleep(min(loop_dt, remaining))

            self._active_control = None
            self._set_arm_state(mode="idle", status="arrived", last_error=None)
            return (
                f"Arm moved to x={x:.3f}m, y={y:.3f}m, z={z:.3f}m, "
                f"r={roll:.3f}p={pitch:.3f}y={yaw:.3f} rad."
            )

        except ImportError as exc:
            return self._error_result(f"pylibfranka not available: {exc}")
        except Exception as exc:
            self._active_control = None
            return self._error_result(f"move_to failed: {exc}")

    def move_joints(
        self,
        joints: list[float] | dict[str, float],
        speed: float = 0.05,
        **kwargs: Any,
    ) -> str:
        """移动关节到目标位置 / Move joints to target positions."""
        if not self.is_connected():
            return self._conn_error()

        try:
            if isinstance(joints, dict):
                q = [float(joints.get(f"q{i}", 0.0)) for i in range(1, 8)]
            else:
                q = [float(v) for v in joints]
            duration = float(kwargs.get("duration", 5.0))
        except (TypeError, ValueError) as exc:
            return self._error_result(f"invalid move_joints parameters: {exc}")

        self._set_arm_state(
            mode="joint_position",
            status="running",
            goal={"joints": q},
            last_error=None,
        )

        try:
            import numpy as np
            from pylibfranka import ControllerMode, JointPositions

            self._active_control = self._robot.start_joint_position_control(
                ControllerMode.CartesianImpedance
            )

            state, _ = self._active_control.readOnce()
            initial_q = np.array(state.q)
            target_q = np.array(q)

            loop_dt = 1.0 / self.control_rate
            deadline = time.monotonic() + duration
            motion_finished = False

            while not motion_finished:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    motion_finished = True
                    break

                elapsed = duration - remaining
                alpha = 1.0 - np.cos(np.pi * min(elapsed / duration, 1.0)) / 2.0

                interp_q = initial_q + alpha * (target_q - initial_q)
                command = JointPositions(interp_q.tolist())

                if alpha >= 1.0:
                    command.motion_finished = True
                    motion_finished = True

                self._active_control.writeOnce(command)
                time.sleep(min(loop_dt, remaining))

            self._active_control = None
            self._set_arm_state(mode="idle", status="arrived", last_error=None)
            return f"Arm moved to joint configuration: {[f'{v:.3f}' for v in q]}."

        except ImportError as exc:
            return self._error_result(f"pylibfranka not available: {exc}")
        except Exception as exc:
            self._active_control = None
            return self._error_result(f"move_joints failed: {exc}")

    def grasp(
        self,
        width: float,
        force: float = 20.0,
        speed: float = 0.1,
        **kwargs: Any,
    ) -> str:
        """闭合夹爪抓取物体 / Close gripper to grasp object."""
        if not self.is_connected():
            return self._conn_error()

        try:
            width = float(width)
            force = float(force)
            speed = float(speed)
        except (TypeError, ValueError) as exc:
            return self._error_result(f"invalid grasp parameters: {exc}")

        width = max(0.0, min(width, 0.08))
        force = max(0.0, min(force, 70.0))

        try:
            self._gripper.homing()
            time.sleep(1.0)
            success = self._gripper.grasp(width, speed, force)
            time.sleep(0.5)

            self._update_gripper_state()

            if success:
                self._set_arm_state(mode="idle", status="grasped", last_error=None)
                return f"Gripper grasped at width={width:.3f}m, force={force:.1f}N."
            else:
                return self._error_result("Grasp failed: object not detected or lost.")

        except Exception as exc:
            return self._error_result(f"grasp failed: {exc}")

    def move_gripper(
        self,
        width: float,
        speed: float = 0.1,
        **kwargs: Any,
    ) -> str:
        """移动夹爪到目标宽度 / Move gripper to target width."""
        if not self.is_connected():
            return self._conn_error()

        try:
            width = float(width)
            speed = float(speed)
        except (TypeError, ValueError) as exc:
            return self._error_result(f"invalid move_gripper parameters: {exc}")

        width = max(0.0, min(width, 0.08))

        try:
            self._gripper.homing()
            time.sleep(1.0)
            self._gripper.move(width, speed)
            time.sleep(0.5)

            self._update_gripper_state()
            self._set_arm_state(mode="idle", status="idle", last_error=None)
            return f"Gripper moved to width={width:.3f}m."

        except Exception as exc:
            return self._error_result(f"move_gripper failed: {exc}")

    def get_robot_state(self) -> dict[str, Any]:
        """获取机器人当前状态 / Get current robot state."""
        if not self.is_connected():
            return {"error": "robot not connected"}

        try:
            state = self._robot.read_once()
            gripper_state = self._gripper.read_once()

            ee_pos = [state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14]]

            return {
                "joint_positions": [float(v) for v in state.q],
                "joint_velocities": [float(v) for v in state.dq],
                "external_torques": [float(v) for v in state.tau_ext_hat_filtered],
                "ee_position": {
                    "x": float(ee_pos[0]),
                    "y": float(ee_pos[1]),
                    "z": float(ee_pos[2]),
                },
                "gripper_width": float(gripper_state.width),
                "gripper_grasped": bool(gripper_state.is_grasped),
            }
        except Exception as exc:
            return {"error": str(exc)}

    def get_backend_info(self) -> BackendInfo:
        """获取后端详细信息 / Get detailed backend information."""
        return BackendInfo(
            name=self.backend_name,
            version=self.backend_version,
            capabilities=self.get_capabilities(),
            connection_status=self._connection_status,
            last_error=self._last_error,
            robot_system_version=self._robot_system_version,
            is_available=self.is_available(),
        )

    def get_capabilities(self) -> set[BackendCapability]:
        """获取后端支持的能力集合 / Get set of capabilities supported by backend."""
        return {
            BackendCapability.JOINT_POSITION,
            BackendCapability.CARTESIAN_POSITION,
            BackendCapability.GRIPPER_CONTROL,
            BackendCapability.GRIPPER_FORCE,
            BackendCapability.COLLISION_DETECTION,
            BackendCapability.AUTO_RECONNECT,
            BackendCapability.HEARTBEAT,
            # Force control capabilities (if supported by configuration)
            BackendCapability.FORCE_CONTROL,
            BackendCapability.IMPEDANCE_CONTROL,
            # Real-time control (if realtime_mode is enabled)
            BackendCapability.REALTIME_CONTROL,
        }

    # ── 内部辅助方法 / Internal Helper Methods ──────────────────────────────

    def _conn_error(self) -> str:
        details = self._robot_state().get("connection_state", {}).get("last_error")
        suffix = f" Details: {details}" if details else ""
        return (
            "Connection error: robot is not connected. "
            "Run connect_robot first." + suffix
        )

    def _error_result(self, reason: str) -> str:
        self._mark_arm_failed(reason)
        self._stop_motion("failed", last_error=reason)
        return f"Error: {reason}"

    def _stop_motion(self, status: str, last_error: str | None = None) -> str:
        try:
            if self._active_control is not None:
                self._robot.stop()
                self._active_control = None
        except Exception:
            pass
        self._set_arm_state(mode="idle", status=status, last_error=last_error)
        return "Motion stopped." if status == "stopped" else f"Motion {status}."

    def _mark_arm_failed(self, reason: str) -> None:
        self._set_arm_state(mode="idle", status="failed", last_error=reason)

    def _robot_state(self) -> dict[str, Any]:
        robots = self._runtime_state.setdefault("robots", {})
        if self._robot_id not in robots:
            robots[self._robot_id] = self._make_robot_state()
        return robots[self._robot_id]

    def _make_robot_state(self) -> dict[str, Any]:
        stamp = self._stamp()
        return {
            "connection_state": {
                "status": "disconnected",
                "transport": "ethernet",
                "host": self._ip,
                "control_rate": self.control_rate,
                "realtime_mode": self.realtime_mode,
                "last_heartbeat": None,
                "last_error": None,
                "reconnect_attempts": 0,
            },
            "robot_pose": {
                "frame": "base_link",
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "stamp": stamp,
            },
            "joint_state": {
                "q": [0.0] * 7,
                "dq": [0.0] * 7,
                "tau": [0.0] * 7,
                "stamp": stamp,
            },
            "arm_state": {
                "mode": "idle",
                "status": "idle",
                "goal_id": None,
                "goal": None,
                "last_error": None,
            },
            "gripper_state": {
                "width": 0.0,
                "max_width": 0.08,
                "force": 0.0,
                "is_grasped": False,
                "stamp": stamp,
            },
        }

    def _update_pose_from_state(self, state) -> None:
        try:
            state_dict = self._robot_state()
            pose = dict(state_dict.get("robot_pose", {}))
            pose["x"] = float(state.O_T_EE[12])
            pose["y"] = float(state.O_T_EE[13])
            pose["z"] = float(state.O_T_EE[14])
            pose["stamp"] = self._stamp()
            state_dict["robot_pose"] = pose
        except Exception:
            pass

    def _update_gripper_state(self) -> None:
        try:
            gripper_state = self._gripper.read_once()
            state_dict = self._robot_state()
            gripper = dict(state_dict.get("gripper_state", {}))
            gripper["width"] = float(gripper_state.width)
            gripper["is_grasped"] = bool(gripper_state.is_grasped)
            gripper["stamp"] = self._stamp()
            state_dict["gripper_state"] = gripper
        except Exception:
            pass

    def _set_connection_status(self, status: str, last_error: str | None) -> None:
        state = self._robot_state()
        conn = dict(state.get("connection_state", {}))
        conn.update(
            {
                "status": status,
                "transport": "ethernet",
                "host": self._ip,
                "control_rate": self.control_rate,
                "realtime_mode": self.realtime_mode,
                "last_error": last_error,
            }
        )
        state["connection_state"] = conn

    def _set_arm_state(
        self,
        *,
        mode: str,
        status: str,
        goal: dict[str, Any] | None = None,
        last_error: str | None = None,
    ) -> None:
        state = self._robot_state()
        state["arm_state"] = {
            "mode": mode,
            "status": status,
            "goal_id": None,
            "goal": goal,
            "last_error": last_error,
        }

    def _touch_heartbeat(self) -> None:
        state = self._robot_state()
        conn = dict(state.get("connection_state", {}))
        conn["last_heartbeat"] = self._stamp()
        state["connection_state"] = conn

    @staticmethod
    def _stamp() -> str:
        return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
