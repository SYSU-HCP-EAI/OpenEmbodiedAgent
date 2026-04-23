"""Franka Research 3 driver for PhyAgentOS HAL.

Control via pylibfranka (Python bindings for libfranka).
Supports joint position, Cartesian pose, and gripper control modes.

Network Architecture:
    WorkStation PC (pylibfranka) --> Control Box (Shop Floor: 172.16.0.x)
                                              |
                                              +--> Robot Arm (internal: 192.168.0.x)

You connect to the Control Box's Shop Floor Interface, not directly to the arm.
"""

from __future__ import annotations

import copy
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from hal.base_driver import BaseDriver

_PROFILES_DIR = Path(__file__).resolve().parent.parent / "profiles"


def _parse_float(raw: str | None, default: float) -> float:
    if raw in (None, ""):
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _parse_int(raw: str | None, default: int) -> int:
    if raw in (None, ""):
        return default
    try:
        return int(raw)
    except ValueError:
        return default


# Default collision thresholds (compatible with FR3)
_DEFAULT_LOWER_TORQUE_THRESHOLDS = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
_DEFAULT_UPPER_TORQUE_THRESHOLDS = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
_DEFAULT_LOWER_FORCE_THRESHOLDS = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
_DEFAULT_UPPER_FORCE_THRESHOLDS = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

# Default joint impedance gains
_DEFAULT_JOINT_IMPEDANCE = [3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0]
_DEFAULT_CARTESIAN_IMPEDANCE = [3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0]

# Known Franka Control Box IPs (Shop Floor network)
# These are tried in auto-discovery mode
_KNOWN_CONTROL_BOX_IPS = [
    "172.16.0.2",   # Common Shop Floor IP (user's configuration)
    "192.168.1.1",  # Franka default
    "172.16.0.1",   # Alternative Shop Floor
    "192.168.0.1",  # Robot network (less common for direct control)
]


class FrankaDriver(BaseDriver):
    """Driver for Franka Research 3 (FR3) 7-DOF collaborative arm.

    Communication: Ethernet via pylibfranka (Franka Control Interface / FCI).

    Network Note:
        You connect to the Control Box's Shop Floor Interface (e.g., 172.16.0.2),
        NOT directly to the Robot Arm. The Control Box then communicates with
        the arm via its internal Robot Network.

    Parameters
    ----------
    ip:
        Control Box IP address (Shop Floor Interface).
        Default ``172.16.0.2``.
    control_rate:
        Desired control loop rate in Hz. Default ``250``.
        Use lower rates (e.g., 100) for non-real-time systems.
    realtime_mode:
        If True, enforce real-time kernel mode (1 kHz, strict timing).
        If False, use kIgnore mode (development-friendly, relaxed timing).
        Default ``False`` (easier to get started).
    robot_id:
        Logical identifier for this robot instance.
    default_gripper_force:
        Default grasping force in Newtons. Default ``20.0``.
    collision_thresholds:
        Custom collision thresholds as dict with keys:
        lower_torque, upper_torque, lower_force, upper_force.
        Each is a list of 7 (torque) or 6 (force) float values.
        If None, uses safe defaults suitable for most scenarios.
    reconnect_policy:
        One of ``"auto"`` or ``"manual"``. Default ``"auto"``.
    auto_discover:
        If True and ip is not provided, try known Control Box IPs.
        Default ``True``.
    """

    def __init__(
        self,
        ip: str | None = None,
        control_rate: float | None = None,
        robot_id: str | None = None,
        default_gripper_force: float | None = None,
        safe_max_linear_m_s: float | None = None,
        safe_max_angular_deg_s: float | None = None,
        reconnect_policy: str = "auto",
        realtime_mode: bool = False,
        collision_thresholds: dict | None = None,
        auto_discover: bool = True,
        gui: bool = False,
        **_kwargs: Any,
    ) -> None:
        # Determine IP: auto-discover or use provided/configured IP
        self._ip = self._resolve_ip(ip)
        self.control_rate = max(
            control_rate if control_rate is not None
            else _parse_float(os.environ.get("PAOS_FRANKA_CONTROL_RATE"), 250.0),
            1.0,
        )
        self.robot_id = (
            robot_id.strip() if robot_id is not None
            else (os.environ.get("PAOS_FRANKA_ROBOT_ID") or "franka_research3_001")
        ).strip()
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
        self.reconnect_policy = reconnect_policy
        self.auto_discover = auto_discover
        self._gui = gui

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

        self._objects: dict[str, dict] = {}
        self._robot: Any | None = None  # pylibfranka Robot instance
        self._gripper: Any | None = None  # pylibfranka Gripper instance
        self._active_control: Any | None = None  # Active control handle
        self._runtime_state = {"robots": {self.robot_id: self._make_robot_state()}}

    def _resolve_ip(self, ip: str | None) -> str:
        """Resolve robot IP from config, env var, or auto-discovery."""
        if ip is not None and ip.strip():
            return ip.strip()

        env_ip = os.environ.get("PAOS_FRANKA_IP", "").strip()
        if env_ip:
            return env_ip

        # Auto-discovery mode: try known IPs
        if self._try_auto_discover():
            return _KNOWN_CONTROL_BOX_IPS[0]  # First reachable IP

        # Fallback to default
        return "172.16.0.2"

    def _try_auto_discover(self) -> bool:
        """Try to discover a reachable Franka Control Box.

        Returns True if a reachable IP was found.
        """
        import socket

        for candidate_ip in _KNOWN_CONTROL_BOX_IPS:
            try:
                # Quick TCP connection test to pylibfranka default port
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1.0)
                # Port 7512 is used by libfranka for robot commands
                result = sock.connect_ex((candidate_ip, 7512))
                sock.close()
                if result == 0:
                    # Found reachable robot
                    _KNOWN_CONTROL_BOX_IPS.insert(0, _KNOWN_CONTROL_BOX_IPS.pop(
                        _KNOWN_CONTROL_BOX_IPS.index(candidate_ip)
                    ))
                    return True
            except Exception:
                continue
        return False

    # ── BaseDriver interface ─────────────────────────────────────────────────

    def get_profile_path(self) -> Path:
        return _PROFILES_DIR / "franka_research3.md"

    def load_scene(self, scene: dict[str, dict]) -> None:
        self._objects = dict(scene)

    def execute_action(self, action_type: str, params: dict) -> str:
        try:
            self._validate_robot_id(params)
            if action_type == "connect_robot":
                return "Robot connection established." if self.connect() else self._conn_error()
            if action_type == "check_connection":
                return "connected" if self.health_check() else "disconnected"
            if action_type == "disconnect_robot":
                self.disconnect()
                return "Robot connection closed."
            if action_type == "stop":
                return self._stop_motion("stopped")

            if action_type == "move_to":
                return self._execute_move_to(params)
            if action_type == "move_joints":
                return self._execute_move_joints(params)
            if action_type == "grasp":
                return self._execute_grasp(params)
            if action_type == "move_gripper":
                return self._execute_move_gripper(params)
            if action_type == "get_robot_state":
                return self._execute_get_state(params)

            return f"Unknown action: {action_type}"
        except ValueError as exc:
            return self._error_result(str(exc))
        except Exception as exc:
            return self._error_result(f"{action_type} failed: {exc}")

    def get_scene(self) -> dict[str, dict]:
        return dict(self._objects)

    # ── Connection lifecycle ────────────────────────────────────────────────

    def connect(self) -> bool:
        if self.is_connected():
            self._set_connection_status("connected", last_error=None)
            self._touch_heartbeat()
            return True

        if not self._ip:
            self._set_connection_status("error", last_error="missing Franka IP")
            return False

        try:
            import pylibfranka
            from pylibfranka import ControllerMode, RealtimeConfig

            # Choose realtime config based on mode
            rt_config = (
                RealtimeConfig.kEnforce if self.realtime_mode
                else RealtimeConfig.kIgnore
            )

            # Connect to robot
            self._robot = pylibfranka.Robot(self._ip, rt_config)

            # Set collision behavior
            ct = self._collision_thresholds
            self._robot.set_collision_behavior(
                ct["lower_torque"],
                ct["upper_torque"],
                ct["lower_force"],
                ct["upper_force"],
            )

            # Set impedance parameters (only available in non-realtime mode)
            if not self.realtime_mode:
                self._robot.set_joint_impedance(_DEFAULT_JOINT_IMPEDANCE)
                self._robot.set_cartesian_impedance(_DEFAULT_CARTESIAN_IMPEDANCE)

            # Connect to gripper
            self._gripper = pylibfranka.Gripper(self._ip)

            # Read initial state to validate connection
            _ = self._robot.read_once()

            self._set_connection_status("connected", last_error=None)
            self._touch_heartbeat()
            self._set_arm_state(mode="idle", status="idle", last_error=None)
            self._update_gripper_state()
            return True

        except ImportError as exc:
            self._set_connection_status(
                "error",
                last_error=f"pylibfranka not installed. Run: pip install pylibfranka",
            )
            self._robot = None
            self._gripper = None
            return False
        except Exception as exc:
            error_msg = str(exc)
            self._set_connection_status("error", last_error=error_msg)
            self._robot = None
            self._gripper = None

            # Provide helpful hints based on error type
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
                        "Ensure pylibfranka version matches robot firmware."
                    ),
                )

            return False

    def disconnect(self) -> None:
        if self._robot is None:
            self._set_connection_status("disconnected", last_error=None)
            self._set_arm_state(mode="idle", status="idle", last_error=None)
            return

        try:
            if self.is_connected():
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
        return self._robot is not None

    def health_check(self) -> bool:
        if not self.is_connected():
            if self.reconnect_policy == "auto":
                self._inc_reconnect_attempts()
                self._set_connection_status("reconnecting", last_error="disconnected")
                connected = self.connect()
                if not connected:
                    self._stop_motion(
                        "failed",
                        last_error=self._robot_state().get("connection_state", {}).get("last_error"),
                    )
                return connected
            self._set_connection_status("disconnected", last_error="disconnected")
            self._stop_motion("failed", last_error="disconnected")
            return False

        try:
            state = self._robot.read_once()
            self._touch_heartbeat()
            self._set_connection_status("connected", last_error=None)
            self._update_pose_from_state(state)
            self._update_joint_state_from_state(state)
            return True
        except Exception as exc:
            self._set_connection_status("error", last_error=str(exc))
            self._stop_motion("failed", last_error=str(exc))
            return False

    def get_runtime_state(self) -> dict[str, Any]:
        return copy.deepcopy(self._runtime_state)

    def close(self) -> None:
        self.disconnect()

    # ── Action executors ────────────────────────────────────────────────────

    def _execute_move_to(self, params: dict[str, Any]) -> str:
        """Move end-effector to Cartesian pose.

        Parameters
        ----------
        x, y, z:
            Target position in meters (relative to base frame).
        roll, pitch, yaw:
            Target orientation (Euler angles) in radians.
        speed:
            Optional, Cartesian speed limit in m/s.
        duration:
            Optional, motion duration in seconds. Default 5.0.
        """
        if not self.is_connected():
            return self._conn_error()

        try:
            x = float(params.get("x", 0.0))
            y = float(params.get("y", 0.0))
            z = float(params.get("z", 0.0))
            roll = float(params.get("roll", 0.0))
            pitch = float(params.get("pitch", 0.0))
            yaw = float(params.get("yaw", 0.0))
            speed = self._clip(
                float(params.get("speed", self.safe_max_linear_m_s)),
                self.safe_max_linear_m_s,
            )
            duration = float(params.get("duration", 5.0))
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

            # Start Cartesian pose control
            self._active_control = self._robot.start_cartesian_pose_control(
                ControllerMode.JointImpedance
            )

            # Get current EE pose
            state, _ = self._active_control.readOnce()
            initial_pose = np.array(state.O_T_EE)

            # Build target pose matrix
            # Use (x, y, z) as position, orientation from Euler angles
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

            # Motion loop with minimum jerk interpolation
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

    def _execute_move_joints(self, params: dict[str, Any]) -> str:
        """Move arm to joint configuration.

        Parameters
        ----------
        q1-q7 or joints:
            Target joint positions in radians.
        speed:
            Optional, motion speed factor (0-1). Default 0.5.
        duration:
            Optional, motion duration in seconds. Default 5.0.
        """
        if not self.is_connected():
            return self._conn_error()

        try:
            if "joints" in params and isinstance(params["joints"], dict):
                q = [float(params["joints"].get(f"q{i}", 0.0)) for i in range(1, 8)]
            else:
                q = [float(params.get(f"q{i}", 0.0)) for i in range(1, 8)]
            duration = float(params.get("duration", 5.0))
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

    def _execute_grasp(self, params: dict[str, Any]) -> str:
        """Grasp with gripper.

        Parameters
        ----------
        width:
            Target grasp width in meters (0 = fully closed, 0.08 = fully open).
        force:
            Grasping force in Newtons.
        speed:
            Closing speed in m/s.
        """
        if not self.is_connected():
            return self._conn_error()

        try:
            width = float(params.get("width", 0.0))
            force = float(params.get("force", self.default_gripper_force))
            speed = float(params.get("speed", 0.1))
        except (TypeError, ValueError) as exc:
            return self._error_result(f"invalid grasp parameters: {exc}")

        width = max(0.0, min(width, 0.08))
        force = max(0.0, min(force, 70.0))

        try:
            # Home gripper for accurate positioning
            self._gripper.homing()
            time.sleep(1.0)

            # Perform grasp
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

    def _execute_move_gripper(self, params: dict[str, Any]) -> str:
        """Move gripper fingers to target width.

        Parameters
        ----------
        width:
            Target width in meters.
        speed:
            Moving speed in m/s.
        """
        if not self.is_connected():
            return self._conn_error()

        try:
            width = float(params.get("width", 0.04))
            speed = float(params.get("speed", 0.1))
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

    def _execute_get_state(self, params: dict[str, Any]) -> str:
        """Return current robot state snapshot."""
        if not self.is_connected():
            return self._conn_error()
        try:
            state = self._robot.read_once()
            gripper_state = self._gripper.read_once()

            ee_pos = [state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14]]

            return (
                f"Robot State:\n"
                f"  Joint positions: {[f'{v:.4f}' for v in state.q]}\n"
                f"  Joint velocities: {[f'{v:.4f}' for v in state.dq]}\n"
                f"  External torques: {[f'{v:.4f}' for v in state.tau_ext_hat_filtered]}\n"
                f"  EE position: x={ee_pos[0]:.4f}, y={ee_pos[1]:.4f}, z={ee_pos[2]:.4f}\n"
                f"  Gripper width: {gripper_state.width:.4f}m, grasped={gripper_state.is_grasped}"
            )
        except Exception as exc:
            return self._error_result(f"get_state failed: {exc}")

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _stop_motion(self, status: str, last_error: str | None = None) -> str:
        try:
            if self._active_control is not None:
                self._robot.stop()
                self._active_control = None
        except Exception:
            pass
        self._set_arm_state(mode="idle", status=status, last_error=last_error)
        return "Motion stopped." if status == "stopped" else f"Motion {status}."

    def _conn_error(self) -> str:
        details = self._robot_state().get("connection_state", {}).get("last_error")
        suffix = f" Details: {details}" if details else ""
        return (
            "Connection error: robot is not connected. "
            "Run connect_robot first." + suffix
        )

    def _mark_arm_failed(self, reason: str) -> None:
        self._set_arm_state(mode="idle", status="failed", last_error=reason)

    def _error_result(self, reason: str) -> str:
        self._mark_arm_failed(reason)
        self._stop_motion("failed", last_error=reason)
        return f"Error: {reason}"

    @staticmethod
    def _clip(value: float, maximum: float) -> float:
        return max(-maximum, min(maximum, value))

    def _validate_robot_id(self, params: dict[str, Any]) -> None:
        requested = str(params.get("robot_id", "")).strip()
        if requested and requested != self.robot_id:
            raise ValueError(
                f"robot_id mismatch: requested={requested}, configured={self.robot_id}"
            )

    def _robot_state(self) -> dict[str, Any]:
        robots = self._runtime_state.setdefault("robots", {})
        if self.robot_id not in robots:
            robots[self.robot_id] = self._make_robot_state()
        return robots[self.robot_id]

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

    def _update_joint_state_from_state(self, state) -> None:
        try:
            state_dict = self._robot_state()
            joint = dict(state_dict.get("joint_state", {}))
            joint["q"] = [float(v) for v in state.q]
            joint["dq"] = [float(v) for v in state.dq]
            joint["tau"] = [float(v) for v in state.tau_ext_hat_filtered]
            joint["stamp"] = self._stamp()
            state_dict["joint_state"] = joint
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

    def _inc_reconnect_attempts(self) -> None:
        state = self._robot_state()
        conn = dict(state.get("connection_state", {}))
        conn["reconnect_attempts"] = int(conn.get("reconnect_attempts", 0)) + 1
        state["connection_state"] = conn

    def _touch_heartbeat(self) -> None:
        state = self._robot_state()
        conn = dict(state.get("connection_state", {}))
        conn["last_heartbeat"] = self._stamp()
        state["connection_state"] = conn

    @staticmethod
    def _stamp() -> str:
        return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")