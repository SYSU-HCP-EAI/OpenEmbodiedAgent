"""
hal/drivers/backends/franky_backend.py

franky-control 后端实现
franky-control Backend Implementation

通过 franky-control (https://github.com/TimSchneider42/franky) 控制 Franka 机器人。
Control Franka robot via franky-control.

franky-control 是一个高层次运动库，基于 libfranka，提供更简洁的 API。
franky-control is a high-level motion library based on libfranka with a simpler API.

Author: PhyAgentOS Team
"""

from __future__ import annotations

import copy
from datetime import datetime, timezone
from typing import Any

from hal.drivers.franka_backends.franka_backend import (
    BackendCapability,
    BackendInfo,
    ConnectionStatus,
    FrankaBackend,
)


class FrankyBackend(FrankaBackend):
    """
    franky-control 后端 / franky-control Backend

    franky-control 是一个高层运动控制库，对 libfranka 进行了封装，
    提供了更简洁的 API 和更宽松的版本兼容性。

    franky-control is a high-level motion control library that wraps libfranka,
    providing a simpler API and more relaxed version compatibility.

    Parameters
    ----------
    ip:
        Control Box IP address (Shop Floor Interface).
    robot_id:
        Logical identifier for this robot instance.
    reconnect_policy:
        One of "auto" or "manual". Default "auto".
    version:
        franky-control version to use. If None, uses latest compatible.
        由用户根据 Robot System Version 选择合适的版本。
        User should select appropriate version based on Robot System Version.
        See: docs/user_manual/appendix/franka_compatibility.md
    """

    def __init__(
        self,
        ip: str = "172.16.0.2",
        robot_id: str = "franka_franky",
        reconnect_policy: str = "auto",
        version: str | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(
            ip=ip,
            robot_id=robot_id,
            reconnect_policy=reconnect_policy,
            **kwargs,
        )

        self._version = version
        self._robot: Any | None = None
        self._gripper: Any | None = None
        self._runtime_state: dict[str, Any] = {
            "robots": {self._robot_id: self._make_robot_state()}
        }

    # ── FrankaBackend 接口实现 / FrankaBackend Interface Implementation ──────

    @property
    def backend_name(self) -> str:
        return "franky"

    @property
    def backend_version(self) -> str | None:
        """获取 franky-control 版本 / Get franky-control version."""
        try:
            import franky
            return getattr(franky, "__version__", None)
        except ImportError:
            return self._version

    def is_available(self) -> bool:
        """检查 franky-control 是否已安装 / Check if franky-control is installed."""
        try:
            import franky
            return True
        except ImportError:
            return False

    def can_connect(self) -> bool:
        """检查是否可以连接到机器人 / Check if can connect to robot."""
        if not self.is_available():
            self._last_error = "franky-control not installed"
            return False

        if self._robot is not None:
            return True

        try:
            import franky
            robot = franky.Robot(self._ip)
            robot.close()
            return True
        except Exception as e:
            self._last_error = str(e)
            return False

    def connect(self) -> bool:
        """连接到机器人 / Connect to robot."""
        if self._robot is not None:
            self._set_connection_status("connected", last_error=None)
            return True

        try:
            import franky

            self._robot = franky.Robot(self._ip)
            self._gripper = franky.Gripper(self._ip)

            self._set_connection_status("connected", last_error=None)
            self._set_arm_state(mode="idle", status="idle", last_error=None)
            return True

        except ImportError:
            self._set_connection_status(
                "error",
                last_error="franky-control not installed. Run: pip install git+https://github.com/TimSchneider42/franky.git",
            )
            self._robot = None
            self._gripper = None
            return False
        except Exception as exc:
            error_msg = str(exc)
            self._set_connection_status("error", last_error=error_msg)
            self._robot = None
            self._gripper = None
            return False

    def disconnect(self) -> None:
        """断开与机器人的连接 / Disconnect from robot."""
        if self._robot is None:
            self._set_connection_status("disconnected", last_error=None)
            self._set_arm_state(mode="idle", status="idle", last_error=None)
            return

        try:
            self._robot.close()
        except Exception:
            pass

        self._robot = None
        self._gripper = None
        self._set_connection_status("disconnected", last_error=None)
        self._set_arm_state(mode="idle", status="idle", last_error=None)

    def is_connected(self) -> bool:
        """检查是否已连接 / Check if connected."""
        return self._robot is not None

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
            import franky
            from franky import Pose

            target_pose = Pose(
                position=[x, y, z],
                orientation=[roll, pitch, yaw],
            )

            robot = self._robot
            motion = franky.cartesian_motion(robot, target_pose, speed=speed)
            robot.execute_motion(motion)

            self._set_arm_state(mode="cartesian_pose", status="arrived", last_error=None)
            return (
                f"Arm moved to x={x:.3f}m, y={y:.3f}m, z={z:.3f}m, "
                f"r={roll:.3f}p={pitch:.3f}y={yaw:.3f} rad."
            )

        except ImportError:
            return self._error_result("franky-control not available")
        except Exception as exc:
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
            import franky

            if isinstance(joints, dict):
                q = [joints.get(f"q{i}", 0.0) for i in range(1, 8)]
            else:
                q = list(joints)

            robot = self._robot
            motion = franky.joint_motion(robot, q, speed=speed)
            robot.execute_motion(motion)

            self._set_arm_state(mode="joint_position", status="arrived", last_error=None)
            return f"Arm moved to joint configuration: {[f'{v:.3f}' for v in q]}."

        except ImportError:
            return self._error_result("franky-control not available")
        except Exception as exc:
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
            width = max(0.0, min(float(width), 0.08))
            force = max(0.0, min(float(force), 70.0))

            self._gripper.grasp(width, force)

            self._set_arm_state(mode="idle", status="grasped", last_error=None)
            return f"Gripper grasped at width={width:.3f}m, force={force:.1f}N."

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
            width = max(0.0, min(float(width), 0.08))
            self._gripper.move(width)

            self._set_arm_state(mode="idle", status="idle", last_error=None)
            return f"Gripper moved to width={width:.3f}m."

        except Exception as exc:
            return self._error_result(f"move_gripper failed: {exc}")

    def get_robot_state(self) -> dict[str, Any]:
        """获取机器人当前状态 / Get current robot state."""
        if not self.is_connected():
            return {"error": "robot not connected"}

        try:
            state = self._robot.current_state()
            return {
                "joint_positions": list(state.q),
                "joint_velocities": list(state.dq),
                "ee_position": {
                    "x": state.O_T_EE[12],
                    "y": state.O_T_EE[13],
                    "z": state.O_T_EE[14],
                },
                "gripper_width": self._gripper.width(),
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
            robot_system_version=None,  # franky doesn't expose this directly
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
            # Note: franky does NOT support real-time 1kHz control
            # Note: franky does NOT support force/impedance control modes
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
        return f"Error: {reason}"

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
                "backend": "franky",
                "last_heartbeat": None,
                "last_error": None,
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

    def _set_connection_status(self, status: str, last_error: str | None) -> None:
        state = self._robot_state()
        conn = dict(state.get("connection_state", {}))
        conn.update(
            {
                "status": status,
                "host": self._ip,
                "backend": "franky",
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

    @staticmethod
    def _stamp() -> str:
        return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
