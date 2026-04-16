"""
hal/drivers/franka_multi_backend_driver.py

Franka 多后端驱动 - 自动协商后端选择
Franka Multi-Backend Driver - Auto-negotiates backend selection

这是一个统一的 Franka 驱动接口，支持多个后端实现。
This is a unified Franka driver interface supporting multiple backend implementations.

当连接到机器人时，驱动会自动尝试按优先级顺序连接每个后端，
直到第一个成功连接的后端被使用。

When connecting to a robot, the driver automatically tries to connect to each
backend in priority order until the first successful connection is used.

支持的后端 / Supported Backends:
    - franky: 基于 libfranka 的高层运动库 (franky-control)
    - pylibfranka: libfranka 官方 Python 绑定

Architecture:
    FrankaMultiBackendDriver
            │
            └── BackendNegotiator
                    │
                    ├── FrankyBackend
                    │
                    └── PylibfrankaBackend

Author: PhyAgentOS Team
"""

from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any

from hal.base_driver import BaseDriver
from hal.drivers.franka_backends import (
    BackendNegotiator,
    BackendCapability,
    create_negotiator,
)


_PROFILES_DIR = Path(__file__).resolve().parent.parent / "profiles"


class FrankaMultiBackendDriver(BaseDriver):
    """
    Franka 多后端驱动 / Franka Multi-Backend Driver

    统一的 Franka Research 3 驱动，支持通过后端协商自动选择控制接口。
    Unified Franka Research 3 driver supporting auto-negotiated backend selection.

    Parameters
    ----------
    ip:
        Control Box IP address (Shop Floor Interface).
        Default: 172.16.0.2
    robot_id:
        Logical identifier for this robot instance.
    backend_priority:
        后端优先级列表，按优先级排序。
        例如: ["franky", "pylibfranka"]
        Backend priority list, ordered by priority.
    force_backend:
        强制使用特定后端。如果为 None，则自动协商。
        Force use of specific backend. If None, auto-negotiate.
        可选值: "franky", "pylibfranka"
    reconnect_policy:
        重连策略: "auto" 或 "manual". Default: "auto"
    auto_discover:
        如果 True 且未提供 IP，自动发现 Control Box.
    control_rate:
        控制频率 (Hz). Default: 250
    realtime_mode:
        如果 True，使用实时模式 (1kHz). Default: False
    default_gripper_force:
        默认夹爪力 (N). Default: 20.0

    Example
    -------
    >>> driver = FrankaMultiBackendDriver(ip="172.16.0.2")
    >>> result = driver.execute_action("connect_robot", {})
    >>> print(result)
    Using backend: franky
    Robot connection established.
    >>> driver.execute_action("move_to", {"x": 0.3, "y": 0.2, "z": 0.5, ...})
    """

    def __init__(
        self,
        ip: str | None = None,
        robot_id: str | None = None,
        backend_priority: list[str] | None = None,
        force_backend: str | None = None,
        reconnect_policy: str = "auto",
        auto_discover: bool = True,
        control_rate: float = 250.0,
        realtime_mode: bool = False,
        default_gripper_force: float = 20.0,
        **_kwargs: Any,
    ) -> None:
        # 解析 IP / Resolve IP
        self._ip = self._resolve_ip(ip, auto_discover)

        # 生成 robot_id / Generate robot_id
        self.robot_id = (
            robot_id.strip() if robot_id is not None
            else os.environ.get("PAOS_FRANKA_ROBOT_ID", "franka_multi_001")
        ).strip()

        self._reconnect_policy = reconnect_policy
        self._control_rate = control_rate
        self._realtime_mode = realtime_mode
        self._default_gripper_force = default_gripper_force

        # 创建协商器 / Create negotiator
        self._negotiator = create_negotiator(
            ip=self._ip,
            backend_priority=backend_priority,
            force_backend=force_backend,
            reconnect_policy=reconnect_policy,
            control_rate=control_rate,
            realtime_mode=realtime_mode,
            default_gripper_force=default_gripper_force,
        )

        # 内部状态 / Internal state
        self._objects: dict[str, dict] = {}
        self._runtime_state: dict[str, Any] = {
            "robots": {
                self.robot_id: {
                    "connection_state": {
                        "status": "disconnected",
                        "backend": None,
                        "host": self._ip,
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
                    },
                    "joint_state": {
                        "q": [0.0] * 7,
                        "dq": [0.0] * 7,
                        "tau": [0.0] * 7,
                    },
                    "arm_state": {
                        "mode": "idle",
                        "status": "idle",
                        "last_error": None,
                    },
                    "gripper_state": {
                        "width": 0.0,
                        "max_width": 0.08,
                        "force": 0.0,
                        "is_grasped": False,
                    },
                }
            }
        }

    def _resolve_ip(self, ip: str | None, auto_discover: bool) -> str:
        """解析机器人 IP / Resolve robot IP."""
        if ip is not None and ip.strip():
            return ip.strip()

        env_ip = os.environ.get("PAOS_FRANKA_IP", "").strip()
        if env_ip:
            return env_ip

        if auto_discover:
            # 简单的自动发现 / Simple auto-discovery
            import socket
            known_ips = ["172.16.0.2", "192.168.1.1", "172.16.0.1", "192.168.0.1"]
            for candidate in known_ips:
                try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(1.0)
                    result = sock.connect_ex((candidate, 7512))
                    sock.close()
                    if result == 0:
                        return candidate
                except Exception:
                    continue

        return "172.16.0.2"

    # ── BaseDriver 接口 / BaseDriver Interface ─────────────────────────────

    def get_profile_path(self) -> Path:
        """获取机器人 profile 路径 / Get robot profile path."""
        return _PROFILES_DIR / "franka_research3.md"

    def load_scene(self, scene: dict[str, dict]) -> None:
        """加载场景物体 / Load scene objects."""
        self._objects = dict(scene)

    def execute_action(self, action_type: str, params: dict) -> str:
        """
        执行动作 / Execute action

        此方法通过活动后端执行动作。
        This method executes actions via the active backend.
        """
        # 验证 robot_id / Validate robot_id
        requested_id = str(params.get("robot_id", "")).strip()
        if requested_id and requested_id != self.robot_id:
            return f"robot_id mismatch: requested={requested_id}, configured={self.robot_id}"

        try:
            # 处理连接相关动作 / Handle connection-related actions
            if action_type == "connect_robot":
                return self._execute_connect()
            elif action_type == "check_connection":
                return self._execute_check_connection()
            elif action_type == "disconnect_robot":
                return self._execute_disconnect()
            elif action_type == "stop":
                return self._execute_stop()
            elif action_type == "get_robot_state":
                return self._execute_get_state()

            # 检查是否已连接 / Check if connected
            if not self._negotiator.active_backend:
                return "Error: No active backend. Call connect_robot first."

            # 执行运动动作 / Execute motion actions
            if action_type == "move_to":
                return self._execute_move_to(params)
            elif action_type == "move_joints":
                return self._execute_move_joints(params)
            elif action_type == "grasp":
                return self._execute_grasp(params)
            elif action_type == "move_gripper":
                return self._execute_move_gripper(params)

            return f"Unknown action: {action_type}"

        except Exception as exc:
            return f"Error: {action_type} failed: {exc}"

    def get_scene(self) -> dict[str, dict]:
        """获取当前场景 / Get current scene."""
        return dict(self._objects)

    # ── 连接动作执行 / Connection Action Execution ───────────────────────────

    def _execute_connect(self) -> str:
        """执行连接 / Execute connect."""
        if self._negotiator.active_backend:
            return f"Already connected via {self._negotiator.active_backend_name}"

        success, result = self._negotiator.connect()

        if success:
            self._update_connection_state("connected")
            return f"Using backend: {result}\nRobot connection established."
        else:
            self._update_connection_state("error", last_error=result)
            return f"Connection failed: {result}"

    def _execute_check_connection(self) -> str:
        """执行连接检查 / Execute connection check."""
        if self._negotiator.active_backend:
            if self._negotiator.active_backend.is_connected():
                return f"connected (backend: {self._negotiator.active_backend_name})"
            else:
                # 尝试重连 / Try to reconnect
                if self._reconnect_policy == "auto":
                    success, _ = self._negotiator.connect()
                    if success:
                        return f"connected (backend: {self._negotiator.active_backend_name})"
        return "disconnected"

    def _execute_disconnect(self) -> str:
        """执行断开连接 / Execute disconnect."""
        self._negotiator.disconnect()
        self._update_connection_state("disconnected")
        return "Robot connection closed."

    def _execute_stop(self) -> str:
        """执行停止 / Execute stop."""
        if self._negotiator.active_backend:
            if hasattr(self._negotiator.active_backend, "_stop_motion"):
                return self._negotiator.active_backend._stop_motion("stopped")
        return "Motion stopped."

    def _execute_get_state(self) -> str:
        """执行获取状态 / Execute get state."""
        if not self._negotiator.active_backend:
            return "Error: No active backend. Call connect_robot first."

        state = self._negotiator.active_backend.get_robot_state()
        if isinstance(state, dict) and "error" in state:
            return f"Error: {state['error']}"

        if isinstance(state, dict):
            lines = ["Robot State:"]
            if "joint_positions" in state:
                lines.append(f"  Joint positions: {[f'{v:.4f}' for v in state['joint_positions']]}")
            if "ee_position" in state:
                ee = state["ee_position"]
                lines.append(f"  EE position: x={ee.get('x', 0):.4f}, y={ee.get('y', 0):.4f}, z={ee.get('z', 0):.4f}")
            if "gripper_width" in state:
                lines.append(f"  Gripper width: {state['gripper_width']:.4f}m")
            return "\n".join(lines)

        return str(state)

    # ── 运动动作执行 / Motion Action Execution ──────────────────────────────

    def _execute_move_to(self, params: dict) -> str:
        """执行 move_to / Execute move_to."""
        if not self._negotiator.active_backend:
            return "Error: No active backend. Call connect_robot first."

        x = params.get("x", 0.0)
        y = params.get("y", 0.0)
        z = params.get("z", 0.0)
        roll = params.get("roll", 0.0)
        pitch = params.get("pitch", 0.0)
        yaw = params.get("yaw", 0.0)
        speed = params.get("speed", 0.05)

        return self._negotiator.active_backend.move_to(
            x, y, z, roll, pitch, yaw, speed
        )

    def _execute_move_joints(self, params: dict) -> str:
        """执行 move_joints / Execute move_joints."""
        if not self._negotiator.active_backend:
            return "Error: No active backend. Call connect_robot first."

        if "joints" in params and isinstance(params["joints"], dict):
            joints = params["joints"]
        else:
            joints = {f"q{i}": params.get(f"q{i}", 0.0) for i in range(1, 8)}

        speed = params.get("speed", 0.05)
        return self._negotiator.active_backend.move_joints(joints, speed)

    def _execute_grasp(self, params: dict) -> str:
        """执行 grasp / Execute grasp."""
        if not self._negotiator.active_backend:
            return "Error: No active backend. Call connect_robot first."

        width = params.get("width", 0.0)
        force = params.get("force", self._default_gripper_force)
        speed = params.get("speed", 0.1)

        return self._negotiator.active_backend.grasp(width, force, speed)

    def _execute_move_gripper(self, params: dict) -> str:
        """执行 move_gripper / Execute move_gripper."""
        if not self._negotiator.active_backend:
            return "Error: No active backend. Call connect_robot first."

        width = params.get("width", 0.04)
        speed = params.get("speed", 0.1)

        return self._negotiator.active_backend.move_gripper(width, speed)

    # ── 状态更新 / State Updates ───────────────────────────────────────────

    def _update_connection_state(
        self,
        status: str,
        backend: str | None = None,
        last_error: str | None = None,
    ) -> None:
        """更新连接状态 / Update connection state."""
        robot_state = self._runtime_state["robots"].get(self.robot_id)
        if robot_state:
            conn_state = robot_state.get("connection_state", {})
            conn_state["status"] = status
            if backend:
                conn_state["backend"] = backend
            if last_error:
                conn_state["last_error"] = last_error

    # ── 公共方法 / Public Methods ───────────────────────────────────────────

    def connect(self) -> bool:
        """连接到机器人 (直接方法) / Connect to robot (direct method)."""
        success, _ = self._negotiator.connect()
        if success:
            self._update_connection_state("connected", self._negotiator.active_backend_name)
        return success

    def disconnect(self) -> None:
        """断开与机器人的连接 / Disconnect from robot."""
        self._negotiator.disconnect()
        self._update_connection_state("disconnected")

    def is_connected(self) -> bool:
        """检查是否已连接 / Check if connected."""
        return (
            self._negotiator.active_backend is not None
            and self._negotiator.active_backend.is_connected()
        )

    def get_active_backend_name(self) -> str | None:
        """获取活动后端名称 / Get active backend name."""
        return self._negotiator.active_backend_name

    def get_backend_capabilities(self) -> set[BackendCapability]:
        """获取活动后端的能力 / Get active backend capabilities."""
        return self._negotiator.get_capabilities()

    def get_runtime_state(self) -> dict[str, Any]:
        """获取运行时状态 / Get runtime state."""
        return copy.deepcopy(self._runtime_state)

    def get_diagnostics(self) -> dict[str, Any]:
        """获取诊断信息 / Get diagnostics."""
        return self._negotiator.get_connection_diagnostics()

    def close(self) -> None:
        """关闭驱动 / Close driver."""
        self.disconnect()

    # ── 属性 / Properties ──────────────────────────────────────────────────

    @property
    def ip(self) -> str:
        """获取机器人 IP / Get robot IP."""
        return self._ip

    @property
    def backend_priority(self) -> list[str]:
        """获取后端优先级 / Get backend priority."""
        return [b.backend_name for b in self._negotiator.backends]
