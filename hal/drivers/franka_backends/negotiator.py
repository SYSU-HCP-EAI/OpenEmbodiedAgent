"""
hal/drivers/backends/negotiator.py

后端协商器 - 自动选择可用的 Franka 控制后端
Backend Negotiator - Automatically selects available Franka control backend

架构说明 / Architecture:
    当 FrankaDriver 需要连接机器人时，它会尝试按优先级顺序
    连接每个后端，直到第一个成功连接的后端被使用。

    When FrankaDriver needs to connect to a robot, it tries each backend
    in priority order until the first successful connection is used.

Author: PhyAgentOS Team
"""

from __future__ import annotations

import os
from typing import Any

from hal.drivers.franka_backends.franka_backend import (
    BackendCapability,
    BackendInfo,
    ConnectionStatus,
    FrankaBackend,
)


class BackendNegotiator:
    """
    后端协商器 / Backend Negotiator

    管理多个后端的优先级和选择逻辑。
    Manages priority and selection logic for multiple backends.

    Parameters
    ----------
    backends:
        后端实例列表，按优先级排序。
        Backend instances, ordered by priority.
    force_backend:
        强制使用特定后端。如果为 None，则自动协商。
        Force use of specific backend. If None, auto-negotiate.

    Example
    -------
    >>> negotiator = BackendNegotiator(
    ...     backends=[FrankyBackend(), PylibfrankaBackend()],
    ...     force_backend=None
    ... )
    >>> negotiator.connect()
    'Using backend: franky'
    """

    def __init__(
        self,
        backends: list[FrankaBackend],
        force_backend: str | None = None,
    ) -> None:
        self._backends = backends
        self._force_backend = force_backend
        self._active_backend: FrankaBackend | None = None
        self._connection_log: list[dict[str, Any]] = []

    @property
    def backends(self) -> list[FrankaBackend]:
        """获取所有后端 / Get all backends."""
        return self._backends

    @property
    def active_backend(self) -> FrankaBackend | None:
        """获取当前活动后端 / Get active backend."""
        return self._active_backend

    @property
    def active_backend_name(self) -> str | None:
        """获取当前活动后端名称 / Get active backend name."""
        return self._active_backend.backend_name if self._active_backend else None

    @property
    def connection_log(self) -> list[dict[str, Any]]:
        """获取连接日志 / Get connection log."""
        return list(self._connection_log)

    def get_available_backends(self) -> list[tuple[str, bool, str | None]]:
        """
        获取所有后端的可用性信息
        Get availability information for all backends

        Returns:
            List of (backend_name, is_available, version_or_error)
        """
        results = []
        for backend in self._backends:
            try:
                is_avail = backend.is_available()
                version = backend.backend_version
                results.append((backend.backend_name, is_avail, version))
            except Exception as e:
                results.append((backend.backend_name, False, str(e)))
        return results

    def can_connect_any(self) -> bool:
        """检查是否有任何后端可以连接 / Check if any backend can connect."""
        if self._force_backend:
            for backend in self._backends:
                if backend.backend_name == self._force_backend:
                    return backend.can_connect()
            return False

        for backend in self._backends:
            if backend.is_available() and backend.can_connect():
                return True
        return False

    def connect(self) -> tuple[bool, str | None]:
        """
        尝试连接所有后端，按优先级顺序
        Try to connect to all backends in priority order

        Returns:
            (success, active_backend_name or error_message)
        """
        self._connection_log.clear()

        if self._force_backend:
            # 强制使用特定后端 / Force use specific backend
            for backend in self._backends:
                if backend.backend_name == self._force_backend:
                    self._log_attempt(backend, "forced")
                    if backend.is_available():
                        if backend.connect():
                            self._active_backend = backend
                            return True, backend.backend_name
                        else:
                            return False, f"Force backend {self._force_backend} failed: {backend.last_error}"
                    else:
                        return False, f"Force backend {self._force_backend} not available"
            return False, f"Force backend {self._force_backend} not found"

        # 自动协商 / Auto negotiate
        for backend in self._backends:
            self._log_attempt(backend, "trying")

            if not backend.is_available():
                self._log_result(backend, False, "backend not installed")
                continue

            if not backend.can_connect():
                self._log_result(backend, False, f"cannot connect: {backend.last_error}")
                continue

            if backend.connect():
                self._active_backend = backend
                self._log_result(backend, True, "connected")
                return True, backend.backend_name
            else:
                self._log_result(backend, False, backend.last_error)

        return False, "No backend could connect. Check connection and backend installation."

    def disconnect(self) -> None:
        """断开当前活动后端的连接 / Disconnect active backend."""
        if self._active_backend:
            try:
                self._active_backend.disconnect()
            except Exception:
                pass
            self._active_backend = None

    def execute_action(self, action: str, params: dict[str, Any]) -> str:
        """
        通过活动后端执行动作 / Execute action via active backend

        Raises:
            RuntimeError: 如果没有活动后端 / If no active backend
        """
        if self._active_backend is None:
            return "Error: No active backend. Call connect() first."

        backend = self._active_backend

        if action == "connect_robot":
            return "Robot connection established." if backend.is_connected() else backend._conn_error()
        elif action == "check_connection":
            return "connected" if backend.is_connected() else "disconnected"
        elif action == "disconnect_robot":
            backend.disconnect()
            return "Robot connection closed."
        elif action == "stop":
            if hasattr(backend, "_stop_motion"):
                return backend._stop_motion("stopped")
            return "Motion stopped."
        elif action == "move_to":
            x = params.get("x", 0.0)
            y = params.get("y", 0.0)
            z = params.get("z", 0.0)
            roll = params.get("roll", 0.0)
            pitch = params.get("pitch", 0.0)
            yaw = params.get("yaw", 0.0)
            speed = params.get("speed", 0.5)
            return backend.move_to(x, y, z, roll, pitch, yaw, speed)
        elif action == "move_joints":
            joints = params.get("joints", {f"q{i}": 0.0 for i in range(1, 8)})
            speed = params.get("speed", 0.5)
            return backend.move_joints(joints, speed)
        elif action == "grasp":
            width = params.get("width", 0.0)
            force = params.get("force", 20.0)
            speed = params.get("speed", 0.1)
            return backend.grasp(width, force, speed)
        elif action == "move_gripper":
            width = params.get("width", 0.04)
            speed = params.get("speed", 0.1)
            return backend.move_gripper(width, speed)
        elif action == "get_robot_state":
            state = backend.get_robot_state()
            if isinstance(state, dict) and "error" in state:
                return f"Error: {state['error']}"
            return str(state)
        else:
            return f"Unknown action: {action}"

    def get_capabilities(self) -> set[BackendCapability]:
        """获取活动后端的能力 / Get active backend capabilities."""
        if self._active_backend:
            return self._active_backend.get_capabilities()
        return set()

    def get_backend_info(self) -> list[BackendInfo]:
        """获取所有后端的信息 / Get information for all backends."""
        return [backend.get_backend_info() for backend in self._backends]

    def get_connection_diagnostics(self) -> dict[str, Any]:
        """
        获取连接诊断信息 / Get connection diagnostics

        Returns:
            包含所有后端状态和连接尝试历史的字典
        """
        return {
            "force_backend": self._force_backend,
            "active_backend": self.active_backend_name,
            "available_backends": self.get_available_backends(),
            "connection_attempts": self._connection_log,
        }

    def _log_attempt(self, backend: FrankaBackend, reason: str) -> None:
        """记录连接尝试 / Log connection attempt."""
        self._connection_log.append({
            "backend": backend.backend_name,
            "action": reason,
            "success": None,
            "message": None,
        })

    def _log_result(self, backend: FrankaBackend, success: bool, message: str) -> None:
        """记录连接结果 / Log connection result."""
        if self._connection_log:
            self._connection_log[-1].update({
                "success": success,
                "message": message,
            })

    def __repr__(self) -> str:
        active = self.active_backend_name or "none"
        return (
            f"BackendNegotiator("
            f"active={active}, "
            f"backends={[b.backend_name for b in self._backends]})"
        )


def create_negotiator(
    ip: str = "172.16.0.2",
    backend_priority: list[str] | None = None,
    force_backend: str | None = None,
    **kwargs: Any,
) -> BackendNegotiator:
    """
    创建后端协商器工厂函数
    Backend Negotiator factory function

    Parameters
    ----------
    ip:
        Robot IP address.
    backend_priority:
        后端优先级列表。如果为 None，使用默认优先级 ["franky", "pylibfranka"]。
        Backend priority list. If None, uses default ["franky", "pylibfranka"].
    force_backend:
        强制使用特定后端。
        Force use specific backend.

    Returns:
        配置好的 BackendNegotiator 实例
    """
    from hal.drivers.franka_backends.pylibfranka_backend import PylibfrankaBackend
    from hal.drivers.franka_backends.franky_backend import FrankyBackend

    if backend_priority is None:
        # 从环境变量读取优先级 / Read priority from environment
        env_priority = os.environ.get("PAOS_FRANKA_BACKEND_PRIORITY", "")
        if env_priority:
            backend_priority = [b.strip() for b in env_priority.split(",")]
        else:
            backend_priority = ["franky", "pylibfranka"]

    # 创建后端实例 / Create backend instances
    backends: list[FrankaBackend] = []
    for name in backend_priority:
        if name == "franky":
            backends.append(FrankyBackend(ip=ip, robot_id=f"franka_{name}", **kwargs))
        elif name == "pylibfranka":
            backends.append(PylibfrankaBackend(ip=ip, robot_id=f"franka_{name}", **kwargs))
        # 可以添加更多后端 / More backends can be added here

    return BackendNegotiator(backends=backends, force_backend=force_backend)
