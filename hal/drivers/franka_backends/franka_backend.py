"""
hal/drivers/backends/franka_backend.py

Franka 机器人控制后端抽象接口
Franka Robot Control Backend Abstract Interface

定义了所有 Franka 后端必须实现的接口。
Defines the interface that all Franka backends must implement.

架构说明 / Architecture:
    FrankaDriver 可以同时支持多个后端 (pylibfranka, franky, ROS2 等)。
    通过 BackendNegotiator 自动选择第一个可用的后端。
    FrankaDriver can support multiple backends (pylibfranka, franky, ROS2, etc.).
    BackendNegotiator automatically selects the first available backend.

Author: PhyAgentOS Team
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Any


class ConnectionStatus(Enum):
    """后端连接状态 / Backend connection status."""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"


class BackendCapability(Enum):
    """后端能力枚举 / Backend capability enumeration."""
    # 运动控制 / Motion Control
    JOINT_POSITION = "joint_position"          # 关节位置控制
    CARTESIAN_POSITION = "cartesian_position"  # 笛卡尔位置控制
    JOINT_VELOCITY = "joint_velocity"          # 关节速度控制
    CARTESIAN_VELOCITY = "cartesian_velocity"  # 笛卡尔速度控制
    FORCE_CONTROL = "force_control"            # 力控
    IMPEDANCE_CONTROL = "impedance_control"    # 阻抗控制

    # 实时控制 / Real-time Control
    REALTIME_CONTROL = "realtime_control"      # 实时控制 (1kHz)

    # 夹爪控制 / Gripper Control
    GRIPPER_CONTROL = "gripper_control"         # 夹爪控制
    GRIPPER_FORCE = "gripper_force"            # 夹爪力控制

    # 安全功能 / Safety Features
    COLLISION_DETECTION = "collision_detection" # 碰撞检测
    SELF_COLLISION = "self_collision"          # 自身碰撞检测
    EXTERNAL_COLLISION = "external_collision"   # 外部碰撞检测

    # 连接功能 / Connection Features
    AUTO_RECONNECT = "auto_reconnect"          # 自动重连
    HEARTBEAT = "heartbeat"                    # 心跳检测


@dataclass
class BackendInfo:
    """后端信息 / Backend information."""
    name: str                                   # 后端名称
    version: str | None                         # 版本号 (如果可知)
    capabilities: set[BackendCapability]         # 支持的能力
    connection_status: ConnectionStatus          # 连接状态
    last_error: str | None                      # 最后错误信息
    robot_system_version: str | None             # 机器人系统版本 (如果已知)
    is_available: bool                          # 后端是否可用 (依赖已安装)


class FrankaBackend(ABC):
    """
    Franka 机器人控制后端抽象基类
    Abstract base class for Franka robot control backends

    所有后端实现必须继承此类并实现所有抽象方法。
    All backend implementations must inherit from this class and implement all abstract methods.

    使用示例 / Usage:
        class MyBackend(FrankaBackend):
            def connect(self) -> bool:
                # 实现连接逻辑
                pass

        backend = MyBackend(ip="172.16.0.2")
        if backend.can_connect():
            backend.connect()
    """

    def __init__(
        self,
        ip: str,
        robot_id: str = "franka_default",
        reconnect_policy: str = "auto",
        **kwargs: Any,
    ) -> None:
        """
        初始化后端 / Initialize backend.

        Args:
            ip: 机器人 IP 地址
            robot_id: 机器人标识符
            reconnect_policy: 重连策略 ("auto" 或 "manual")
        """
        self._ip = ip
        self._robot_id = robot_id
        self._reconnect_policy = reconnect_policy
        self._connection_status = ConnectionStatus.DISCONNECTED
        self._last_error: str | None = None
        self._robot_system_version: str | None = None
        self._extra_params: dict[str, Any] = kwargs

    # ── 属性访问 / Properties ────────────────────────────────────────────────

    @property
    def ip(self) -> str:
        """获取机器人 IP / Get robot IP."""
        return self._ip

    @property
    def robot_id(self) -> str:
        """获取机器人 ID / Get robot ID."""
        return self._robot_id

    @property
    def connection_status(self) -> ConnectionStatus:
        """获取连接状态 / Get connection status."""
        return self._connection_status

    @property
    def last_error(self) -> str | None:
        """获取最后错误 / Get last error."""
        return self._last_error

    @property
    @abstractmethod
    def backend_name(self) -> str:
        """获取后端名称 / Get backend name."""
        ...

    @property
    @abstractmethod
    def backend_version(self) -> str | None:
        """获取后端版本 / Get backend version."""
        ...

    # ── 核心连接方法 / Core Connection Methods ──────────────────────────────

    @abstractmethod
    def is_available(self) -> bool:
        """
        检查后端是否可用 (依赖是否满足)
        Check if backend is available (dependencies satisfied)

        例如: pylibfranka 后端检查 pylibfranka 是否已安装
        For example: pylibfranka backend checks if pylibfranka is installed

        Returns:
            True 如果后端可用，否则 False
        """
        ...

    @abstractmethod
    def can_connect(self) -> bool:
        """
        检查是否可以连接到机器人 (不实际建立连接)
        Check if can connect to robot without actually establishing connection

        此方法进行版本兼容性检查和预连接验证。
        This method performs version compatibility check and pre-connection validation.

        Returns:
            True 如果可能连接成功，否则 False
        """
        ...

    @abstractmethod
    def connect(self) -> bool:
        """
        连接到机器人
        Connect to robot

        Returns:
            True 如果连接成功，否则 False
        """
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """断开与机器人的连接 / Disconnect from robot."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """
        检查是否已连接
        Check if connected

        Returns:
            True 如果已连接，否则 False
        """
        ...

    # ── 运动控制方法 / Motion Control Methods ──────────────────────────────

    @abstractmethod
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
        """
        移动末端执行器到笛卡尔位置
        Move end-effector to Cartesian position

        Args:
            x, y, z: 位置 (米)
            roll, pitch, yaw: 姿态角 (弧度)
            speed: 速度因子 (0-1)

        Returns:
            执行结果描述字符串
        """
        ...

    @abstractmethod
    def move_joints(
        self,
        joints: list[float] | dict[str, float],
        speed: float = 0.5,
        **kwargs: Any,
    ) -> str:
        """
        移动关节到目标位置
        Move joints to target positions

        Args:
            joints: 关节位置列表 [q1-q7] 或字典 {"q1": val, ...}
            speed: 速度因子 (0-1)

        Returns:
            执行结果描述字符串
        """
        ...

    # ── 夹爪控制方法 / Gripper Control Methods ─────────────────────────────

    @abstractmethod
    def grasp(
        self,
        width: float,
        force: float = 20.0,
        speed: float = 0.1,
        **kwargs: Any,
    ) -> str:
        """
        闭合夹爪抓取物体
        Close gripper to grasp object

        Args:
            width: 目标宽度 (米)
            force: 抓取力 (牛顿)
            speed: 闭合速度 (m/s)

        Returns:
            执行结果描述字符串
        """
        ...

    @abstractmethod
    def move_gripper(
        self,
        width: float,
        speed: float = 0.1,
        **kwargs: Any,
    ) -> str:
        """
        移动夹爪到目标宽度
        Move gripper to target width

        Args:
            width: 目标宽度 (米)
            speed: 移动速度 (m/s)

        Returns:
            执行结果描述字符串
        """
        ...

    # ── 状态查询方法 / State Query Methods ─────────────────────────────────

    @abstractmethod
    def get_robot_state(self) -> dict[str, Any]:
        """
        获取机器人当前状态
        Get current robot state

        Returns:
            包含位置、速度、力矩等信息的字典
        """
        ...

    @abstractmethod
    def get_backend_info(self) -> BackendInfo:
        """
        获取后端详细信息
        Get detailed backend information

        Returns:
            BackendInfo 对象
        """
        ...

    # ── 能力查询方法 / Capability Query Methods ────────────────────────────

    @abstractmethod
    def get_capabilities(self) -> set[BackendCapability]:
        """
        获取后端支持的能力集合
        Get set of capabilities supported by backend

        Returns:
            BackendCapability 枚举值的集合
        """
        ...

    def has_capability(self, capability: BackendCapability) -> bool:
        """检查是否具有特定能力 / Check if has specific capability."""
        return capability in self.get_capabilities()

    # ── 工具方法 / Utility Methods ─────────────────────────────────────────

    def __repr__(self) -> str:
        return (
            f"{self.__class__.__name__}("
            f"ip={self._ip}, "
            f"robot_id={self._robot_id}, "
            f"status={self._connection_status.value})"
        )
