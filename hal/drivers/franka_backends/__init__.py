"""
hal/drivers/backends/__init__.py

Franka 后端模块 - 支持多后端架构
Franka Backend Module - Supports Multi-Backend Architecture

不同后端实现可以通过协商自动选择，或由用户强制指定。
Different backend implementations can be auto-negotiated or explicitly selected by user.
"""

from hal.drivers.franka_backends.franka_backend import (
    FrankaBackend,
    BackendCapability,
    BackendInfo,
    ConnectionStatus,
)

from hal.drivers.franka_backends.pylibfranka_backend import (
    PylibfrankaBackend,
)

from hal.drivers.franka_backends.franky_backend import (
    FrankyBackend,
)

from hal.drivers.franka_backends.negotiator import (
    BackendNegotiator,
    create_negotiator,
)

__all__ = [
    # 抽象接口 / Abstract Interface
    "FrankaBackend",
    "BackendCapability",
    "BackendInfo",
    "ConnectionStatus",
    # 后端实现 / Backend Implementations
    "PylibfrankaBackend",
    "FrankyBackend",
    # 协商器 / Negotiator
    "BackendNegotiator",
    "create_negotiator",
]
