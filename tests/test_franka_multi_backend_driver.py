"""
tests/test_franka_multi_backend_driver.py

Franka Multi-Backend Driver 单元测试
Unit Tests for Franka Multi-Backend Driver

测试覆盖 / Test Coverage:
- 驱动注册验证 / Driver registration verification
- 后端协商逻辑 / Backend negotiation logic
- 多后端支持 / Multi-backend support
- 统一的 BaseDriver 接口 / Unified BaseDriver interface

运行方法 / Run:
    pytest tests/test_franka_multi_backend_driver.py -v
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch

# Ensure repo root is on sys.path
sys.path.insert(0, str(Path(__file__).parent.parent))

import pytest

from hal.drivers.franka_backends import (
    BackendNegotiator,
    BackendCapability,
    ConnectionStatus,
    FrankaBackend,
    create_negotiator,
)
from hal.drivers.franka_backends.pylibfranka_backend import PylibfrankaBackend
from hal.drivers.franka_backends.franky_backend import FrankyBackend
from hal.drivers.franka_multi_backend_driver import FrankaMultiBackendDriver


# ============================================================================
# Test Fixtures / 测试夹具
# ============================================================================

@pytest.fixture
def mock_backends() -> list[FrankaBackend]:
    """创建模拟后端列表 / Create mock backends list."""
    # 创建两个模拟后端
    backend1 = MagicMock(spec=FrankaBackend)
    backend1.backend_name = "mock_backend1"
    backend1.backend_version = "1.0.0"
    backend1.is_available.return_value = True
    backend1.can_connect.return_value = True
    backend1.connect.return_value = True
    backend1.is_connected.return_value = True
    backend1.disconnect.return_value = None
    backend1.get_capabilities.return_value = {BackendCapability.JOINT_POSITION}
    backend1.get_backend_info.return_value = MagicMock(
        name="mock_backend1",
        version="1.0.0",
        capabilities={BackendCapability.JOINT_POSITION},
    )

    backend2 = MagicMock(spec=FrankaBackend)
    backend2.backend_name = "mock_backend2"
    backend2.backend_version = "2.0.0"
    backend2.is_available.return_value = True
    backend2.can_connect.return_value = True
    backend2.connect.return_value = True
    backend2.is_connected.return_value = True
    backend2.disconnect.return_value = None
    backend2.get_capabilities.return_value = {BackendCapability.CARTESIAN_POSITION}
    backend2.get_backend_info.return_value = MagicMock(
        name="mock_backend2",
        version="2.0.0",
        capabilities={BackendCapability.CARTESIAN_POSITION},
    )

    return [backend1, backend2]


@pytest.fixture
def default_multi_driver() -> FrankaMultiBackendDriver:
    """创建使用默认参数的 FrankaMultiBackendDriver 实例."""
    return FrankaMultiBackendDriver(
        ip="172.16.0.2",
        robot_id="test_multi_001",
    )


# ============================================================================
# Test Class: 驱动注册 / Driver Registration
# ============================================================================

class TestFrankaMultiBackendDriverRegistration:
    """测试 Franka Multi-Backend Driver 的注册 / Test Franka Multi-Backend Driver registration."""

    def test_driver_is_registered(self) -> None:
        """验证 franka_multi 已正确注册 / Verify franka_multi is properly registered."""
        from hal.drivers import list_drivers
        assert "franka_multi" in list_drivers()

    def test_load_driver_returns_multi_backend_driver(self) -> None:
        """验证 load_driver 返回 FrankaMultiBackendDriver / Verify load_driver returns FrankaMultiBackendDriver."""
        from hal.drivers import load_driver
        driver = load_driver("franka_multi")
        assert isinstance(driver, FrankaMultiBackendDriver)

    def test_original_franka_driver_still_works(self) -> None:
        """验证原有的 FrankaDriver 仍然可用 / Verify original FrankaDriver still works."""
        from hal.drivers import load_driver
        from hal.drivers.franka_driver import FrankaDriver
        driver = load_driver("franka_research3")
        assert isinstance(driver, FrankaDriver)


# ============================================================================
# Test Class: 后端协商器 / Backend Negotiator
# ============================================================================

class TestBackendNegotiator:
    """测试 BackendNegotiator 的协商逻辑 / Test BackendNegotiator negotiation logic."""

    def test_negotiator_creation(self, mock_backends) -> None:
        """验证协商器创建 / Verify negotiator creation."""
        negotiator = BackendNegotiator(backends=mock_backends)
        assert negotiator.backends == mock_backends
        assert negotiator.active_backend is None

    def test_negotiator_selects_first_available(self, mock_backends) -> None:
        """验证协商器选择第一个可用的后端 / Verify negotiator selects first available backend."""
        negotiator = BackendNegotiator(backends=mock_backends)
        success, backend_name = negotiator.connect()

        assert success is True
        assert backend_name == "mock_backend1"
        assert negotiator.active_backend == mock_backends[0]

    def test_negotiator_falls_back_to_second(self, mock_backends) -> None:
        """验证协商器在第一个失败时回退到第二个 / Verify negotiator falls back to second when first fails."""
        mock_backends[0].connect.return_value = False
        mock_backends[0].can_connect.return_value = False
        mock_backends[0].last_error = "Connection failed"

        negotiator = BackendNegotiator(backends=mock_backends)
        success, backend_name = negotiator.connect()

        assert success is True
        assert backend_name == "mock_backend2"
        assert negotiator.active_backend == mock_backends[1]

    def test_negotiator_force_backend(self, mock_backends) -> None:
        """验证强制使用特定后端 / Verify forcing specific backend."""
        negotiator = BackendNegotiator(backends=mock_backends, force_backend="mock_backend2")
        success, backend_name = negotiator.connect()

        assert success is True
        assert backend_name == "mock_backend2"
        assert negotiator.active_backend == mock_backends[1]

    def test_negotiator_force_backend_not_found(self, mock_backends) -> None:
        """验证强制使用不存在后端时的错误 / Verify error when forcing non-existent backend."""
        negotiator = BackendNegotiator(backends=mock_backends, force_backend="nonexistent")
        success, error = negotiator.connect()

        assert success is False
        assert "not found" in error

    def test_negotiator_disconnect(self, mock_backends) -> None:
        """验证断开连接 / Verify disconnect."""
        negotiator = BackendNegotiator(backends=mock_backends)
        negotiator.connect()
        assert negotiator.active_backend is not None

        negotiator.disconnect()
        assert negotiator.active_backend is None

    def test_negotiator_get_available_backends(self, mock_backends) -> None:
        """验证获取可用后端列表 / Verify get available backends."""
        negotiator = BackendNegotiator(backends=mock_backends)
        available = negotiator.get_available_backends()

        assert len(available) == 2
        assert available[0] == ("mock_backend1", True, "1.0.0")
        assert available[1] == ("mock_backend2", True, "2.0.0")


# ============================================================================
# Test Class: FrankaMultiBackendDriver 初始化
# ============================================================================

class TestFrankaMultiBackendDriverInitialization:
    """测试 FrankaMultiBackendDriver 初始化 / Test FrankaMultiBackendDriver initialization."""

    def test_default_ip(self) -> None:
        """验证默认 IP / Verify default IP."""
        driver = FrankaMultiBackendDriver()
        assert driver.ip == "172.16.0.2"

    def test_custom_ip(self) -> None:
        """验证自定义 IP / Verify custom IP."""
        driver = FrankaMultiBackendDriver(ip="192.168.1.100")
        assert driver.ip == "192.168.1.100"

    def test_default_backend_priority(self) -> None:
        """验证默认后端优先级 / Verify default backend priority."""
        driver = FrankaMultiBackendDriver()
        assert driver.backend_priority == ["franky", "pylibfranka"]

    def test_custom_backend_priority(self) -> None:
        """验证自定义后端优先级 / Verify custom backend priority."""
        driver = FrankaMultiBackendDriver(backend_priority=["pylibfranka", "franky"])
        assert driver.backend_priority == ["pylibfranka", "franky"]


# ============================================================================
# Test Class: FrankaMultiBackendDriver BaseDriver 接口
# ============================================================================

class TestFrankaMultiBackendDriverBaseInterface:
    """测试 FrankaMultiBackendDriver 的 BaseDriver 接口 / Test FrankaMultiBackendDriver BaseDriver interface."""

    def test_get_profile_path(self, default_multi_driver) -> None:
        """验证 get_profile_path / Verify get_profile_path."""
        path = default_multi_driver.get_profile_path()
        assert isinstance(path, Path)
        assert "franka_research3.md" in str(path)

    def test_load_scene(self, default_multi_driver) -> None:
        """验证 load_scene / Verify load_scene."""
        scene = {"box": {"type": "box", "position": [1, 2, 3]}}
        default_multi_driver.load_scene(scene)
        assert default_multi_driver.get_scene() == scene

    def test_get_scene(self, default_multi_driver) -> None:
        """验证 get_scene / Verify get_scene."""
        scene = {"cylinder": {"type": "cylinder", "position": [0, 0, 0]}}
        default_multi_driver.load_scene(scene)
        retrieved = default_multi_driver.get_scene()
        assert retrieved == scene

    def test_execute_unknown_action_returns_error(self, default_multi_driver) -> None:
        """验证未知动作返回错误 / Verify unknown action returns error."""
        # 先连接（使用 mock）
        default_multi_driver._negotiator._active_backend = MagicMock()
        default_multi_driver._negotiator._active_backend.is_connected.return_value = True

        result = default_multi_driver.execute_action("unknown_action", {})
        assert "Unknown action" in result


# ============================================================================
# Test Class: FrankaMultiBackendDriver 连接生命周期
# ============================================================================

class TestFrankaMultiBackendDriverConnectionLifecycle:
    """测试 FrankaMultiBackendDriver 连接生命周期 / Test FrankaMultiBackendDriver connection lifecycle."""

    def test_initial_state_is_disconnected(self, default_multi_driver) -> None:
        """验证初始状态为断开 / Verify initial state is disconnected."""
        assert default_multi_driver.is_connected() is False

    def test_execute_disconnect_when_not_connected(self, default_multi_driver) -> None:
        """验证断开未连接的 driver / Verify disconnect when not connected."""
        result = default_multi_driver.execute_action("disconnect_robot", {})
        assert "closed" in result.lower()
        assert default_multi_driver.is_connected() is False


# ============================================================================
# Test Class: FrankaMultiBackendDriver 运行时状态
# ============================================================================

class TestFrankaMultiBackendDriverRuntimeState:
    """测试 FrankaMultiBackendDriver 运行时状态 / Test FrankaMultiBackendDriver runtime state."""

    def test_initial_runtime_state_has_required_keys(self, default_multi_driver) -> None:
        """验证初始运行时状态包含所有必需字段 / Verify initial runtime state has all required keys."""
        state = default_multi_driver.get_runtime_state()
        robot_state = state["robots"][default_multi_driver.robot_id]

        assert "connection_state" in robot_state
        assert "robot_pose" in robot_state
        assert "joint_state" in robot_state
        assert "arm_state" in robot_state
        assert "gripper_state" in robot_state

    def test_get_diagnostics(self, default_multi_driver) -> None:
        """验证 get_diagnostics / Verify get_diagnostics."""
        diagnostics = default_multi_driver.get_diagnostics()

        assert "force_backend" in diagnostics
        assert "active_backend" in diagnostics
        assert "available_backends" in diagnostics


# ============================================================================
# Test Class: create_negotiator 工厂函数
# ============================================================================

class TestCreateNegotiator:
    """测试 create_negotiator 工厂函数 / Test create_negotiator factory function."""

    def test_create_negotiator_default(self) -> None:
        """验证创建默认协商器 / Verify create default negotiator."""
        negotiator = create_negotiator()
        assert len(negotiator.backends) == 2
        assert negotiator.backends[0].backend_name == "franky"
        assert negotiator.backends[1].backend_name == "pylibfranka"

    def test_create_negotiator_custom_priority(self) -> None:
        """验证创建自定义优先级协商器 / Verify create custom priority negotiator."""
        negotiator = create_negotiator(backend_priority=["pylibfranka"])
        assert len(negotiator.backends) == 1
        assert negotiator.backends[0].backend_name == "pylibfranka"

    def test_create_negotiator_force_backend(self) -> None:
        """验证创建强制后端协商器 / Verify create force backend negotiator."""
        negotiator = create_negotiator(force_backend="pylibfranka")
        assert negotiator._force_backend == "pylibfranka"


# ============================================================================
# Test Class: FrankaBackend 抽象接口
# ============================================================================

class TestFrankaBackendInterface:
    """测试 FrankaBackend 抽象接口 / Test FrankaBackend abstract interface."""

    def test_backend_capability_values(self) -> None:
        """验证 BackendCapability 枚举值 / Verify BackendCapability enum values."""
        expected = [
            "joint_position",
            "cartesian_position",
            "joint_velocity",
            "cartesian_velocity",
            "force_control",
            "impedance_control",
            "realtime_control",
            "gripper_control",
            "gripper_force",
            "collision_detection",
            "self_collision",
            "external_collision",
            "auto_reconnect",
            "heartbeat",
        ]
        actual = [c.value for c in BackendCapability]
        assert set(actual) == set(expected)

    def test_connection_status_values(self) -> None:
        """验证 ConnectionStatus 枚举值 / Verify ConnectionStatus enum values."""
        expected = ["disconnected", "connecting", "connected", "error"]
        actual = [s.value for s in ConnectionStatus]
        assert set(actual) == set(expected)
