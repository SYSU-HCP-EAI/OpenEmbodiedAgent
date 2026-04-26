#!/usr/bin/env python3
"""
tests/test_franka_smoke.py

Franka Research 3 Driver 本地冒烟测试 / Local Smoke Tests for Franka Research 3 Driver

此脚本不需要真实硬件即可运行，用于验证：
- Driver 可以被正确加载
- 参数解析正确
- 基础接口工作正常

This script can run WITHOUT real hardware to verify:
- Driver loads correctly
- Parameter parsing is correct
- Basic interfaces work normally

运行方法 / Run:
    cd /path/to/PhyAgentOS
    python tests/test_franka_smoke.py
"""

from __future__ import annotations

import sys
from pathlib import Path

# 确保仓库根目录在 sys.path 中 / Ensure repo root is on sys.path
sys.path.insert(0, str(Path(__file__).parent.parent))

import time


def print_header(title: str) -> None:
    """打印测试标题 / Print test header."""
    print(f"\n{'=' * 60}")
    print(f" {title}")
    print(f"{'=' * 60}")


def print_result(name: str, passed: bool, detail: str = "") -> None:
    """打印测试结果 / Print test result.

    Parameters
    ----------
    name: 测试名称 / Test name
    passed: 是否通过 / Whether passed
    detail: 详细信息 / Detailed information
    """
    status = "✅ PASS" if passed else "❌ FAIL"
    print(f"  {status}: {name}")
    if detail:
        print(f"         {detail}")


def test_driver_registration() -> bool:
    """测试驱动注册 / Test driver registration."""
    print_header("测试 1: 驱动注册 / Driver Registration")

    try:
        from hal.drivers import list_drivers

        drivers = list_drivers()
        passed = "franka_research3" in drivers

        print_result(
            "franka_research3 已注册",
            passed,
            f"可用驱动: {', '.join(drivers)}" if passed else "驱动未找到"
        )
        return passed

    except Exception as e:
        print_result("驱动注册检查", False, str(e))
        return False


def test_driver_load() -> bool:
    """测试驱动加载 / Test driver loading."""
    print_header("测试 2: 驱动加载 / Driver Loading")

    try:
        from hal.drivers import load_driver

        driver = load_driver("franka_research3")
        print_result(
            "load_driver('franka_research3')",
            True,
            f"类型: {type(driver).__name__}"
        )
        driver.close()
        return True

    except Exception as e:
        print_result("驱动加载", False, str(e))
        return False


def test_parameter_initialization() -> bool:
    """测试参数初始化 / Test parameter initialization."""
    print_header("测试 3: 参数初始化 / Parameter Initialization")

    from hal.drivers.franka_driver import FrankaDriver

    all_passed = True

    # 测试默认参数 / Test default parameters
    driver = FrankaDriver()
    checks = [
        ("默认 IP", driver._ip == "172.16.0.2"),
        ("默认控制频率", driver.control_rate == 250.0),
        ("默认实时模式", driver.realtime_mode is False),
        ("默认 robot_id", driver.robot_id == "franka_research3_001"),
        ("默认夹爪力", driver.default_gripper_force == 20.0),
        ("默认线速度限制", driver.safe_max_linear_m_s == 0.5),
        ("默认角速度限制", driver.safe_max_angular_deg_s == 30.0),
        ("默认自动发现", driver.auto_discover is True),
        ("默认重连策略", driver.reconnect_policy == "auto"),
        ("初始状态断开", driver.is_connected() is False),
    ]

    for name, passed in checks:
        print_result(name, passed)
        all_passed = all_passed and passed

    # 测试自定义参数 / Test custom parameters
    driver2 = FrankaDriver(
        ip="192.168.1.100",
        control_rate=100.0,
        realtime_mode=True,
        robot_id="custom_franka",
        default_gripper_force=40.0,
    )

    custom_checks = [
        ("自定义 IP", driver2._ip == "192.168.1.100"),
        ("自定义控制频率", driver2.control_rate == 100.0),
        ("启用实时模式", driver2.realtime_mode is True),
        ("自定义 robot_id", driver2.robot_id == "custom_franka"),
        ("自定义夹爪力", driver2.default_gripper_force == 40.0),
    ]

    for name, passed in custom_checks:
        print_result(name, passed)
        all_passed = all_passed and passed

    return all_passed


def test_base_interface() -> bool:
    """测试 BaseDriver 接口 / Test BaseDriver interface."""
    print_header("测试 4: BaseDriver 接口 / BaseDriver Interface")

    from hal.drivers.franka_driver import FrankaDriver

    driver = FrankaDriver(ip="172.16.0.2")
    all_passed = True

    # 测试 get_profile_path / Test get_profile_path
    try:
        profile_path = driver.get_profile_path()
        checks = [
            ("get_profile_path 返回 Path", isinstance(profile_path, Path)),
            ("profile 文件存在", profile_path.exists()),
            ("profile 文件名正确", profile_path.name == "franka_research3.md"),
        ]
        for name, passed in checks:
            print_result(name, passed)
            all_passed = all_passed and passed
    except Exception as e:
        print_result("get_profile_path", False, str(e))
        all_passed = False

    # 测试 load_scene / Test load_scene
    try:
        scene = {"table": {"type": "box", "position": {"x": 1, "y": 2, "z": 0}}}
        driver.load_scene(scene)
        print_result("load_scene", True)
    except Exception as e:
        print_result("load_scene", False, str(e))
        all_passed = False

    # 测试 get_scene / Test get_scene
    try:
        result = driver.get_scene()
        checks = [
            ("get_scene 返回 dict", isinstance(result, dict)),
            ("get_scene 内容正确", result == scene),
        ]
        for name, passed in checks:
            print_result(name, passed)
            all_passed = all_passed and passed
    except Exception as e:
        print_result("get_scene", False, str(e))
        all_passed = False

    # 测试 execute_action / Test execute_action
    try:
        result = driver.execute_action("stop", {})
        checks = [
            ("execute_action 返回 str", isinstance(result, str)),
            ("stop 动作成功", "Motion stopped" in result),
        ]
        for name, passed in checks:
            print_result(name, passed)
            all_passed = all_passed and passed

        # 测试未知动作 / Test unknown action
        unknown_result = driver.execute_action("__unknown_action__", {})
        passed = "Unknown action" in unknown_result
        print_result("未知动作返回错误", passed)
        all_passed = all_passed and passed
    except Exception as e:
        print_result("execute_action", False, str(e))
        all_passed = False

    # 测试 context manager / Test context manager
    try:
        with FrankaDriver(ip="172.16.0.2") as d:
            d.load_scene({})
        print_result("context manager", True)
    except Exception as e:
        print_result("context manager", False, str(e))
        all_passed = False

    return all_passed


def test_runtime_state() -> bool:
    """测试运行时状态 / Test runtime state."""
    print_header("测试 5: 运行时状态 / Runtime State")

    from hal.drivers.franka_driver import FrankaDriver

    driver = FrankaDriver(ip="172.16.0.2", robot_id="test_robot")
    all_passed = True

    try:
        state = driver.get_runtime_state()

        # 验证必需字段 / Verify required fields
        robot_state = state["robots"]["test_robot"]
        required_keys = [
            "connection_state",
            "robot_pose",
            "joint_state",
            "arm_state",
            "gripper_state",
        ]

        for key in required_keys:
            passed = key in robot_state
            print_result(f"包含 {key}", passed)
            all_passed = all_passed and passed

        # 验证初始值 / Verify initial values
        checks = [
            ("connection_state.status = disconnected", robot_state["connection_state"]["status"] == "disconnected"),
            ("arm_state.mode = idle", robot_state["arm_state"]["mode"] == "idle"),
            ("arm_state.status = idle", robot_state["arm_state"]["status"] == "idle"),
            ("gripper_state.max_width = 0.08", robot_state["gripper_state"]["max_width"] == 0.08),
        ]

        for name, passed in checks:
            print_result(name, passed)
            all_passed = all_passed and passed

        # 验证深拷贝 / Verify deep copy
        state["robots"]["test_robot"]["connection_state"]["status"] = "modified"
        fresh = driver.get_runtime_state()
        passed = fresh["robots"]["test_robot"]["connection_state"]["status"] == "disconnected"
        print_result("运行时状态是深拷贝", passed)
        all_passed = all_passed and passed

    except Exception as e:
        print_result("运行时状态检查", False, str(e))
        all_passed = False

    return all_passed


def test_action_validation() -> bool:
    """测试动作参数校验 / Test action parameter validation."""
    print_header("测试 6: 动作参数校验 / Action Parameter Validation")

    from hal.drivers.franka_driver import FrankaDriver

    driver = FrankaDriver(ip="172.16.0.2", robot_id="test_robot")
    all_passed = True

    # 测试 robot_id 不匹配 / Test robot_id mismatch
    result = driver.execute_action("stop", {"robot_id": "wrong_robot"})
    passed = "robot_id mismatch" in result
    print_result("robot_id 不匹配检测", passed)
    all_passed = all_passed and passed

    # 测试 move_to 需要连接 / Test move_to requires connection
    # 注意：由于没有连接真实机器人，move_to 会返回连接错误
    # Note: Since we don't have a real robot connected, move_to returns connection error
    result = driver.execute_action("move_to", {"x": 0.5})
    passed = "Connection error" in result or "not connected" in result
    print_result("move_to 连接检查", passed)
    all_passed = all_passed and passed

    # 测试未知动作 / Test unknown action
    result = driver.execute_action("unknown_action", {})
    passed = "Unknown action" in result
    print_result("未知动作处理", passed)
    all_passed = all_passed and passed

    return all_passed


def test_pylibfranka_import() -> bool:
    """测试 pylibfranka 导入 / Test pylibfranka import."""
    print_header("测试 7: pylibfranka 依赖检查 / pylibfranka Dependency Check")

    try:
        import pylibfranka
        print_result(
            "pylibfranka 已安装",
            True,
            f"版本: {pylibfranka.__version__ if hasattr(pylibfranka, '__version__') else 'unknown'}"
        )
        return True
    except ImportError:
        print_result(
            "pylibfranka 未安装",
            False,
            "运行: pip install pylibfranka"
        )
        return False


def test_watchdog_config_parsing() -> bool:
    """测试 Watchdog 配置解析 / Test Watchdog config parsing."""
    print_header("测试 8: Watchdog 配置解析 / Watchdog Config Parsing")

    all_passed = True

    # 测试 driver-config 文件存在 / Test driver-config file exists
    config_path = Path(__file__).parent.parent / "examples" / "franka_research3.driver.json"

    passed = config_path.exists()
    print_result(
        "franka_research3.driver.json 存在",
        passed,
        str(config_path) if passed else "文件未找到"
    )
    all_passed = all_passed and passed

    if passed:
        import json
        try:
            with open(config_path) as f:
                config = json.load(f)

            checks = [
                ("包含 ip 字段", "ip" in config),
                ("包含 robot_id 字段", "robot_id" in config),
                ("包含 control_rate 字段", "control_rate" in config),
                ("包含 realtime_mode 字段", "realtime_mode" in config),
            ]

            for name, check in checks:
                print_result(name, check)
                all_passed = all_passed and check

            if "ip" in config:
                print_result(f"配置 IP: {config['ip']}", True)

        except Exception as e:
            print_result("配置文件解析", False, str(e))
            all_passed = False

    return all_passed


def run_all_tests() -> bool:
    """运行所有冒烟测试 / Run all smoke tests."""
    print("\n")
    print("╔" + "=" * 58 + "╗")
    print("║" + " " * 15 + "Franka Research 3 Driver" + " " * 16 + "║")
    print("║" + " " * 12 + "本地冒烟测试 / Local Smoke Tests" + " " * 11 + "║")
    print("╚" + "=" * 58 + "╝")

    tests = [
        ("驱动注册", test_driver_registration),
        ("驱动加载", test_driver_load),
        ("参数初始化", test_parameter_initialization),
        ("BaseDriver 接口", test_base_interface),
        ("运行时状态", test_runtime_state),
        ("动作参数校验", test_action_validation),
        ("pylibfranka 依赖", test_pylibfranka_import),
        ("Watchdog 配置", test_watchdog_config_parsing),
    ]

    results = []
    for name, test_func in tests:
        try:
            passed = test_func()
            results.append((name, passed))
        except Exception as e:
            print(f"\n❌ 测试 '{name}' 抛出异常:")
            print(f"   {type(e).__name__}: {e}")
            results.append((name, False))

    # 打印总结 / Print summary
    print_header("测试总结 / Test Summary")

    passed_count = sum(1 for _, p in results if p)
    total_count = len(results)

    for name, passed in results:
        status = "✅" if passed else "❌"
        print(f"  {status} {name}")

    print(f"\n通过: {passed_count}/{total_count}")

    if passed_count == total_count:
        print("\n🎉 所有测试通过！/ All tests passed!")
        print("\n下一步 / Next Steps:")
        print("  1. pytest tests/test_franka_driver.py -v  # 运行完整单元测试")
        print("  2. 连接真实机器人，执行 Watchdog 集成测试")
        print("  3. 运行: python hal/hal_watchdog.py --driver franka_research3")
    else:
        print("\n⚠️  部分测试失败，请检查上述输出 / Some tests failed, please check the output above.")

    return passed_count == total_count


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
