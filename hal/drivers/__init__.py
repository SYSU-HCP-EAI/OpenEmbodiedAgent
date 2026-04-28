"""
hal/drivers/__init__.py

Driver registry — maps short names to fully-qualified class paths.

To register a new driver, add one entry to ``DRIVER_REGISTRY`` and create
the corresponding module under ``hal/drivers/``.  External drivers can also
be installed as plugins and resolved from the local PhyAgentOS plugin registry.
"""

from __future__ import annotations

import importlib
from typing import Any

from hal.base_driver import BaseDriver
from hal.plugins import activate_external_driver, list_external_drivers, resolve_external_driver

# ── Registry ────────────────────────────────────────────────────────────────
# Format:  "short_name": "module_path.ClassName"

DRIVER_REGISTRY: dict[str, str] = {
    "simulation":  "hal.drivers.simulation_driver.SimulationDriver",
    "multi_robot_unified_isaac": "hal.drivers.multi_robot_unified_isaac_driver.MultiRobotUnifiedIsaacDriver",
    # Preferred explicit name: scoped to InternUtopia simulation family.
    "multi_robot_simulation_internutopia": "hal.drivers.multi_robot_simulation_driver.MultiRobotSimulationInternutopiaDriver",
    # Backward-compatible alias; keep temporarily to avoid breaking local configs.
    "multi_robot_simulation": "hal.drivers.multi_robot_simulation_driver.MultiRobotSimulationDriver",
    "go2_edu":     "hal.drivers.go2_driver.Go2Driver",
    "xlerobot_2wheels_remote": "hal.drivers.xlerobot_2wheels_remote_driver.XLerobot2WheelsRemoteDriver",
    "pipergo2_manipulation": "hal.drivers.pipergo2_manipulation_driver.PiperGo2ManipulationDriver",
    # Franka Research 3 驱动
    # Franka Research 3 Driver
    "franka_research3": "hal.drivers.franka_driver.FrankaDriver",          # 原始 pylibfranka 驱动
    "franka_multi": "hal.drivers.franka_multi_backend_driver.FrankaMultiBackendDriver",  # 多后端协商驱动
    # Franka Simulation 驱动 (Isaac Sim)
    "franka_simulation": "hal.drivers.franka_simulation_driver.FrankaSimulationDriver",  # Isaac Sim 仿真驱动

    # G1 Simulation 驱动 (Isaac Sim) - 人形机器人 locomotion
    "g1_simulation": "hal.drivers.g1_simulation_driver.G1SimulationDriver",  # Isaac Sim 仿真驱动
    # Future drivers — uncomment when implemented:
    # "desktop_pet": "hal.drivers.desktop_pet_driver.DesktopPetDriver",
    # "dobot_nova5": "hal.drivers.dobot_driver.DobotDriver",
}


def load_driver(name: str, **kwargs: Any) -> BaseDriver:
    """Dynamically import and instantiate a driver by its short name.

    Parameters
    ----------
    name:
        Key in ``DRIVER_REGISTRY`` (e.g. ``"simulation"``).
    **kwargs:
        Passed through to the driver constructor (e.g. ``gui=True``).

    Raises
    ------
    KeyError
        If *name* is not in the registry.
    ImportError
        If the driver module cannot be imported (missing dependency).
    """
    dotted = DRIVER_REGISTRY.get(name)
    if dotted is None:
        spec = resolve_external_driver(name)
        if spec is None:
            available = ", ".join(list_drivers())
            raise KeyError(
                f"Unknown driver {name!r}. Available drivers: {available}"
            )
        activate_external_driver(spec)
        dotted = spec.dotted_path
    module_path, class_name = dotted.rsplit(".", 1)
    module = importlib.import_module(module_path)
    cls = getattr(module, class_name)

    if not (isinstance(cls, type) and issubclass(cls, BaseDriver)):
        raise TypeError(f"{dotted} is not a BaseDriver subclass")

    return cls(**kwargs)


def list_drivers() -> list[str]:
    """Return sorted list of registered driver names."""
    return sorted(set(DRIVER_REGISTRY) | set(list_external_drivers()))
