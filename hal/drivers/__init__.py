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
    "go2_edu":     "hal.drivers.go2_driver.Go2Driver",
    "franka_simulation": "hal.drivers.franka_simulation_driver.FrankaSimulationDriver",
    "g1_navigation": "hal.drivers.g1_navigation_driver.G1NavigationDriver",
    "g1_simulation": "hal.drivers.g1_simulation_driver.G1SimulationDriver",
    "xlerobot_sim": "hal.drivers.xlerobot_sim_driver.XLerobotSimDriver",
    "xlerobot_2wheels_remote": "hal.drivers.xlerobot_2wheels_remote_driver.XLerobot2WheelsRemoteDriver",
    "pipergo2_manipulation": "hal.drivers.pipergo2_manipulation_driver.PiperGo2ManipulationDriver",
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
    from hal.internutopia_paths import ensure_bundled_internutopia_sys_path

    ensure_bundled_internutopia_sys_path()
    try:
        from hal.simulation.og_attribute_deque_shim import try_apply_omnigraph_deque_attribute_shim

        try_apply_omnigraph_deque_attribute_shim()
    except Exception:
        pass

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
