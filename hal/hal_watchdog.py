#!/usr/bin/env python3
"""
hal/hal_watchdog.py

HAL Watchdog — polls ACTION.md for commands, dispatches them to the
active driver, and writes updated state back to ENVIRONMENT.md.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from hal.simulation.scene_io import (
    load_environment_doc,
    load_scene_from_md,
    merge_environment_doc,
    save_environment_doc,
)
from PhyAgentOS.utils.action_queue import (
    dump_action_document,
    first_pending_action,
    infer_terminal_status,
    normalize_action_document,
    parse_action_markdown,
)


def _log(msg: str) -> None:
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[HAL Watchdog {ts}] {msg}", flush=True)


def parse_action(content: str) -> dict | None:
    """Extract the first pending action from ACTION.md content."""
    payload = parse_action_markdown(content)
    if payload is None:
        return None
    document = normalize_action_document(payload)
    if document is None:
        return None
    pending = first_pending_action(document)
    if pending is None:
        return None
    _, action = pending
    return action


def _load_scene(path: Path) -> dict[str, dict]:
    return load_scene_from_md(path)


def load_driver_config(path: Path | None) -> dict[str, object]:
    """Load a driver config JSON object for transparent kwargs passthrough."""
    if path is None:
        return {}
    if not path.exists():
        raise FileNotFoundError(f"driver-config file not found: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"failed to parse driver-config JSON: {path}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"driver-config must be a JSON object: {path}")
    return data


def _save_scene(driver, path: Path, scene: dict[str, dict], registry=None) -> None:
    existing = load_environment_doc(path)
    runtime_state = {}
    runtime_getter = getattr(driver, "get_runtime_state", None)
    if callable(runtime_getter):
        runtime_state = runtime_getter() or {}
    updated = merge_environment_doc(
        existing,
        objects=scene,
        robots=runtime_state.get("robots"),
        scene_graph=runtime_state.get("scene_graph"),
        map_data=runtime_state.get("map"),
        tf_data=runtime_state.get("tf"),
        updated_at=datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z"),
    )
    save_environment_doc(path, updated)
    if registry is not None and getattr(registry, "is_fleet", False):
        registry.write_robot_index()


def _ensure_connection(driver) -> None:
    connect = getattr(driver, "connect", None)
    is_connected = getattr(driver, "is_connected", None)
    if callable(connect) and callable(is_connected) and not is_connected():
        connect()


def _refresh_health(driver, env_file: Path, registry=None) -> None:
    health_check = getattr(driver, "health_check", None)
    if callable(health_check):
        health_check()
    _save_scene(driver, env_file, driver.get_scene(), registry=registry)


def _install_profile(driver, workspace: Path) -> None:
    """Copy the driver's EMBODIED.md profile into the workspace."""
    src = driver.get_profile_path()
    dst = workspace / "EMBODIED.md"
    if src.exists():
        workspace.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        _log(f"Profile installed: {src.name} -> {dst}")
    else:
        _log(f"WARNING: profile not found at {src}")


def _resolve_watchdog_topology(
    workspace: Path | None,
    driver_name: str,
    robot_id: str | None,
):
    if not robot_id:
        if workspace is None:
            workspace = Path.home() / ".PhyAgentOS" / "workspace"
        return workspace, workspace / "ENVIRONMENT.md", driver_name, None

    from PhyAgentOS.config.loader import load_config
    from PhyAgentOS.embodiment_registry import EmbodimentRegistry

    registry = EmbodimentRegistry(load_config())
    instance = registry.require_instance(robot_id)
    return instance.workspace, registry.resolve_environment_path(robot_id=robot_id), instance.driver, registry


def watch_loop(
    workspace: Path,
    driver_name: str = "simulation",
    gui: bool = False,
    poll_interval: float = 1.0,
    *,
    driver_kwargs: dict[str, object] | None = None,
    env_file: Path | None = None,
    registry=None,
) -> None:
    """Load a driver, install its profile, then poll ACTION.md forever."""
    from hal.drivers import load_driver

    env_file = env_file or (workspace / "ENVIRONMENT.md")

    _log(f"Workspace : {workspace}")
    _log(f"Driver    : {driver_name}")
    _log(f"GUI       : {gui}")
    _log(f"Env File  : {env_file}")
    if driver_kwargs:
        _log(f"DriverCfg : {json.dumps(driver_kwargs, ensure_ascii=False, sort_keys=True)}")

    driver = load_driver(driver_name, gui=gui, **(driver_kwargs or {}))

    with driver:
        _install_profile(driver, workspace)
        _ensure_connection(driver)

        scene = _load_scene(env_file)
        driver.load_scene(scene)
        _refresh_health(driver, env_file, registry=registry)
        _log(f"Scene loaded ({len(scene)} object(s))")
        _log("Watching ACTION.md ... Ctrl+C to stop.\n")

        action_file = workspace / "ACTION.md"
        try:
            while True:
                _poll_once(driver, action_file, env_file, registry=registry)
                time.sleep(poll_interval)
        except KeyboardInterrupt:
            _log("Shutdown.")


def _poll_once(driver, action_file: Path, env_file: Path, registry=None) -> None:
    """Single poll: refresh connection state, then execute pending ACTION.md."""
    _refresh_health(driver, env_file, registry=registry)

    if not action_file.exists():
        return
    content = action_file.read_text(encoding="utf-8").strip()
    if not content:
        return

    payload = parse_action_markdown(content)
    if payload is None:
        _log("ACTION.md has content but no valid JSON - skipping.")
        return
    document = normalize_action_document(payload)
    if document is None:
        _log("ACTION.md contains unreadable action data - skipping.")
        return
    pending = first_pending_action(document)
    if pending is None:
        _log("ACTION.md has no pending actions - skipping.")
        return
    action_index, action = pending

    action_type = action.get("action_type", "unknown")
    params = action.get("parameters", {})
    _log(f"Action: {action_type!r}  params={params}")

    time.sleep(0.3)

    result = driver.execute_action(action_type, params)
    _log(f"Result: {result}")

    _save_scene(driver, env_file, driver.get_scene(), registry=registry)
    _log("ENVIRONMENT.md updated.")

    document["actions"][action_index]["status"] = infer_terminal_status(result)
    document["actions"][action_index]["result"] = result
    action_file.write_text(dump_action_document(document), encoding="utf-8")
    _log("ACTION.md updated.\n")


def main() -> None:
    from hal.drivers import list_drivers

    parser = argparse.ArgumentParser(
        description="HAL Watchdog - Physical Agent Operating System hardware layer",
    )
    parser.add_argument(
        "--driver",
        default="simulation",
        help=f"Driver name (available: {', '.join(list_drivers())})",
    )
    parser.add_argument(
        "--workspace",
        default=None,
        help="Workspace directory (single mode only; fleet mode prefers --robot-id)",
    )
    parser.add_argument("--robot-id", default=None, help="Robot instance id in fleet mode")
    parser.add_argument("--gui", action="store_true", help="Open 3-D viewer (local X display)")
    parser.add_argument(
        "--vnc",
        action="store_true",
        help=(
            "Open 3-D viewer over Xvfb/VNC. Auto-bootstraps Isaac Sim env "
            "(DISPLAY, CARB/EXP paths, setup_python_env.sh, PYTHONPATH) from "
            "the 'isaac_env' block of --driver-config. Mutually exclusive with --gui."
        ),
    )
    parser.add_argument(
        "--interval", type=float, default=1.0, help="Poll interval (seconds)",
    )
    parser.add_argument(
        "--driver-config",
        default=None,
        help="Path to a JSON object file that will be passed through to the selected driver as keyword args.",
    )
    args = parser.parse_args()

    if args.gui and args.vnc:
        print("Error: --gui and --vnc are mutually exclusive", file=sys.stderr)
        sys.exit(2)

    workspace = Path(args.workspace).expanduser().resolve() if args.workspace else None
    driver_config_path = Path(args.driver_config).expanduser().resolve() if args.driver_config else None
    try:
        driver_kwargs = load_driver_config(driver_config_path)
    except (FileNotFoundError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)

    # Pull the isaac_env block out so it is never forwarded as a driver kwarg.
    isaac_env_cfg = driver_kwargs.pop("isaac_env", None) if isinstance(driver_kwargs, dict) else None
    if args.vnc:
        from hal.simulation.isaac_bootstrap import bootstrap_isaac_env
        bootstrap_isaac_env(isaac_env_cfg, want_gui=True)

    # Either --gui (local X) or --vnc (Xvfb-backed) asks the driver/API to open
    # a viewport; the driver layer does not need to distinguish them.
    gui_effective = bool(args.gui or args.vnc)

    robot_workspace, env_file, resolved_driver, registry = _resolve_watchdog_topology(
        workspace,
        args.driver,
        args.robot_id,
    )

    if not robot_workspace.exists():
        print(f"Error: workspace not found: {robot_workspace}", file=sys.stderr)
        print("Run 'paos onboard' first.", file=sys.stderr)
        sys.exit(1)

    watch_loop(
        robot_workspace,
        driver_name=resolved_driver,
        gui=gui_effective,
        poll_interval=args.interval,
        driver_kwargs=driver_kwargs,
        env_file=env_file,
        registry=registry,
    )


if __name__ == "__main__":
    main()
