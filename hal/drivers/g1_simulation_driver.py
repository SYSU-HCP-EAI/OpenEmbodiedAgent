"""
G1SimulationDriver — PhyAgentOS HAL driver for G1 humanoid robot simulation.

Bridges ACTION.md to G1 navigation via internutopia sim stack.
Supports waypoint navigation for mobile manipulation scenarios.
"""

from __future__ import annotations

import importlib
import json
import math
import threading
import time
from pathlib import Path
from typing import Any

from hal.base_driver import BaseDriver
from hal.simulation.isaac_scene_bootstrap import bootstrap_isaac_scene

_PROFILES_DIR = Path(__file__).resolve().parent.parent / "profiles"


class G1SimulationDriver(BaseDriver):
    """Bridge ACTION.md calls to G1 navigation via internutopia.

    G1 is a humanoid robot with locomotion capabilities (move_to_point).
    This driver only supports navigation — no manipulation.
    """

    def __init__(self, gui: bool = False, **kwargs: Any) -> None:
        self._gui = gui
        self._api = None
        self._env = None
        self._env_lock = threading.RLock()
        self._last_obs: Any = None
        self._last_scene: dict[str, dict] = {}

        self._scene_asset_path = str(kwargs.get("scene_asset_path", "")).strip()
        self._robot_position = tuple(kwargs.get("robot_position", (0.0, 0.0, 0.78)))
        self._robot_cfg = None

        self._waypoints: dict[str, list[float]] = self._normalize_waypoints(kwargs.get("waypoints", {}))
        self._waypoint_aliases: dict[str, str] = self._normalize_aliases(kwargs.get("waypoint_aliases", {}))
        self._navigation_action_name = kwargs.get("navigation_action_name")
        self._navigation_max_steps = int(kwargs.get("navigation_max_steps", 1500))
        self._navigation_threshold = float(kwargs.get("navigation_threshold", 0.10))

        self._output_dir = Path(kwargs.get("output_dir", "/tmp/paos_g1_sim_logs"))

        self._pythonpath_entries = self._normalize_pythonpath(kwargs.get("pythonpath", []))
        self._ensure_pythonpath()

        self._idle_step_enabled = bool(kwargs.get("idle_step_enabled", True))
        self._idle_steps_per_cycle = int(kwargs.get("idle_steps_per_cycle", 1))
        self._idle_step_interval_s = float(kwargs.get("idle_step_interval_s", 1.0 / 30.0))
        self._last_idle_step_ts = 0.0

        self._api_kwargs = dict(kwargs.get("api_kwargs", {}))
        self._collision_patch = bool(kwargs.get("enable_static_scene_mesh_collision_patch", True))

        self._room_lighting = str(kwargs.get("room_lighting", "none")).strip().lower()
        self._camera_eye_offset = tuple(kwargs.get("camera_eye_offset", (-2.8, -2.2, 1.8)))
        self._camera_target_z_offset = float(kwargs.get("camera_target_z_offset", -0.4))
        self._camera_target_min_z = float(kwargs.get("camera_target_min_z", 0.2))
        self._room_bootstrap = dict(kwargs.get("room_bootstrap", {}))

    @staticmethod
    def _normalize_pythonpath(raw: Any) -> list[str]:
        if isinstance(raw, str):
            raw = [raw]
        if not isinstance(raw, list):
            return []
        out: list[str] = []
        for item in raw:
            p = str(item).strip()
            if not p:
                continue
            out.append(str(Path(p).expanduser().resolve()))
        return out

    def _ensure_pythonpath(self) -> None:
        import sys
        for entry in reversed(self._pythonpath_entries):
            if entry in sys.path:
                continue
            sys.path.insert(0, entry)

    @staticmethod
    def _normalize_waypoints(raw: Any) -> dict[str, list[float]]:
        out: dict[str, list[float]] = {}
        if not isinstance(raw, dict):
            return out
        for k, v in raw.items():
            key = str(k).strip()
            if not key:
                continue
            if isinstance(v, (list, tuple)) and len(v) >= 2:
                out[key] = [float(v[0]), float(v[1])]
        return out

    @staticmethod
    def _normalize_aliases(raw: Any) -> dict[str, str]:
        out: dict[str, str] = {}
        if not isinstance(raw, dict):
            return out
        for k, v in raw.items():
            a = str(k).strip().lower()
            t = str(v).strip()
            if a and t:
                out[a] = t
        return out

    def get_profile_path(self) -> Path:
        return _PROFILES_DIR / "g1_simulation.md"

    def load_scene(self, scene: dict[str, dict]) -> None:
        self._last_scene = dict(scene or {})

    def execute_action(self, action_type: str, params: dict) -> str:
        handlers = {
            "start": self._start_from_action,
            "enter_simulation": self._start_from_action,
            "close": self._close_api,
            "step": self._step_env,
            "api_call": self._api_call,
            "navigate_to_waypoint": self._navigate_to_waypoint,
            "navigate_to_named": self._navigate_to_named,
        }
        handler = handlers.get(action_type)
        if handler is None:
            return f"Unknown action: {action_type}"
        try:
            return handler(params or {})
        except Exception as exc:
            return f"Error: {type(exc).__name__}: {exc}"

    def get_scene(self) -> dict[str, Any]:
        scene = dict(self._last_scene)
        robot_xy = [float(self._robot_position[0]), float(self._robot_position[1])]
        navigable = sorted(set(self._waypoints.keys()) | set(self._waypoint_aliases.keys()))
        scene["manipulation_runtime"] = {
            "location": "sim",
            "state": "running" if self._api is not None else "idle",
            "robot_type": "g1",
            "robot_xy": robot_xy,
            "waypoint_keys": sorted(self._waypoints.keys()),
            "waypoint_aliases": dict(self._waypoint_aliases),
            "navigable_names": navigable,
            "gui_requested": bool(self._gui),
        }
        return scene

    def get_runtime_state(self) -> dict[str, Any]:
        if self._api is None or self._env is None:
            return {}
        try:
            obs = self._env.get_observations()
            if isinstance(obs, dict) and len(obs) > 0:
                first_obs = obs[0] if isinstance(obs, list) else obs
                if isinstance(first_obs, dict) and "position" in first_obs:
                    pos = first_obs["position"]
                    if hasattr(pos, "tolist"):
                        pos = pos.tolist()
                    return {
                        "position": list(pos) if isinstance(pos, (list, tuple)) else [float(pos)],
                    }
        except Exception:
            pass
        return {}

    def health_check(self) -> bool:
        if self._idle_step_enabled:
            self._idle_step_if_due()
        return self._api is not None

    def close(self) -> None:
        self._close_api({})

    def _resolve_nav_action_name(self) -> str:
        if self._navigation_action_name:
            return str(self._navigation_action_name)
        return "move_to_point"

    def _start_from_action(self, params: dict[str, Any]) -> str:
        if self._api is not None:
            return "G1 Simulation API already started."

        scene_asset_path = str(params.get("scene_asset_path", self._scene_asset_path)).strip()
        if not scene_asset_path:
            return "Error: missing scene_asset_path in driver config or action params."
        if not Path(scene_asset_path).exists():
            return f"Error: scene file not found: {scene_asset_path}"

        robot_position = params.get("robot_position", list(self._robot_position))
        if isinstance(robot_position, list) and len(robot_position) >= 3:
            self._robot_position = tuple(float(x) for x in robot_position[:3])

        api_kwargs = dict(self._api_kwargs)
        api_kwargs.update(params.get("api_kwargs", {}))

        self._api = self._build_api(
            scene_asset_path=scene_asset_path,
            robot_position=self._robot_position,
            api_kwargs=api_kwargs,
        )
        # internutopia.core.gym_env.Env has .step / .get_observations on itself (no nested _env).
        self._env = self._api
        try:
            self._last_obs, _ = self._api.reset()
        except Exception:
            self._last_obs = None

        rb = dict(self._room_bootstrap)
        rb.update(params.get("room_bootstrap") or {})
        boot_steps: list[str] = []
        if rb.get("enabled") is True and not params.get("skip_room_bootstrap"):
            lighting_mode = str(rb.get("lighting", self._room_lighting)).strip()
            apply_lighting = bool(rb.get("apply_lighting", True))
            focus_camera = bool(rb.get("focus_camera", rb.get("focus_view_on_robot", False)))
            boot_steps = bootstrap_isaac_scene(
                self._api,
                robot_xy=(float(self._robot_position[0]), float(self._robot_position[1])),
                robot_z=float(self._robot_position[2]),
                lighting_mode=lighting_mode,
                camera_eye_offset=self._camera_eye_offset,
                camera_target_z_offset=self._camera_target_z_offset,
                camera_target_min_z=self._camera_target_min_z,
                apply_lighting=apply_lighting,
                focus_camera=focus_camera,
            )

        if boot_steps:
            return f"G1 Simulation API started. bootstrap[{','.join(boot_steps)}]"
        return "G1 Simulation API started."

    def _build_api(self, *, scene_asset_path: str, robot_position: tuple[float, ...], api_kwargs: dict[str, Any]):
        import sys
        from pathlib import Path

        pythonpath = self._pythonpath_entries
        for entry in reversed(pythonpath):
            ep = str(Path(entry).expanduser().resolve())
            if ep not in sys.path:
                sys.path.insert(0, ep)

        pythonpath_from_kwargs = api_kwargs.pop("pythonpath", None)
        if pythonpath_from_kwargs:
            if isinstance(pythonpath_from_kwargs, str):
                pythonpath_from_kwargs = [pythonpath_from_kwargs]
            for entry in reversed(pythonpath_from_kwargs):
                ep = str(Path(entry).expanduser().resolve())
                if ep not in sys.path:
                    sys.path.insert(0, ep)

        bridge = importlib.import_module("internutopia.bridge")
        create_g1_robot_cfg = bridge.create_g1_robot_cfg

        core_config = importlib.import_module("internutopia.core.config")
        Config = core_config.Config
        SimConfig = core_config.SimConfig

        core_gym_env = importlib.import_module("internutopia.core.gym_env")
        Env = core_gym_env.Env

        internutopia_extension = importlib.import_module("internutopia_extension")
        import_extensions = internutopia_extension.import_extensions

        tasks = importlib.import_module("internutopia_extension.configs.tasks")
        SingleInferenceTaskCfg = tasks.SingleInferenceTaskCfg

        # Piper-style configs use force_gui; SimConfig only has headless.
        force_gui = api_kwargs.pop("force_gui", None)
        headless = api_kwargs.pop("headless", None)
        if headless is None:
            if force_gui is not None:
                headless = not bool(force_gui)
            else:
                headless = not self._gui

        robot_cfg = create_g1_robot_cfg(position=robot_position)
        robot_cfg.name = "g1"
        robot_cfg.prim_path = "/g1"

        config = Config(
            simulator=SimConfig(
                physics_dt=1 / 240,
                rendering_dt=1 / 240,
                use_fabric=False,
                headless=headless,
                webrtc=headless,
            ),
            metrics_save_path="none",
            task_configs=[
                SingleInferenceTaskCfg(
                    scene_asset_path=scene_asset_path,
                    robots=[robot_cfg],
                    enable_static_scene_mesh_collision_patch=self._collision_patch,
                )
            ],
        )
        import_extensions()
        return Env(config)

    def _close_api(self, _params: dict[str, Any]) -> str:
        if self._api is None:
            return "G1 Simulation API already closed."
        try:
            self._api.close()
        finally:
            self._api = None
            self._env = None
        return "G1 Simulation API closed."

    def _step_env(self, params: dict[str, Any]) -> str:
        if self._env is None:
            return "Error: API not started. Dispatch action_type='start' first."
        action = params.get("action", {})
        with self._env_lock:
            self._last_obs, _, _, _, _ = self._env.step(action=action)
        return "Environment stepped."

    def _api_call(self, params: dict[str, Any]) -> str:
        if self._api is None:
            return "Error: API not started. Dispatch action_type='start' first."
        method_name = str(params.get("method", "")).strip()
        if not method_name:
            return "Error: parameters.method is required for api_call."
        method = getattr(self._api, method_name, None)
        if method is None or not callable(method):
            return f"Error: method not found on API: {method_name}"
        args = params.get("args", [])
        kwargs = params.get("kwargs", {})
        if not isinstance(args, list):
            return "Error: parameters.args must be a list."
        if not isinstance(kwargs, dict):
            return "Error: parameters.kwargs must be a dict."
        result = method(*args, **kwargs)
        return f"api_call ok: {method_name} => {self._safe_json(result)}"

    def _resolve_waypoint_key(self, key: str) -> str:
        k = key.strip()
        for canon in self._waypoints:
            if canon.lower() == k.lower():
                return canon
        alias = self._waypoint_aliases.get(k.lower())
        if alias:
            at = alias.strip()
            for canon in self._waypoints:
                if canon.lower() == at.lower():
                    return canon
        return k

    def _navigate_to_named(self, params: dict[str, Any]) -> str:
        raw_key = str(params.get("waypoint_key") or params.get("target") or "").strip()
        if not raw_key:
            return "Error: waypoint_key or target is required."
        key = self._resolve_waypoint_key(raw_key)
        wp = self._waypoints.get(key)
        if not wp:
            known = sorted(self._waypoints.keys())
            als = sorted(self._waypoint_aliases.keys())
            return (
                f"Error: unknown waypoint_key={raw_key!r} (resolved={key!r}). "
                f"Known waypoints: {known}. Aliases: {als}"
            )
        return self._navigate_xy(
            [float(wp[0]), float(wp[1])],
            max_steps=int(params.get("max_steps", self._navigation_max_steps)),
            threshold=float(params.get("threshold", self._navigation_threshold)),
        )

    def _navigate_to_waypoint(self, params: dict[str, Any]) -> str:
        waypoint = params.get("waypoint_xy")
        if not (isinstance(waypoint, list) and len(waypoint) >= 2):
            return "Error: waypoint_xy must be [x, y]."
        return self._navigate_xy(
            [float(waypoint[0]), float(waypoint[1])],
            max_steps=int(params.get("max_steps", self._navigation_max_steps)),
            threshold=float(params.get("threshold", self._navigation_threshold)),
        )

    def _navigate_xy(
        self,
        xy: list[float],
        *,
        max_steps: int,
        threshold: float,
    ) -> str:
        if self._env is None:
            return "Error: API not started. Dispatch action_type='start' first."
        action_name = self._resolve_nav_action_name()
        goal_action = {action_name: [(float(xy[0]), float(xy[1]), 0.0)]}
        dist = 9999.0
        stable_finished = 0
        for _ in range(max_steps):
            with self._env_lock:
                self._last_obs, _, terminated, _, _ = self._env.step(action=goal_action)
            episode_terminated = terminated[0] if isinstance(terminated, (list, tuple)) else bool(terminated)
            if episode_terminated:
                return "navigate terminated early."
            obs_list = self._last_obs if isinstance(self._last_obs, list) else [self._last_obs]
            robot_obs = None
            for obs in obs_list:
                if isinstance(obs, dict) and "g1" in obs:
                    robot_obs = obs["g1"]
                    break
                if isinstance(obs, dict) and "position" in obs:
                    robot_obs = obs
                    break
            if robot_obs is None:
                continue
            pos = robot_obs.get("position")
            if pos is not None:
                if hasattr(pos, "tolist"):
                    pos = pos.tolist()
                if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                    dx = pos[0] - float(xy[0])
                    dy = pos[1] - float(xy[1])
                    dist = math.hypot(dx, dy)
                    if dist <= threshold:
                        return f"navigate ok: reached xy=({xy[0]:.4f},{xy[1]:.4f}), dist={dist:.4f}"
            ctrl = (robot_obs.get("controllers") or {}).get(action_name) or {}
            if bool(ctrl.get("finished")):
                stable_finished += 1
                if stable_finished >= 15:
                    return f"navigate ok: controller finished (dist={dist:.4f})"
            else:
                stable_finished = 0
        return f"navigate failed: dist={dist:.4f}"

    def _idle_step_if_due(self) -> None:
        if self._api is None or self._env is None:
            return
        now = time.monotonic()
        if now - self._last_idle_step_ts < self._idle_step_interval_s:
            return
        self._last_idle_step_ts = now
        steps = max(1, self._idle_steps_per_cycle)
        action_name = self._resolve_nav_action_name()
        obs_list = self._last_obs if isinstance(self._last_obs, list) else [self._last_obs]
        robot_obs = None
        for obs in obs_list:
            if isinstance(obs, dict) and "g1" in obs:
                robot_obs = obs["g1"]
                break
            if isinstance(obs, dict) and "position" in obs:
                robot_obs = obs
                break
        if robot_obs:
            pos = robot_obs.get("position")
            if pos is not None:
                if hasattr(pos, "tolist"):
                    pos = pos.tolist()
                hold_xy = (float(pos[0]), float(pos[1])) if isinstance(pos, (list, tuple)) and len(pos) >= 2 else (float(self._robot_position[0]), float(self._robot_position[1]))
            else:
                hold_xy = (float(self._robot_position[0]), float(self._robot_position[1]))
        else:
            hold_xy = (float(self._robot_position[0]), float(self._robot_position[1]))
        hold_action = {action_name: [(float(hold_xy[0]), float(hold_xy[1]), 0.0)]}
        for _ in range(steps):
            with self._env_lock:
                self._last_obs, _, _, _, _ = self._env.step(action=hold_action)

    @staticmethod
    def _safe_json(value: Any) -> str:
        try:
            return json.dumps(value, ensure_ascii=False, default=str)
        except TypeError:
            return str(value)
