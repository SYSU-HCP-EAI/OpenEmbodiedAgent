"""
Monolithic multi-robot InternUtopia simulation driver.

Design goals:
1) Single watchdog process.
2) Single InternUtopia Env (single scene runtime).
3) Spawn PiperGo2 + Franka + G1 in one scene at startup.
4) Do not dispatch to external HAL backend drivers.
"""

from __future__ import annotations

import importlib
import math
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from hal.base_driver import BaseDriver

_PROFILES_DIR = Path(__file__).resolve().parent.parent / "profiles"


@dataclass
class RobotSpec:
    robot_id: str
    robot_type: str
    position: tuple[float, float, float]
    waypoints: dict[str, list[float]]
    waypoint_aliases: dict[str, str]
    nav_action_name: str | None
    nav_max_steps: int
    nav_threshold: float
    grasp_targets: dict[str, dict[str, Any]]
    place_targets: dict[str, dict[str, Any]]
    arm_mass_scale: float
    objects: list[dict[str, Any]]
    api_kwargs: dict[str, Any]
    visible_objects: list[dict[str, Any]]
    pick_place: dict[str, Any]
    pick_place_defaults: dict[str, Any]
    output_dir: str
    grasp_dump: str
    place_dump: str
    room_bootstrap: dict[str, Any]
    room_lighting: str
    camera_eye_offset: tuple[float, float, float]
    camera_target_z_offset: float
    camera_target_min_z: float
    collision_patch: bool
    pause_steps: int
    gripper_settle_steps: int
    arm_waypoint_count: int


class MultiRobotSimulationInternutopiaDriver(BaseDriver):
    """Single-file cohesive driver for PiperGo2 + Franka + G1."""

    def __init__(self, gui: bool = False, **kwargs: Any) -> None:
        self._gui = bool(gui)
        self._strict_robot_id = bool(kwargs.get("strict_robot_id", True))
        self._scene_asset_path = str(kwargs.get("scene_asset_path", "")).strip()
        self._pythonpath_entries = self._normalize_pythonpath(kwargs.get("pythonpath", []))
        self._idle_step_enabled = bool(kwargs.get("idle_step_enabled", True))
        self._idle_steps_per_cycle = int(kwargs.get("idle_steps_per_cycle", 1))
        self._idle_step_interval_s = float(kwargs.get("idle_step_interval_s", 1.0 / 30.0))
        self._last_idle_step_ts = 0.0
        # Keep GUI responsive: rendering every physics step (interval=0) can starve UI events
        # for heavy multi-robot scenes. Match single-driver behavior by defaulting to 5 in GUI.
        self._rendering_interval = int(kwargs.get("rendering_interval", 5 if self._gui else 0))
        self._env_lock = threading.RLock()

        self._last_scene: dict[str, dict] = {}
        self._runtime_summary: dict[str, dict[str, Any]] = {}
        self._robots: dict[str, RobotSpec] = self._build_robot_specs(kwargs)

        self._env = None
        self._last_obs: Any = None
        self._started = False

    def get_profile_path(self) -> Path:
        return _PROFILES_DIR / "multi_robot_simulation_internutopia.md"

    def connect(self) -> bool:
        return True

    def disconnect(self) -> None:
        self.close()

    def is_connected(self) -> bool:
        return True

    def load_scene(self, scene: dict[str, dict]) -> None:
        self._last_scene = dict(scene or {})

    def execute_action(self, action_type: str, params: dict) -> str:
        payload = dict(params or {})
        robot_id = str(payload.get("robot_id") or "").strip()

        if action_type in ("start", "enter_simulation"):
            result = self._start_runtime(payload)
            self._update_runtime("scene", action_type, result)
            return result
        if action_type == "close":
            result = self._close_runtime()
            self._update_runtime("scene", action_type, result)
            return result
        if action_type == "step":
            result = self._step_env(payload)
            self._update_runtime("scene", action_type, result)
            return result
        if action_type == "api_call":
            result = self._api_call(payload)
            self._update_runtime("scene", action_type, result)
            return result

        if self._strict_robot_id and not robot_id:
            return "Error: missing robot_id for multi_robot_simulation_internutopia action."
        if robot_id not in self._robots:
            known = ", ".join(sorted(self._robots.keys()))
            return f"Error: unknown robot_id={robot_id!r}. known=[{known}]"

        if action_type in ("navigate_to_named", "navigate_to_waypoint"):
            result = self._navigate(robot_id=robot_id, action_type=action_type, params=payload)
        elif action_type in ("pick", "grasp", "release", "place"):
            result = self._franka_manipulate(robot_id=robot_id, action_type=action_type, params=payload)
        else:
            return f"Unknown action: {action_type}"

        self._update_runtime(robot_id, action_type, result)
        return result

    def get_scene(self) -> dict[str, dict]:
        scene = dict(self._last_scene)
        scene["multi_robot_runtime"] = {
            "location": "sim",
            "state": "running" if self._started else "idle",
            "scene_asset_path": self._scene_asset_path,
            "registered_robot_ids": sorted(self._robots.keys()),
            "robot_types": {rid: spec.robot_type for rid, spec in self._robots.items()},
            "single_env_mode": True,
            "single_process_mode": True,
            "robot_configs": {
                rid: {
                    "position": list(spec.position),
                    "waypoints": sorted(spec.waypoints.keys()),
                    "room_lighting": spec.room_lighting,
                    "collision_patch": spec.collision_patch,
                }
                for rid, spec in self._robots.items()
            },
        }
        return scene

    def get_runtime_state(self) -> dict[str, Any]:
        return {
            "robots": dict(self._runtime_summary),
            "scene_driver": {
                "name": "multi_robot_simulation_internutopia",
                "single_env_mode": True,
            },
        }

    def health_check(self) -> bool:
        if self._idle_step_enabled:
            self._idle_step_if_due()
        return True

    def close(self) -> None:
        self._close_runtime()

    def _start_runtime(self, params: dict[str, Any]) -> str:
        if self._started and self._env is not None:
            return "Multi-robot single-env runtime already started."

        scene_asset_path = str(params.get("scene_asset_path", self._scene_asset_path)).strip()
        if not scene_asset_path:
            return "Error: missing scene_asset_path in config or action params."
        if not Path(scene_asset_path).exists():
            return f"Error: scene file not found: {scene_asset_path}"
        self._scene_asset_path = scene_asset_path
        self._ensure_pythonpath()

        try:
            Env = importlib.import_module("internutopia.core.vec_env").Env
            core_cfg = importlib.import_module("internutopia.core.config")
            Config = core_cfg.Config
            SimConfig = core_cfg.SimConfig
            bridge = importlib.import_module("internutopia.bridge")
            tasks = importlib.import_module("internutopia_extension.configs.tasks")
            objects_mod = importlib.import_module("internutopia_extension.configs.objects")
            import_extensions = importlib.import_module("internutopia_extension").import_extensions
            SingleInferenceTaskCfg = tasks.SingleInferenceTaskCfg
        except Exception as exc:
            return f"Error: import internutopia modules failed: {type(exc).__name__}: {exc}"

        force_gui = bool(params.get("force_gui", self._gui))
        headless = bool(params.get("headless", not force_gui))

        robot_cfgs: list[Any] = []
        all_objects_spec: list[dict[str, Any]] = []

        for rid, spec in sorted(self._robots.items()):
            if spec.robot_type == "pipergo2":
                create = bridge.create_pipergo2_robot_cfg
                cfg = create(position=spec.position, arm_mass_scale=spec.arm_mass_scale)
                cfg.name = rid
                cfg.prim_path = f"/{rid}"
                robot_cfgs.append(cfg)
                if spec.objects:
                    all_objects_spec.extend(spec.objects)
            elif spec.robot_type == "franka":
                create = bridge.create_franka_robot_cfg
                cfg = create(position=spec.position)
                cfg.name = rid
                cfg.prim_path = f"/{rid}"
                robot_cfgs.append(cfg)
                if spec.objects:
                    all_objects_spec.extend(spec.objects)
            elif spec.robot_type == "g1":
                create = bridge.create_g1_robot_cfg
                cfg = create(position=spec.position)
                cfg.name = rid
                cfg.prim_path = f"/{rid}"
                robot_cfgs.append(cfg)
                if spec.objects:
                    all_objects_spec.extend(spec.objects)

        objects: list[Any] = []
        DynamicCubeCfg = objects_mod.DynamicCubeCfg
        VisualCubeCfg = objects_mod.VisualCubeCfg
        for item in all_objects_spec:
            if not isinstance(item, dict):
                continue
            kind = str(item.get("kind", "dynamic_cube")).strip().lower()
            cfg = {
                "name": item["name"],
                "prim_path": item["prim_path"],
                "position": tuple(float(x) for x in item["position"]),
                "scale": tuple(float(x) for x in item["scale"]),
                "color": item.get("color", (0.5, 0.5, 0.5)),
            }
            if kind == "visual_cube":
                cfg["color"] = list(cfg["color"])
                objects.append(VisualCubeCfg(**cfg))
            else:
                objects.append(DynamicCubeCfg(**cfg))

        try:
            use_collision_patch = any(spec.collision_patch for spec in self._robots.values())
            task_cfg = SingleInferenceTaskCfg(
                scene_asset_path=scene_asset_path,
                robots=robot_cfgs,
                objects=objects,
                enable_static_scene_mesh_collision_patch=use_collision_patch,
            )

            config = Config(
                simulator=SimConfig(
                    physics_dt=1 / 240,
                    rendering_dt=1 / 240,
                    use_fabric=False,
                    rendering_interval=max(0, self._rendering_interval),
                    headless=headless,
                    native=headless,
                    webrtc=headless,
                ),
                env_num=1,
                metrics_save_path="none",
                task_configs=[task_cfg],
            )
            import_extensions()
            self._env = Env(config)
            self._last_obs, _ = self._env.reset()
            self._started = True
            return (
                "Multi-robot single-env runtime started with robots: "
                f"{sorted(self._robots.keys())}"
            )
        except Exception as exc:
            self._env = None
            self._started = False
            return f"Error: runtime start failed: {type(exc).__name__}: {exc}"

    def _close_runtime(self) -> str:
        if self._env is None:
            self._started = False
            return "Multi-robot runtime already closed."
        try:
            self._env.close()
        except Exception:
            pass
        self._env = None
        self._started = False
        return "Multi-robot runtime closed."

    def _step_env(self, params: dict[str, Any]) -> str:
        if self._env is None:
            return "Error: runtime not started. Dispatch action_type='enter_simulation' first."
        action = params.get("action", {})
        with self._env_lock:
            self._last_obs, _, _, _, _ = self._env.step(action=action)
        return "Environment stepped."

    def _api_call(self, params: dict[str, Any]) -> str:
        if self._env is None:
            return "Error: runtime not started. Dispatch action_type='enter_simulation' first."
        method_name = str(params.get("method", "")).strip()
        if not method_name:
            return "Error: parameters.method is required for api_call."
        method = getattr(self._env, method_name, None)
        if method is None or not callable(method):
            return f"Error: method not found on Env: {method_name}"
        args = params.get("args", [])
        kwargs = params.get("kwargs", {})
        if not isinstance(args, list) or not isinstance(kwargs, dict):
            return "Error: parameters.args must be list and kwargs must be dict."
        result = method(*args, **kwargs)
        return f"api_call ok: {method_name} => {result}"

    def _navigate(self, *, robot_id: str, action_type: str, params: dict[str, Any]) -> str:
        spec = self._robots[robot_id]
        if spec.robot_type not in ("pipergo2", "g1"):
            return f"Error: {robot_id} ({spec.robot_type}) does not support navigation."
        if self._env is None:
            return "Error: runtime not started. Dispatch action_type='enter_simulation' first."

        if action_type == "navigate_to_named":
            key_raw = str(params.get("waypoint_key") or params.get("target") or "").strip()
            if not key_raw:
                return "Error: waypoint_key or target is required."
            key = self._resolve_waypoint_key(spec, key_raw)
            wp = spec.waypoints.get(key)
            if not wp:
                return f"Error: unknown waypoint_key={key_raw!r} for robot={robot_id!r}."
            xy = [float(wp[0]), float(wp[1])]
        else:
            wp = params.get("waypoint_xy")
            if not (isinstance(wp, list) and len(wp) >= 2):
                return "Error: waypoint_xy must be [x, y]."
            xy = [float(wp[0]), float(wp[1])]

        max_steps = int(params.get("max_steps", spec.nav_max_steps))
        threshold = float(params.get("threshold", spec.nav_threshold))
        action_name = str(spec.nav_action_name or "move_to_point")
        goal_action = {action_name: [(float(xy[0]), float(xy[1]), 0.0)]}
        stable_finished = 0
        dist = 9999.0

        for _ in range(max_steps):
            with self._env_lock:
                self._last_obs, _, terminated, _, _ = self._env.step(action=goal_action)
            term = terminated[0] if isinstance(terminated, (list, tuple)) else bool(terminated)
            if term:
                return "navigate terminated early."
            robot_obs = self._extract_robot_obs(self._last_obs, robot_id=robot_id, robot_type=spec.robot_type)
            if not isinstance(robot_obs, dict):
                continue
            pos = robot_obs.get("position")
            if hasattr(pos, "tolist"):
                pos = pos.tolist()
            if isinstance(pos, (list, tuple)) and len(pos) >= 2:
                dx = float(pos[0]) - float(xy[0])
                dy = float(pos[1]) - float(xy[1])
                dist = math.hypot(dx, dy)
                if dist <= threshold:
                    return f"navigate ok: {robot_id} reached ({xy[0]:.3f},{xy[1]:.3f}), dist={dist:.4f}"
            ctrl = (robot_obs.get("controllers") or {}).get(action_name) or {}
            if bool(ctrl.get("finished")):
                stable_finished += 1
                if stable_finished >= 15:
                    return f"navigate ok: {robot_id} controller finished (dist={dist:.4f})"
            else:
                stable_finished = 0
        return f"navigate timeout: {robot_id} dist={dist:.4f}"

    def _franka_manipulate(self, *, robot_id: str, action_type: str, params: dict[str, Any]) -> str:
        spec = self._robots[robot_id]
        if spec.robot_type != "franka":
            return f"Error: {robot_id} ({spec.robot_type}) does not support {action_type}."
        if self._env is None:
            return "Error: runtime not started. Dispatch action_type='enter_simulation' first."
        canonical = "grasp" if action_type in ("pick", "grasp") else "place"
        registry = spec.grasp_targets if canonical == "grasp" else spec.place_targets
        target = params.get("target")
        if isinstance(target, str):
            target_dict = registry.get(target)
        elif isinstance(target, dict):
            target_dict = target
        else:
            target_dict = None
        if not isinstance(target_dict, dict):
            return (
                f"Error: invalid {canonical} target for {robot_id}. "
                f"Known={sorted(registry.keys())}"
            )
        return (
            f"{canonical} request accepted for {robot_id} in single-env mode. "
            "Execution path is integrated; detailed Cartesian/gripper routine is pending full in-driver port."
        )

    def _idle_step_if_due(self) -> None:
        if self._env is None:
            return
        now = time.time()
        if now - self._last_idle_step_ts < self._idle_step_interval_s:
            return
        self._last_idle_step_ts = now
        try:
            with self._env_lock:
                for _ in range(max(1, self._idle_steps_per_cycle)):
                    self._last_obs, _, _, _, _ = self._env.step(action={})
        except Exception:
            pass

    @staticmethod
    def _extract_robot_obs(obs_data: Any, *, robot_id: str, robot_type: str) -> dict[str, Any] | None:
        if isinstance(obs_data, dict):
            if robot_id in obs_data and isinstance(obs_data[robot_id], dict):
                return obs_data[robot_id]
            if robot_type in obs_data and isinstance(obs_data[robot_type], dict):
                return obs_data[robot_type]
            if "position" in obs_data:
                return obs_data
        if isinstance(obs_data, list):
            for item in obs_data:
                if isinstance(item, dict):
                    if robot_id in item and isinstance(item[robot_id], dict):
                        return item[robot_id]
                    if robot_type in item and isinstance(item[robot_type], dict):
                        return item[robot_type]
                    if "position" in item:
                        return item
        return None

    def _update_runtime(self, robot_id: str, action_type: str, result: str) -> None:
        low = str(result).lower()
        status = "failed" if (low.startswith("error") or "failed" in low) else "completed"
        self._runtime_summary[robot_id] = {
            "last_action": action_type,
            "last_status": status,
            "last_result": str(result),
            "updated_at": datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z"),
        }

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
        for entry in reversed(self._pythonpath_entries):
            if entry not in sys.path:
                sys.path.insert(0, entry)

    @staticmethod
    def _normalize_waypoints(raw: Any) -> dict[str, list[float]]:
        out: dict[str, list[float]] = {}
        if not isinstance(raw, dict):
            return out
        for k, v in raw.items():
            key = str(k).strip()
            if key and isinstance(v, (list, tuple)) and len(v) >= 2:
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

    @staticmethod
    def _resolve_waypoint_key(spec: RobotSpec, key: str) -> str:
        k = key.strip()
        for canon in spec.waypoints:
            if canon.lower() == k.lower():
                return canon
        alias = spec.waypoint_aliases.get(k.lower())
        if alias:
            at = alias.strip()
            for canon in spec.waypoints:
                if canon.lower() == at.lower():
                    return canon
        return k

    def _build_robot_specs(self, raw: dict[str, Any]) -> dict[str, RobotSpec]:
        robots_raw = raw.get("robots", {})
        if isinstance(robots_raw, dict) and robots_raw:
            return self._from_nested_schema(robots_raw)
        return self._from_flat_schema(raw)

    def _from_flat_schema(self, raw: dict[str, Any]) -> dict[str, RobotSpec]:
        out: dict[str, RobotSpec] = {}
        defs = [
            ("pipergo2_001", "pipergo2", "pipergo2_"),
            ("franka_001", "franka", "franka_"),
            ("g1_001", "g1", "g1_"),
        ]
        for rid, rtype, prefix in defs:
            has_prefix = any(str(k).startswith(prefix) for k in raw.keys())
            if not has_prefix:
                continue
            pos_key = "robot_start" if rtype == "pipergo2" else "robot_position"
            pos = raw.get(f"{prefix}{pos_key}", [0.0, 0.0, 0.0])
            pos3 = (
                float(pos[0]) if isinstance(pos, (list, tuple)) and len(pos) > 0 else 0.0,
                float(pos[1]) if isinstance(pos, (list, tuple)) and len(pos) > 1 else 0.0,
                float(pos[2]) if isinstance(pos, (list, tuple)) and len(pos) > 2 else 0.0,
            )
            out[rid] = RobotSpec(
                robot_id=rid,
                robot_type=rtype,
                position=pos3,
                waypoints=self._normalize_waypoints(raw.get(f"{prefix}waypoints", {})),
                waypoint_aliases=self._normalize_aliases(raw.get(f"{prefix}waypoint_aliases", {})),
                nav_action_name=raw.get(f"{prefix}navigation_action_name"),
                nav_max_steps=int(raw.get(f"{prefix}navigation_max_steps", 1500)),
                nav_threshold=float(raw.get(f"{prefix}navigation_threshold", 0.10)),
                grasp_targets=dict(raw.get(f"{prefix}grasp_targets", {}) or {}),
                place_targets=dict(raw.get(f"{prefix}place_targets", {}) or {}),
                arm_mass_scale=float(raw.get(f"{prefix}arm_mass_scale", 1.0)),
                objects=list(raw.get(f"{prefix}objects", []) or []),
                api_kwargs=dict(raw.get(f"{prefix}api_kwargs", {}) or {}),
                visible_objects=list(raw.get(f"{prefix}visible_objects", []) or []),
                pick_place=dict(raw.get(f"{prefix}pick_place", {}) or {}),
                pick_place_defaults=dict(raw.get(f"{prefix}pick_place_defaults", {}) or {}),
                output_dir=str(raw.get(f"{prefix}output_dir", "")),
                grasp_dump=str(raw.get(f"{prefix}grasp_dump", "")),
                place_dump=str(raw.get(f"{prefix}place_dump", "")),
                room_bootstrap=dict(raw.get(f"{prefix}room_bootstrap", raw.get("room_bootstrap", {})) or {}),
                room_lighting=str(raw.get(f"{prefix}room_lighting", raw.get("room_lighting", "none"))),
                camera_eye_offset=tuple(raw.get(f"{prefix}camera_eye_offset", raw.get("camera_eye_offset", [2.5, 2.5, 2.0]))),
                camera_target_z_offset=float(
                    raw.get(f"{prefix}camera_target_z_offset", raw.get("camera_target_z_offset", 0.0))
                ),
                camera_target_min_z=float(
                    raw.get(f"{prefix}camera_target_min_z", raw.get("camera_target_min_z", 0.2))
                ),
                collision_patch=bool(raw.get(f"{prefix}enable_static_scene_mesh_collision_patch", True)),
                pause_steps=int(raw.get(f"{prefix}pause_steps", 45)),
                gripper_settle_steps=int(raw.get(f"{prefix}gripper_settle_steps", 30)),
                arm_waypoint_count=int(raw.get(f"{prefix}arm_waypoint_count", 8)),
            )
        return out

    def _from_nested_schema(self, robots_raw: dict[str, Any]) -> dict[str, RobotSpec]:
        out: dict[str, RobotSpec] = {}
        for rid, meta in robots_raw.items():
            robot_id = str(rid).strip()
            if not robot_id:
                continue
            m = dict(meta) if isinstance(meta, dict) else {}
            rtype = str(m.get("robot_type", "")).strip().lower()
            if not rtype:
                low = robot_id.lower()
                if "piper" in low:
                    rtype = "pipergo2"
                elif "franka" in low:
                    rtype = "franka"
                elif "g1" in low:
                    rtype = "g1"
                else:
                    rtype = "generic"
            p = m.get("position", [0.0, 0.0, 0.0])
            pos3 = (
                float(p[0]) if isinstance(p, (list, tuple)) and len(p) > 0 else 0.0,
                float(p[1]) if isinstance(p, (list, tuple)) and len(p) > 1 else 0.0,
                float(p[2]) if isinstance(p, (list, tuple)) and len(p) > 2 else 0.0,
            )
            out[robot_id] = RobotSpec(
                robot_id=robot_id,
                robot_type=rtype,
                position=pos3,
                waypoints=self._normalize_waypoints(m.get("waypoints", {})),
                waypoint_aliases=self._normalize_aliases(m.get("waypoint_aliases", {})),
                nav_action_name=m.get("navigation_action_name"),
                nav_max_steps=int(m.get("navigation_max_steps", 1500)),
                nav_threshold=float(m.get("navigation_threshold", 0.10)),
                grasp_targets=dict(m.get("grasp_targets", {}) or {}),
                place_targets=dict(m.get("place_targets", {}) or {}),
                arm_mass_scale=float(m.get("arm_mass_scale", 1.0)),
                objects=list(m.get("objects", []) or []),
                api_kwargs=dict(m.get("api_kwargs", {}) or {}),
                visible_objects=list(m.get("visible_objects", []) or []),
                pick_place=dict(m.get("pick_place", {}) or {}),
                pick_place_defaults=dict(m.get("pick_place_defaults", {}) or {}),
                output_dir=str(m.get("output_dir", "")),
                grasp_dump=str(m.get("grasp_dump", "")),
                place_dump=str(m.get("place_dump", "")),
                room_bootstrap=dict(m.get("room_bootstrap", {}) or {}),
                room_lighting=str(m.get("room_lighting", "none")),
                camera_eye_offset=tuple(m.get("camera_eye_offset", [2.5, 2.5, 2.0])),
                camera_target_z_offset=float(m.get("camera_target_z_offset", 0.0)),
                camera_target_min_z=float(m.get("camera_target_min_z", 0.2)),
                collision_patch=bool(m.get("enable_static_scene_mesh_collision_patch", True)),
                pause_steps=int(m.get("pause_steps", 45)),
                gripper_settle_steps=int(m.get("gripper_settle_steps", 30)),
                arm_waypoint_count=int(m.get("arm_waypoint_count", 8)),
            )
        return out


class MultiRobotSimulationDriver(MultiRobotSimulationInternutopiaDriver):
    """Backward-compatible alias."""
