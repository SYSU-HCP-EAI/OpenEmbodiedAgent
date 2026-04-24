"""
PiperGo2 manipulation HAL driver.

Bridges ACTION.md to PiperGo2ManipulationAPI (external sim stack).
Supports named waypoints, table-scene narration (from config), and pick/place presets.
"""

from __future__ import annotations

import importlib
import json
import math
import sys
import threading
import time
from pathlib import Path
from typing import Any

from hal.base_driver import BaseDriver
from hal.simulation.isaac_scene_bootstrap import apply_lighting_for_mode, focus_viewport_on_robot

_PROFILES_DIR = Path(__file__).resolve().parent.parent / "profiles"


class PiperGo2ManipulationDriver(BaseDriver):
    """Bridge ACTION.md calls to PiperGo2ManipulationAPI methods."""

    def __init__(self, gui: bool = False, **kwargs: Any) -> None:
        self._gui = gui
        self._api = None
        self._env = None
        self._env_lock = threading.RLock()
        self._last_obs: Any = None
        self._last_scene: dict[str, dict] = {}

        self._scene_asset_path = str(kwargs.get("scene_asset_path", "")).strip()
        self._robot_start = tuple(kwargs.get("robot_start", (0.0, 0.0, 0.55)))
        self._arm_mass_scale = float(kwargs.get("arm_mass_scale", 1.0))
        # Optional override for robot USD. Without this, InternUtopia's
        # create_pipergo2_robot_cfg() falls back to a hard-coded author path
        # (/home/zyserver/work/go2/urdf/pipergo2/pipergo2.usd) that doesn't
        # exist in containerized deployments, leaving /World/env_0/robots/
        # pipergo2 empty and crashing PhysX articulation init later with:
        #     AttributeError: 'NoneType' object has no attribute
        #     'is_homogeneous'
        self._robot_usd_path = str(kwargs.get("robot_usd_path", "")).strip()
        self._objects_spec = kwargs.get("objects", [])
        self._api_kwargs = dict(kwargs.get("api_kwargs", {}))
        self._api_kwargs["force_gui"] = bool(gui)
        self._api_kwargs.pop("headless", None)
        # Optional isaac_env block. Watchdog usually pops this out before
        # constructing the driver (because it must bootstrap BEFORE driver
        # import), but we still accept it here as a fallback for callers that
        # instantiate the driver directly (tests, notebooks, etc.).
        self._isaac_env_cfg = kwargs.get("isaac_env") or None

        self._waypoints: dict[str, list[float]] = self._normalize_waypoints(kwargs.get("waypoints", {}))
        self._waypoint_aliases: dict[str, str] = self._normalize_aliases(kwargs.get("waypoint_aliases", {}))
        self._navigation_action_name = kwargs.get("navigation_action_name")
        self._navigation_max_steps = int(kwargs.get("navigation_max_steps", 1200))
        self._navigation_threshold = float(kwargs.get("navigation_threshold", 0.10))

        self._visible_objects: list[dict[str, Any]] = list(kwargs.get("visible_objects", []))
        pp = kwargs.get("pick_place") or {}
        self._pick_target_raw = dict(pp.get("pick_target", {}))
        self._place_target_raw = dict(pp.get("place_target", {}))
        self._pick_place_output_dir = str(pp.get("output_dir", "/tmp/paos_pipergo2_logs"))
        self._pick_dump_name = str(pp.get("pick_dump", "room_pick.json"))
        self._place_dump_name = str(pp.get("place_dump", "room_place.json"))

        self._room_bootstrap = dict(kwargs.get("room_bootstrap", {}))
        self._pp_defaults = dict(kwargs.get("pick_place_defaults", {}))
        self._scene_narration_cn = ""
        self._last_pick_place_summary = ""
        self._pythonpath_entries = self._normalize_pythonpath(kwargs.get("pythonpath", []))
        self._ensure_pythonpath()
        self._idle_step_enabled = bool(kwargs.get("idle_step_enabled", True))
        self._idle_steps_per_cycle = int(kwargs.get("idle_steps_per_cycle", kwargs.get("idle_steps_per_poll", 1)))
        self._idle_step_interval_s = float(kwargs.get("idle_step_interval_s", 1.0 / 30.0))
        self._last_idle_step_ts = 0.0
        self._room_lighting = str(kwargs.get("room_lighting", "grey_studio")).strip().lower()
        self._camera_eye_offset = tuple(kwargs.get("camera_eye_offset", (-2.8, -2.2, 1.8)))
        self._camera_target_z_offset = float(kwargs.get("camera_target_z_offset", -0.4))
        self._camera_target_min_z = float(kwargs.get("camera_target_min_z", 0.2))
        self._eef_live_marker_enabled = bool(kwargs.get("eef_live_marker_enabled", False))

        # VLA closed-loop pick config (optional). Loaded lazily on first
        # run_vla_pick_and_return so watchdog startup stays cheap when VLA
        # is not requested. Contains ckpt_path / task_text / cameras /
        # thresholds — see examples/pipergo2_manipulation_driver.json.
        self._vla_cfg: dict[str, Any] = dict(kwargs.get("vla", {}) or {})
        self._vla_session: dict[str, Any] | None = None

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
        return _PROFILES_DIR / "pipergo2_manipulation.md"

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
            "describe_visible_scene": self._describe_visible_scene,
            "run_pick_place": self._run_pick_place,
            "run_vla_pick_and_return": self._run_vla_pick_and_return,
        }
        handler = handlers.get(action_type)
        if handler is None:
            return f"Unknown action: {action_type}"
        try:
            return handler(params or {})
        except Exception as exc:  # pragma: no cover - runtime dependency bridge
            import sys
            import traceback
            # Full traceback to stderr so the watchdog terminal shows the real
            # failure site (ACTION.md only gets str(exc), which loses context
            # for things like "NoneType has no attribute 'is_homogeneous'").
            print(
                f"[pipergo2] action {action_type!r} failed:",
                file=sys.stderr,
                flush=True,
            )
            traceback.print_exc()
            return f"Error: {type(exc).__name__}: {exc}"

    def get_scene(self) -> dict[str, dict]:
        scene = dict(self._last_scene)
        # Prefer the live pose from the most recent observation; fall back
        # to robot_start so the scene snapshot is still populated before
        # the first env.step. Without this the Critic sees a stale robot_xy
        # and incorrectly rejects actions that rely on "am I near X".
        live_xy: tuple[float, float] | None = None
        try:
            robot_obs = self._extract_robot_obs(self._last_obs)
            if robot_obs:
                live_xy = self._xy_from_robot_position(robot_obs.get("position"))
        except Exception:
            live_xy = None
        if live_xy is None:
            live_xy = (float(self._robot_start[0]), float(self._robot_start[1]))
        robot_xy = [float(live_xy[0]), float(live_xy[1])]
        navigable = sorted(set(self._waypoints.keys()) | set(self._waypoint_aliases.keys()))
        scene["manipulation_runtime"] = {
            "location": "sim",
            "state": "running" if self._api is not None else "idle",
            "robot_xy": robot_xy,
            "waypoint_keys": sorted(self._waypoints.keys()),
            "waypoint_aliases": dict(self._waypoint_aliases),
            "navigable_names": navigable,
            "gui_requested": bool(self._gui),
            "table_summary_cn": self._scene_narration_cn,
            "last_pick_place_cn": self._last_pick_place_summary,
            "last_obs": self._obs_brief(self._last_obs),
        }
        return scene

    def get_runtime_state(self) -> dict[str, Any]:
        nodes: list[dict[str, Any]] = []
        for vo in self._visible_objects:
            ok = vo.get("object_key")
            if not ok:
                continue
            nodes.append(
                {
                    "id": f"obj_{ok}",
                    "class": vo.get("shape_cn", "object"),
                    "object_key": ok,
                    "color_label_cn": vo.get("color_label_cn", ""),
                    "role": vo.get("role", ""),
                }
            )
        return {
            "scene_graph": {
                "nodes": nodes,
                "edges": [],
            }
        }

    def health_check(self) -> bool:
        """Keep simulator UI responsive even when ACTION.md is idle."""
        if self._idle_step_enabled:
            self._idle_step_if_due()
        return True

    def close(self) -> None:
        self._close_api({})

    def _resolve_nav_action_name(self) -> str:
        if self._navigation_action_name:
            return str(self._navigation_action_name)
        self._ensure_pythonpath()
        rob = importlib.import_module("internutopia_extension.configs.robots.pipergo2")
        return rob.move_to_point_cfg.name

    def _start_from_action(self, params: dict[str, Any]) -> str:
        if self._api is not None:
            return "Manipulation API already started."

        # Fallback bootstrap for callers that bypassed the watchdog (direct
        # driver instantiation). Idempotent: a no-op if the watchdog already
        # bootstrapped, or if gui is False and no isaac_env is configured.
        if self._gui and self._isaac_env_cfg is not None:
            try:
                from hal.simulation.isaac_bootstrap import bootstrap_isaac_env
                bootstrap_isaac_env(self._isaac_env_cfg, want_gui=True)
            except Exception as exc:  # don't block the action path on bootstrap
                print(f"[pipergo2] WARNING: isaac_env bootstrap skipped: {exc}")

        scene_asset_path = str(params.get("scene_asset_path", self._scene_asset_path)).strip()
        if not scene_asset_path:
            return "Error: missing scene_asset_path in driver config or action params."
        if not Path(scene_asset_path).exists():
            return f"Error: scene file not found: {scene_asset_path}"

        robot_start = params.get("robot_start", list(self._robot_start))
        arm_mass_scale = float(params.get("arm_mass_scale", self._arm_mass_scale))
        robot_usd_path = str(params.get("robot_usd_path", self._robot_usd_path)).strip()
        objects_spec = params.get("objects", self._objects_spec)
        api_kwargs = dict(self._api_kwargs)
        api_kwargs.update(params.get("api_kwargs", {}))

        self._api = self._build_api(
            scene_asset_path=scene_asset_path,
            robot_start=robot_start,
            arm_mass_scale=arm_mass_scale,
            robot_usd_path=robot_usd_path,
            objects_spec=objects_spec,
            api_kwargs=api_kwargs,
        )
        self._last_obs = self._api.start()
        marker_enabled = bool(params.get("eef_live_marker_enabled", self._eef_live_marker_enabled))
        if not marker_enabled:
            self._disable_api_eef_live_marker()
        self._env = getattr(self._api, "_env", None)
        if isinstance(robot_start, list) and len(robot_start) >= 3:
            self._robot_start = tuple(float(x) for x in robot_start[:3])

        rb = dict(self._room_bootstrap)
        rb.update(params.get("room_bootstrap") or {})
        boot_msg = ""
        if rb.get("enabled", True) and not params.get("skip_room_bootstrap"):
            boot_msg = self._room_bootstrap_sequence(rb)
        self._rebuild_scene_narration()

        vla_msg = self._maybe_preheat_vla_session()
        tail = " ".join(m for m in (boot_msg, vla_msg) if m)
        if tail:
            return f"Manipulation API started. {tail}"
        return "Manipulation API started."

    def _maybe_preheat_vla_session(self) -> str:
        """Attach VLA cameras + ArticulationView at start-up so the reference
        script's time-ordering is preserved: cameras go up while the robot is
        still parked at home with the arm in its default pose, and the
        60-step warmup is a pure static hold.

        SmolVLA itself is NOT loaded here — that happens lazily on the first
        ``run_vla_pick_and_return`` dispatch, so the watchdog can start cheap
        and the GPU stays free until the user explicitly asks for a deploy.

        Config: set ``vla.attach_on_start`` to ``false`` (or leave ``vla``
        empty) to skip the camera preheat too.
        """
        if not isinstance(self._vla_cfg, dict) or not self._vla_cfg:
            return ""
        if not self._vla_cfg.get("attach_on_start", True):
            return ""
        if not self._vla_cfg.get("cameras"):
            return ""
        hold_xy = (float(self._robot_start[0]), float(self._robot_start[1]))
        err = self._ensure_vla_cameras(self._vla_cfg, hold_xy)
        if err:
            print(f"[pipergo2] WARNING: VLA camera preheat skipped: {err}", flush=True)
            return f"vla_cam_preheat_skipped:{err}"
        return "vla_cam_preheat:ok"

    def _disable_api_eef_live_marker(self) -> None:
        if self._api is None:
            return
        try:
            # Disable per-step debug sphere update in InternUtopia API.
            setattr(self._api, "_update_eef_debug_marker", lambda _obs: None)
        except Exception:
            pass
        try:
            import omni

            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return
            marker_prim = stage.GetPrimAtPath("/World/debug_eef_live_marker")
            if marker_prim and marker_prim.IsValid():
                stage.RemovePrim("/World/debug_eef_live_marker")
        except Exception:
            pass

    def _room_bootstrap_sequence(self, rb: dict[str, Any]) -> str:
        if self._api is None or self._env is None:
            return ""
        steps: list[str] = []
        if rb.get("apply_room_lighting", True):
            try:
                mode = str(rb.get("lighting", self._room_lighting)).strip()
                steps.extend(apply_lighting_for_mode(self._api, mode))
            except Exception as exc:
                steps.append(f"lighting_skipped:{exc}")
        if rb.get("focus_view_on_robot", True):
            try:
                focus_viewport_on_robot(
                    (float(self._robot_start[0]), float(self._robot_start[1])),
                    float(self._robot_start[2]),
                    camera_eye_offset=self._camera_eye_offset,
                    camera_target_z_offset=self._camera_target_z_offset,
                    camera_target_min_z=self._camera_target_min_z,
                )
                steps.append("viewport_focus")
            except Exception as exc:
                steps.append(f"viewport_focus_skipped:{exc}")
        if rb.get("collision_patch", True):
            try:
                self._collision_patch_merom_scene()
                steps.append("collision_patch")
            except Exception as exc:
                steps.append(f"collision_patch_skipped:{exc}")
        masses = rb.get("set_masses") or {}
        if isinstance(masses, dict):
            for name, mass in masses.items():
                try:
                    obj = self._api._env.runner.get_obj(str(name))
                    obj.set_mass(float(mass))
                    steps.append(f"mass:{name}")
                except Exception:
                    pass
        sticky = rb.get("sticky_material")
        if isinstance(sticky, dict) and sticky.get("targets"):
            try:
                bound = self._apply_sticky_material(sticky)
                steps.append(f"sticky:{bound}")
            except Exception as exc:
                steps.append(f"sticky_skipped:{exc}")
        n_prev = int(rb.get("scene_preview_steps", 0))
        for _ in range(max(0, n_prev)):
            with self._env_lock:
                self._last_obs, _, _, _, _ = self._env.step({})
        if n_prev:
            steps.append(f"preview_steps:{n_prev}")
        n_stab = int(rb.get("stabilize_steps", 0))
        if n_stab > 0:
            xy = (float(self._robot_start[0]), float(self._robot_start[1]))
            self._stabilize_robot(xy, n_stab)
            steps.append(f"stabilize:{n_stab}")

        if rb.get("micro_navigate_on_start", False):
            off = rb.get("micro_navigate_offset_xy", [0.1, 0.0])
            if isinstance(off, (list, tuple)) and len(off) >= 2:
                gx = float(self._robot_start[0]) + float(off[0])
                gy = float(self._robot_start[1]) + float(off[1])
                micro_msg = self._navigate_xy(
                    [gx, gy],
                    max_steps=int(rb.get("micro_navigate_max_steps", 500)),
                    threshold=float(rb.get("micro_navigate_threshold", self._navigation_threshold)),
                )
                steps.append(f"micro_navigate:{micro_msg}")
        return "bootstrap[" + ",".join(steps) + "]"

    def _collision_patch_merom_scene(self) -> None:
        from pxr import PhysxSchema, Usd, UsdPhysics

        stage = self._api._env.runner._world.stage
        scene_root = stage.GetPrimAtPath("/World/env_0/scene")
        if not scene_root.IsValid():
            return
        for prim in Usd.PrimRange(scene_root):
            if prim.IsInstance():
                prim.SetInstanceable(False)
        for prim in Usd.PrimRange(scene_root):
            if prim.GetTypeName() != "Mesh":
                continue
            try:
                UsdPhysics.CollisionAPI.Apply(prim)
                physx = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                physx.CreateApproximationAttr().Set("convexHull")
            except Exception:
                pass

    def _apply_sticky_material(self, cfg: dict[str, Any]) -> int:
        # PhysX default μ=0.5 is too slippery for the Piper parallel-jaw
        # gripper (kp=80) to actually clamp the 5cm cube against gravity.
        # Bind a high-friction PhysicsMaterial to cube + finger meshes so
        # the VLA grasp holds instead of sliding back 3-4mm per tick.
        from pxr import Sdf, Usd, UsdPhysics, UsdShade

        stage = self._api._env.runner._world.stage
        path = str(cfg.get("path", "/World/PhysicsMaterials/StickyGrip"))
        static_f = float(cfg.get("static_friction", 2.0))
        dynamic_f = float(cfg.get("dynamic_friction", 2.0))
        restitution = float(cfg.get("restitution", 0.0))
        targets = cfg.get("targets") or []

        scope_path = path.rsplit("/", 1)[0]
        if scope_path and not stage.GetPrimAtPath(scope_path).IsValid():
            stage.DefinePrim(scope_path, "Scope")
        mat_prim = stage.GetPrimAtPath(path)
        if not mat_prim or not mat_prim.IsValid():
            UsdShade.Material.Define(stage, Sdf.Path(path))
            mat_prim = stage.GetPrimAtPath(path)
        physx = UsdPhysics.MaterialAPI.Apply(mat_prim)
        physx.CreateStaticFrictionAttr().Set(static_f)
        physx.CreateDynamicFrictionAttr().Set(dynamic_f)
        physx.CreateRestitutionAttr().Set(restitution)
        material = UsdShade.Material(mat_prim)

        mesh_types = {"Mesh", "Cube", "Sphere", "Cylinder", "Capsule"}
        total = 0
        for t in targets:
            root = stage.GetPrimAtPath(str(t))
            if not root or not root.IsValid():
                continue
            for prim in Usd.PrimRange(root):
                if prim.GetTypeName() not in mesh_types:
                    continue
                try:
                    UsdShade.MaterialBindingAPI(prim).Bind(
                        material,
                        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                        materialPurpose="physics",
                    )
                    total += 1
                except Exception:
                    pass
        return total

    def _stabilize_robot(self, target_xy: tuple[float, float], settle_steps: int) -> None:
        action_name = self._resolve_nav_action_name()
        idle_action = {action_name: [(float(target_xy[0]), float(target_xy[1]), 0.0)]}
        for _ in range(settle_steps):
            with self._env_lock:
                self._last_obs, _, terminated, _, _ = self._env.step(action=idle_action)
            ep = terminated[0] if isinstance(terminated, (list, tuple)) else bool(terminated)
            if ep:
                break

    def _close_api(self, _params: dict[str, Any]) -> str:
        if self._api is None:
            return "Manipulation API already closed."
        try:
            self._api.close()
        finally:
            self._api = None
            self._env = None
        return "Manipulation API closed."

    def _step_env(self, params: dict[str, Any]) -> str:
        if self._env is None:
            return "Error: API not started. Dispatch action_type='start' first."
        action = params.get("action", {})
        with self._env_lock:
            self._last_obs, _, _, _, _ = self._env.step(action=action)
        return "Environment stepped."

    def _idle_step_if_due(self) -> None:
        if self._api is None or self._env is None:
            return
        now = time.monotonic()
        if now - self._last_idle_step_ts < self._idle_step_interval_s:
            return
        self._last_idle_step_ts = now
        steps = max(1, self._idle_steps_per_cycle)
        # Empty env.step({}) advances physics without running AliengoMoveBySpeedController.forward,
        # so policy observation buffers (_old_policy_obs) desync from the articulation — later
        # move_to_point can output a frozen / collapsed gait. Hold current XY via move_to_point instead.
        action_name = self._resolve_nav_action_name()
        robot_obs = self._extract_robot_obs(self._last_obs)
        hold_xy = self._xy_from_robot_position(robot_obs.get("position") if robot_obs else None)
        if hold_xy is None:
            hold_xy = (float(self._robot_start[0]), float(self._robot_start[1]))
        hold_action = {action_name: [(float(hold_xy[0]), float(hold_xy[1]), 0.0)]}
        for _ in range(steps):
            with self._env_lock:
                self._last_obs, _, _, _, _ = self._env.step(action=hold_action)

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
            return "Error: parameters.kwargs must be an object."
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
            return "Error: waypoint_key or target is required (e.g. staging_table or desk)."
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
            action_name_override=(str(params["action_name"]) if params.get("action_name") else None),
        )

    def _navigate_xy(
        self,
        xy: list[float],
        *,
        max_steps: int,
        threshold: float,
        action_name_override: str | None = None,
        arm_target: list[float] | None = None,
        arm_action_name: str = "arm_joint_controller",
    ) -> str:
        if self._env is None:
            return "Error: API not started. Dispatch action_type='start' first."
        action_name = str(action_name_override or self._resolve_nav_action_name())
        goal_action: dict[str, Any] = {action_name: [(float(xy[0]), float(xy[1]), 0.0)]}
        if arm_target is not None:
            # Forward a fixed arm pose alongside nav so the gripper keeps
            # biting a grabbed object while the base drives home.
            goal_action[arm_action_name] = [list(arm_target)]
        dist = 9999.0
        stable_finished = 0
        for _ in range(max_steps):
            with self._env_lock:
                self._last_obs, _, terminated, _, _ = self._env.step(action=goal_action)
            episode_terminated = terminated[0] if isinstance(terminated, (list, tuple)) else bool(terminated)
            if episode_terminated:
                return "navigate terminated early."
            robot_obs = self._extract_robot_obs(self._last_obs)
            if not robot_obs:
                continue
            pos_xy = self._xy_from_robot_position(robot_obs.get("position"))
            if pos_xy is not None:
                dx = pos_xy[0] - float(xy[0])
                dy = pos_xy[1] - float(xy[1])
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

    def _rebuild_scene_narration(self) -> None:
        self._scene_narration_cn = self._build_table_narration_cn()

    def _build_table_narration_cn(self) -> str:
        if not self._visible_objects:
            return "No visible_objects configured; please define them in driver-config."
        cubes = [vo for vo in self._visible_objects if "cube" in str(vo.get("shape_cn", "")).lower()]
        others = [vo for vo in self._visible_objects if vo not in cubes]
        parts: list[str] = []
        if cubes:
            colors = [str(vo.get("color_label_cn", "")).strip() for vo in cubes if vo.get("color_label_cn")]
            uq = []
            for c in colors:
                if c and c not in uq:
                    uq.append(c)
            if uq:
                parts.append(f"multiple cubes with colors including {', '.join(uq)}")
            else:
                parts.append(f"{len(cubes)} cubes on the table")
        for vo in others:
            shape = vo.get("shape_cn", "object")
            col = vo.get("color_label_cn", "")
            parts.append(f"{col} {shape}".strip() if col else f"{shape}")
        return "I can see: " + "; ".join(parts) + "."

    def _describe_visible_scene(self, _params: dict[str, Any]) -> str:
        self._rebuild_scene_narration()
        return f"scene_description: {self._scene_narration_cn}"

    def _run_pick_place(self, params: dict[str, Any]) -> str:
        if self._api is None:
            return "Error: API not started. Dispatch action_type='start' first."
        defaults = dict(self._pp_defaults)
        execute_place = bool(params.get("execute_place", defaults.get("default_execute_place", True)))
        return_home = bool(params.get("return_home_after_place", defaults.get("return_home_after_place", False)))
        navigate_after_pick = bool(
            params.get("navigate_after_pick", defaults.get("navigate_to_place_pedestal_after_pick", False))
        )

        hint = self._normalize_color_hint(params.get("target_color_cn", "") or params.get("color_hint", ""))
        primary = str(defaults.get("primary_pick_object_key", "pick_cube"))
        keywords = [self._normalize_color_hint(x) for x in defaults.get("primary_pick_color_keywords", ["red"])]
        if hint:
            matched = any(k in hint for k in keywords)
            if not matched:
                return f"Error: pick hint {hint!r} does not match configured primary pick keywords {keywords}."

        pick_target = self._tupleize_grasp_dict(self._pick_target_raw)
        place_target = self._tupleize_grasp_dict(self._place_target_raw)
        if not pick_target or not pick_target.get("position"):
            return "Error: pick_place.pick_target missing in driver-config."

        out_dir = Path(params.get("output_dir", self._pick_place_output_dir))
        out_dir.mkdir(parents=True, exist_ok=True)
        pick_name = str(params.get("pick_dump", self._pick_dump_name))
        place_name = str(params.get("place_dump", self._place_dump_name))
        pick_path = out_dir / pick_name

        pick_result = self._api.pick(pick_target, dump_path=pick_path)
        pick_ok = self._result_ok(pick_result)
        lines = [f"pick success={pick_ok} steps={self._result_steps(pick_result)}"]
        if execute_place and pick_ok:
            if not place_target or not place_target.get("position"):
                return "Error: execute_place true but place_target missing in driver-config."
            place_path = out_dir / place_name
            place_result = self._api.release(place_target, dump_path=place_path)
            lines.append(
                f"place success={self._result_ok(place_result)} steps={self._result_steps(place_result)}"
            )
        elif navigate_after_pick and pick_ok:
            nav_xy_raw = params.get("navigate_after_pick_xy")
            if not (isinstance(nav_xy_raw, (list, tuple)) and len(nav_xy_raw) >= 2):
                nav_xy_raw = defaults.get("navigate_after_pick_xy")
            if isinstance(nav_xy_raw, (list, tuple)) and len(nav_xy_raw) >= 2:
                nav_xy = [float(nav_xy_raw[0]), float(nav_xy_raw[1])]
            elif place_target and place_target.get("position") and len(place_target["position"]) >= 2:
                nav_xy = [float(place_target["position"][0]), float(place_target["position"][1])]
            else:
                nav_xy = None
            if nav_xy:
                lines.append(
                    self._navigate_xy(
                        nav_xy,
                        max_steps=self._navigation_max_steps,
                        threshold=self._navigation_threshold,
                    )
                )
            else:
                lines.append("navigate_after_pick skipped (no valid target xy).")
        elif not execute_place:
            lines.append("place skipped (execute_place=false).")

        if return_home and self._waypoints.get("robot_home"):
            home = self._waypoints["robot_home"]
            lines.append(
                self._navigate_xy(
                    [float(home[0]), float(home[1])],
                    max_steps=self._navigation_max_steps,
                    threshold=self._navigation_threshold,
                )
            )

        self._last_pick_place_summary = "；".join(lines)
        return self._last_pick_place_summary

    @staticmethod
    def _normalize_color_hint(raw: Any) -> str:
        hint = str(raw or "").strip().lower()
        if not hint:
            return ""
        alias_map = {
            "红": "red",
            "紅": "red",
            "红色": "red",
            "紅色": "red",
            "蓝": "blue",
            "藍": "blue",
            "蓝色": "blue",
            "藍色": "blue",
            "绿": "green",
            "綠": "green",
            "绿色": "green",
            "綠色": "green",
            "黄": "yellow",
            "黃": "yellow",
            "黄色": "yellow",
            "黃色": "yellow",
        }
        return alias_map.get(hint, hint)

    @staticmethod
    def _result_ok(result: Any) -> bool:
        if result is None:
            return False
        if hasattr(result, "success"):
            return bool(getattr(result, "success"))
        if isinstance(result, dict):
            return bool(result.get("success", False))
        return True

    @staticmethod
    def _result_steps(result: Any) -> Any:
        if hasattr(result, "steps"):
            return getattr(result, "steps")
        if isinstance(result, dict):
            return result.get("steps", "?")
        return "?"

    @staticmethod
    def _tupleize_grasp_dict(raw: dict[str, Any]) -> dict[str, Any]:
        if not raw:
            return {}
        out: dict[str, Any] = {}
        for k, v in raw.items():
            if k in ("position", "pre_position", "post_position", "orientation") and isinstance(v, list):
                out[k] = tuple(float(x) for x in v)
            elif k == "metadata" and isinstance(v, dict):
                out[k] = dict(v)
            else:
                out[k] = v
        return out

    @staticmethod
    def _build_api(
        *,
        scene_asset_path: str,
        robot_start: Any,
        arm_mass_scale: float,
        objects_spec: Any,
        api_kwargs: dict[str, Any],
        robot_usd_path: str = "",
    ):
        pythonpath = api_kwargs.pop("pythonpath", None)
        if pythonpath:
            if isinstance(pythonpath, str):
                pythonpath = [pythonpath]
            for entry in reversed(pythonpath):
                ep = str(Path(str(entry)).expanduser().resolve())
                if ep not in sys.path:
                    sys.path.insert(0, ep)
        bridge = importlib.import_module("internutopia.bridge")
        objects_mod = importlib.import_module("internutopia_extension.configs.objects")
        PiperGo2ManipulationAPI = bridge.PiperGo2ManipulationAPI
        create_pipergo2_robot_cfg = bridge.create_pipergo2_robot_cfg
        DynamicCubeCfg = objects_mod.DynamicCubeCfg
        VisualCubeCfg = objects_mod.VisualCubeCfg

        rs = robot_start
        if isinstance(rs, list):
            rs = tuple(float(x) for x in rs)
        else:
            rs = tuple(rs)
        robot_cfg = create_pipergo2_robot_cfg(position=rs, arm_mass_scale=arm_mass_scale)
        if robot_usd_path:
            # Override the hard-coded default usd_path baked into
            # InternUtopia's create_pipergo2_robot_cfg(). Without this the
            # robot prim ends up empty and PhysX articulation init fails.
            robot_cfg.usd_path = robot_usd_path
        objects = []
        if isinstance(objects_spec, list):
            for item in objects_spec:
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

        headless = api_kwargs.pop("headless", None)
        if headless is None:
            headless = not bool(api_kwargs.pop("force_gui", False))
        return PiperGo2ManipulationAPI(
            scene_asset_path=scene_asset_path,
            robot_cfg=robot_cfg,
            objects=objects,
            headless=headless,
            **api_kwargs,
        )

    # ────────────────────────────────────────────────────────────────────
    # VLA closed-loop pick (ported from test_isaacsim_with_vla_pick.py)
    # ────────────────────────────────────────────────────────────────────

    def _resolve_approach_xy(
        self, pick_prim_path: str, offset: float
    ) -> tuple[float, float] | None:
        """Approach XY = cube XY shifted back along +X by ``offset``.

        Uses the cube position from driver-config ``objects`` first, then
        falls back to ``pick_place.pick_target.position``.
        """
        for obj in (self._objects_spec or []):
            if not isinstance(obj, dict):
                continue
            if obj.get("prim_path") == pick_prim_path:
                pos = obj.get("position")
                if pos and len(pos) >= 2:
                    return (float(pos[0]) - float(offset), float(pos[1]))
        pt = self._pick_target_raw or {}
        pos = pt.get("position")
        if pos and len(pos) >= 2:
            return (float(pos[0]) - float(offset), float(pos[1]))
        return None

    def _ensure_vla_cameras(
        self,
        cfg: dict[str, Any],
        hold_xy: tuple[float, float],
    ) -> str | None:
        """Attach the 3 training cameras + ArticulationView + run the 60-
        step warmup. Cheap enough to call at ``enter_simulation`` so the
        cameras go up while the robot is still at home with the arm in its
        default pose — same time-ordering as ``test_isaacsim_with_vla_pick``.

        Does NOT load SmolVLA (that happens lazily in
        ``_ensure_vla_controller`` the first time ``run_vla_pick_and_return``
        is dispatched). Idempotent: a no-op once ``cameras`` is populated.
        """
        if self._vla_session is not None and self._vla_session.get("cameras"):
            return None
        if self._api is None or self._env is None:
            return "Error: API not started. Dispatch action_type='enter_simulation' first."

        try:
            from hal.simulation import vla_pick as _vla
        except Exception as exc:
            return f"Error: could not import hal.simulation.vla_pick: {exc}"
        import numpy as _np

        robot_prim_path = str(cfg.get("robot_prim_path", "/World/env_0/robots/pipergo2"))
        mounts = dict(cfg.get("cameras") or {})
        if not mounts:
            return "Error: vla.cameras not configured (need camera1/2/3)"

        cam_resolution = tuple(cfg.get("cam_resolution", [640, 400]))
        warmup_steps = int(cfg.get("cam_warmup_steps", 60))
        sim_hz = int(cfg.get("sim_hz", 240))
        control_hz = int(cfg.get("control_hz", 10))

        # ArticulationView (needed for per-tick joint reads).
        robot_view, joint_indices = _vla.build_articulation_view(robot_prim_path)
        if robot_view is None:
            return "Error: could not initialize ArticulationView for VLA."

        # USD stage (for cube Z lookup + camera parent validation).
        stage = None
        try:
            stage = self._env.runner._world.stage  # type: ignore[attr-defined]
        except Exception:
            stage = None
        if stage is None:
            try:
                import omni  # type: ignore
                stage = omni.usd.get_context().get_stage()
            except Exception:
                return "Error: could not obtain USD stage for VLA camera setup."

        action_name = self._resolve_nav_action_name()
        warmup_hold_action = {
            action_name: [(float(hold_xy[0]), float(hold_xy[1]), 0.0)]
        }

        cameras, flip_set = _vla.attach_cameras(
            stage=stage,
            env=self._env,
            env_lock=self._env_lock,
            hold_action=warmup_hold_action,
            mounts=mounts,
            resolution=cam_resolution,
            warmup_steps=warmup_steps,
        )
        if not cameras:
            return "Error: no VLA cameras could be attached (see [vla-pick] log)."

        # Session-owned hold_action ref so run-time code can swap it to the
        # approach XY without rebuilding the controller's read_images closure.
        live_hold = {"action": dict(warmup_hold_action)}

        # Stash numpy here so the closure built in _ensure_vla_controller
        # doesn't need to re-import np later.
        self._vla_session = {
            "controller": None,
            "robot_view": robot_view,
            "joint_indices": joint_indices,
            "cameras": cameras,
            "flip_set": flip_set,
            "stage": stage,
            "sim_steps_per_action": max(1, sim_hz // max(1, control_hz)),
            "arm_action_name": str(cfg.get("arm_action_name", "arm_joint_controller")),
            "live_hold": live_hold,
            "_np": _np,
        }
        return None

    def _ensure_vla_controller(self, cfg: dict[str, Any]) -> str | None:
        """Lazy-load SmolVLA. Call this only when a VLA action is actually
        being dispatched so startup stays cheap and GPU stays free for other
        drivers until the user asks for a deploy.

        Requires that ``_ensure_vla_cameras`` has already populated session
        with cameras + robot_view + live_hold.
        """
        session = self._vla_session
        if session is None or not session.get("cameras"):
            return (
                "Error: VLA cameras not initialized; call _ensure_vla_cameras "
                "(or enter_simulation with vla.attach_on_start=true) first."
            )
        if session.get("controller") is not None:
            return None

        try:
            from hal.simulation import vla_pick as _vla
        except Exception as exc:
            return f"Error: could not import hal.simulation.vla_pick: {exc}"

        ckpt_path = str(cfg.get("ckpt_path", "")).strip()
        if not ckpt_path:
            return "Error: vla.ckpt_path not configured"
        if not Path(ckpt_path).exists():
            return f"Error: vla ckpt path not found: {ckpt_path}"

        task_text = str(cfg.get("task_text", "pick up the red cube"))
        n_action_steps = int(cfg.get("n_action_steps", 2))
        max_delta_arm = float(cfg.get("max_per_tick_delta_arm", 0.45))
        max_delta_gripper = float(cfg.get("max_per_tick_delta_gripper", 0.06))
        training_max = float(cfg.get("vla_gripper_training_max", 0.28))
        piper_max = float(cfg.get("piper_gripper_width_max", 0.07))
        gripper_scale = float(cfg.get("gripper_scale", piper_max / training_max))
        gripper_bias = float(cfg.get("gripper_bias", 0.0))
        state_gripper_scale = float(cfg.get("state_gripper_scale", training_max / piper_max))
        joint_limits = cfg.get("joint_limits") or _vla.DEFAULT_PIPER_JOINT_LIMITS

        robot_view = session["robot_view"]
        joint_indices = session["joint_indices"]
        cameras = session["cameras"]
        flip_set = session["flip_set"]
        live_hold = session["live_hold"]
        _np = session["_np"]

        def _read_state7():
            return _vla.read_piper_state7(robot_view, joint_indices)

        def _read_images():
            out: dict[str, Any] = {}
            for name, cam in cameras.items():
                rgb = _vla.grab_rgb(
                    cam, self._env, self._env_lock, live_hold["action"]
                )
                if rgb is None or not rgb.any():
                    return None
                if name in flip_set:
                    rgb = _np.ascontiguousarray(_np.fliplr(rgb))
                out[name] = rgb
            return out

        controller = _vla.VLAController(
            ckpt_path=ckpt_path,
            task_text=task_text,
            gripper_scale=gripper_scale,
            gripper_bias=gripper_bias,
            state_gripper_scale=state_gripper_scale,
            joint_limits=joint_limits,
            max_delta_arm=max_delta_arm,
            max_delta_gripper=max_delta_gripper,
            read_state7=_read_state7,
            read_images=_read_images,
        )
        controller.set_n_action_steps(n_action_steps)
        session["controller"] = controller
        return None

    def _run_vla_pick_and_return(self, params: dict[str, Any]) -> str:
        """Approach → closed-loop VLA pick (stops after gripper squeeze).

        The base parks at the training-camera approach pose, runs the
        SmolVLA closed-loop pick, force-closes the gripper, and stops
        there. The robot does NOT drive back home — the arm keeps the
        cube held in the post-pick pose for inspection / downstream
        skills.

        Preconditions: caller has already issued ``enter_simulation``.
        Approach offset from the cube is handled here so the base ends
        up in the exact training-camera pose regardless of where it
        currently is.
        """
        if self._api is None or self._env is None:
            return "Error: API not started. Dispatch action_type='enter_simulation' first."

        cfg = dict(self._vla_cfg)
        for k, v in (params or {}).items():
            if k == "action_type":
                continue
            cfg[k] = v

        pick_target_prim_path = str(cfg.get("pick_target_prim_path", "/World/pick_cube"))
        pick_nav_offset = float(cfg.get("pick_nav_offset", 0.41))
        approach_xy = self._resolve_approach_xy(pick_target_prim_path, pick_nav_offset)
        if approach_xy is None:
            return (
                f"Error: could not resolve pick_target position for "
                f"prim_path={pick_target_prim_path!r}; "
                f"add the cube to driver-config 'objects' or set pick_place.pick_target.position."
            )

        # Cameras are normally preheated at enter_simulation (robot still at
        # home, arm in default pose). If the user started the sim with
        # vla.attach_on_start=false, fall back to attaching them here with
        # the base at its CURRENT XY so the 60-step warmup hold is a static
        # hold instead of pulling the base somewhere else mid-warmup.
        live_xy: tuple[float, float] | None = None
        try:
            robot_obs = self._extract_robot_obs(self._last_obs)
            if robot_obs:
                live_xy = self._xy_from_robot_position(robot_obs.get("position"))
        except Exception:
            live_xy = None
        warmup_xy = live_xy or (
            float(self._robot_start[0]),
            float(self._robot_start[1]),
        )
        err = self._ensure_vla_cameras(cfg, warmup_xy)
        if err:
            return err
        # SmolVLA load is gated on the deploy command (this handler), not on
        # simulation start — keeps watchdog boot cheap and the GPU free.
        err = self._ensure_vla_controller(cfg)
        if err:
            return err
        session = self._vla_session
        assert session is not None

        from hal.simulation import vla_pick as _vla

        # Step A.2: approach the cube (same offset the rule-based path used).
        print(
            f"[pipergo2] vla approach_xy={approach_xy} "
            f"(offset={pick_nav_offset}m from {pick_target_prim_path})",
            flush=True,
        )
        approach_msg = self._navigate_xy(
            [approach_xy[0], approach_xy[1]],
            max_steps=int(cfg.get("approach_max_steps", self._navigation_max_steps)),
            threshold=float(cfg.get("approach_threshold", self._navigation_threshold)),
        )
        if approach_msg.startswith("navigate failed") or approach_msg.startswith("Error"):
            return f"Error: approach nav before VLA pick failed: {approach_msg}"

        # Step B: closed-loop VLA pick.
        max_ticks = int(cfg.get("max_ticks", 30))
        lift_threshold = float(cfg.get("cube_lift_threshold", 0.07))
        close_ramp = int(cfg.get("close_gripper_ramp_ticks", 8))
        close_hold = int(cfg.get("close_gripper_hold_sim_steps", 60))
        max_delta_gripper = float(cfg.get("max_per_tick_delta_gripper", 0.06))

        action_name = self._resolve_nav_action_name()
        arm_action_name = str(session["arm_action_name"])
        hold_action_pick = {action_name: [(float(approach_xy[0]), float(approach_xy[1]), 0.0)]}
        # Swap the session's live hold_action so grab_rgb() retries tick the
        # sim with approach_xy (base already here) instead of whatever XY we
        # used at camera attach time (e.g. robot home during preheat).
        live_hold = session.get("live_hold")
        if isinstance(live_hold, dict):
            live_hold["action"] = dict(hold_action_pick)

        def _read_arm8():
            return _vla.read_piper_arm8(session["robot_view"], session["joint_indices"])

        def _read_cube_z():
            return _vla.read_cube_world_z(self._env, session["stage"], pick_target_prim_path)

        dump_root = cfg.get("dump_root", "/tmp/paos_pipergo2_test_logs/paos")
        dump_every = int(cfg.get("dump_every", 1))
        dump_root_str: str | None = None
        if dump_root:
            import os as _os
            import time as _time

            run_tag = _time.strftime("%Y%m%d_%H%M%S")
            dump_root_str = _os.path.join(str(dump_root), f"pick_{run_tag}")

        print("[pipergo2] vla: starting closed-loop pick", flush=True)
        result = _vla.execute_pick(
            controller=session["controller"],
            env=self._env,
            env_lock=self._env_lock,
            nav_action_name=action_name,
            arm_action_name=arm_action_name,
            hold_action=hold_action_pick,
            read_arm8=_read_arm8,
            read_cube_z=_read_cube_z,
            hold_xy=approach_xy,
            max_ticks=max_ticks,
            lift_threshold=lift_threshold,
            sim_steps_per_action=int(session["sim_steps_per_action"]),
            close_gripper_ramp_ticks=close_ramp,
            close_gripper_hold_sim_steps=close_hold,
            max_per_tick_delta_gripper=max_delta_gripper,
            dump_root=dump_root_str,
            dump_every=dump_every,
        )

        init_z = result.get("initial_cube_z")
        fin_z = result.get("final_cube_z")
        dz_str = "n/a"
        if init_z is not None and fin_z is not None:
            dz_str = f"{float(fin_z) - float(init_z):+.3f}m"
        prefix = "vla pick SUCCESS" if result.get("success") else "Error: vla pick FAILED"
        return (
            f"{prefix} ticks={result.get('ticks_used')} Δz={dz_str} "
            f"terminate={result.get('terminate')}"
        )

    @staticmethod
    def _safe_obs(value: Any) -> Any:
        try:
            json.dumps(value, ensure_ascii=False)
            return value
        except TypeError:
            return str(value)

    @staticmethod
    def _safe_json(value: Any) -> str:
        try:
            return json.dumps(value, ensure_ascii=False, default=str)
        except TypeError:
            return str(value)

    @staticmethod
    def _obs_brief(obs_data: Any) -> dict[str, Any] | None:
        """Return a tiny snapshot to avoid bloating ENVIRONMENT.md."""
        robot = PiperGo2ManipulationDriver._extract_robot_obs(obs_data)
        if not isinstance(robot, dict):
            return None
        pos = robot.get("position")
        pos_list: list[float] | None = None
        if pos is not None:
            try:
                if hasattr(pos, "tolist"):
                    pos = pos.tolist()
                if isinstance(pos, (list, tuple)) and len(pos) >= 3:
                    pos_list = [float(pos[0]), float(pos[1]), float(pos[2])]
            except (TypeError, ValueError):
                pos_list = None
        if pos_list is not None:
            return {
                "position": pos_list,
                "render": bool(robot.get("render", False)),
            }
        return {
            "render": bool(robot.get("render", False)),
        }

    @staticmethod
    def _xy_from_robot_position(position: Any) -> tuple[float, float] | None:
        if position is None:
            return None
        try:
            if hasattr(position, "tolist"):
                position = position.tolist()
            if isinstance(position, (list, tuple)) and len(position) >= 2:
                return (float(position[0]), float(position[1]))
        except (TypeError, ValueError):
            return None
        return None

    @staticmethod
    def _extract_robot_obs(obs_data: Any) -> dict[str, Any] | None:
        if isinstance(obs_data, dict) and "position" in obs_data:
            return obs_data
        if isinstance(obs_data, dict) and "pipergo2" in obs_data:
            return obs_data["pipergo2"]
        if isinstance(obs_data, dict) and "pipergo2_0" in obs_data:
            return obs_data["pipergo2_0"]
        if isinstance(obs_data, (list, tuple)) and len(obs_data) > 0:
            first = obs_data[0]
            if isinstance(first, dict) and "pipergo2" in first:
                return first["pipergo2"]
            if isinstance(first, dict) and "pipergo2_0" in first:
                return first["pipergo2_0"]
            if isinstance(first, dict):
                return first
        return None
