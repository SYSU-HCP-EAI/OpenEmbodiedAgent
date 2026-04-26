"""
FrankaSimulationDriver — PhyAgentOS HAL driver for Franka arm simulation.

Bridges ACTION.md to FrankaManipulationAPI (internutopia sim stack).
Supports pick/place presets with Cartesian motion and gripper control.
"""

from __future__ import annotations

import importlib
import json
import threading
import time
from pathlib import Path
from typing import Any

from hal.base_driver import BaseDriver
from hal.simulation.isaac_scene_bootstrap import bootstrap_isaac_scene

_PROFILES_DIR = Path(__file__).resolve().parent.parent / "profiles"

_FRANKA_API_WITH_OBJECTS: type | None = None


def _build_task_objects_from_spec(objects_spec: Any) -> list[Any]:
    """Turn driver-config ``objects`` list into InternUtopia DynamicCubeCfg / VisualCubeCfg."""
    if not isinstance(objects_spec, list):
        return []
    objects_mod = importlib.import_module("internutopia_extension.configs.objects")
    DynamicCubeCfg = objects_mod.DynamicCubeCfg
    VisualCubeCfg = objects_mod.VisualCubeCfg
    out: list[Any] = []
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
            out.append(VisualCubeCfg(**cfg))
        else:
            out.append(DynamicCubeCfg(**cfg))
    return out


def _franka_manipulation_api_with_objects_class() -> type:
    """InternUtopia ``FrankaManipulationAPI`` omits ``objects`` on ``ManipulationTaskCfg``; extend in-process."""
    global _FRANKA_API_WITH_OBJECTS
    if _FRANKA_API_WITH_OBJECTS is not None:
        return _FRANKA_API_WITH_OBJECTS

    bridge = importlib.import_module("internutopia.bridge")
    base = bridge.FrankaManipulationAPI

    class FrankaManipulationAPIWithObjects(base):
        def __init__(
            self,
            scene_asset_path: str,
            robot_cfg: Any | None = None,
            *,
            objects: list[Any] | None = None,
            headless: bool | None = None,
            max_steps_per_phase: int = 600,
            gripper_settle_steps: int = 30,
            pause_steps: int = 45,
            arm_waypoint_count: int = 4,
        ) -> None:
            self._paos_manipulation_objects = list(objects or [])
            super().__init__(
                scene_asset_path,
                robot_cfg=robot_cfg,
                headless=headless,
                max_steps_per_phase=max_steps_per_phase,
                gripper_settle_steps=gripper_settle_steps,
                pause_steps=pause_steps,
                arm_waypoint_count=arm_waypoint_count,
            )

        def start(self) -> dict[str, Any]:
            from internutopia.bridge.atomic_actions import _to_builtin

            if self._env is not None:
                return _to_builtin(self._env.get_observations())

            from internutopia.core.config import Config, SimConfig
            from internutopia.core.gym_env import Env
            from internutopia_extension import import_extensions
            from internutopia_extension.configs.tasks import SingleInferenceTaskCfg

            # SingleInferenceTaskCfg matches PiperGo2ManipulationAPI (objects spawn reliably);
            # Merom baked ground needs mesh collision patch so dynamics rest on the floor.
            config = Config(
                simulator=SimConfig(
                    physics_dt=1 / 240,
                    rendering_dt=1 / 240,
                    use_fabric=False,
                    headless=self.headless,
                    webrtc=self.headless,
                ),
                metrics_save_path="none",
                task_configs=[
                    SingleInferenceTaskCfg(
                        scene_asset_path=self.scene_asset_path,
                        robots=[self.robot_cfg],
                        objects=self._paos_manipulation_objects,
                        enable_static_scene_mesh_collision_patch=True,
                    ),
                ],
            )
            import_extensions()
            self._env = Env(config)
            obs, _ = self._env.reset()
            return _to_builtin(obs)

    _FRANKA_API_WITH_OBJECTS = FrankaManipulationAPIWithObjects
    return FrankaManipulationAPIWithObjects


class FrankaSimulationDriver(BaseDriver):
    """Bridge ACTION.md calls to FrankaManipulationAPI methods.

    This driver wraps the FrankaManipulationAPI from internutopia to provide
    Franka arm manipulation capabilities in Isaac Sim simulation.

    Unlike PiperGo2 (mobile robot), Franka is a fixed-base 7-DOF arm, so
    navigation actions are not applicable.
    """

    def __init__(self, gui: bool = False, *, robot_id: str | None = None, **kwargs: Any) -> None:
        self.robot_id = str(robot_id or kwargs.get("robot_id") or "franka_001").strip() or "franka_001"
        self._gui = gui
        self._api = None
        self._env = None
        self._env_lock = threading.RLock()
        self._last_obs: Any = None
        self._last_scene: dict[str, dict] = {}

        self._scene_asset_path = str(kwargs.get("scene_asset_path", "")).strip()
        self._robot_position = tuple(kwargs.get("robot_position", (0.0, 0.0, 0.0)))
        self._robot_cfg = None
        self._robot_usd_path = str(kwargs.get("robot_usd_path", "")).strip()

        self._grasp_targets: dict[str, dict] = kwargs.get("grasp_targets", {})
        self._place_targets: dict[str, dict] = kwargs.get("place_targets", {})

        self._output_dir = Path(kwargs.get("output_dir", "/tmp/paos_franka_sim_logs"))
        self._grasp_dump_name = str(kwargs.get("grasp_dump", "grasp.json"))
        self._place_dump_name = str(kwargs.get("place_dump", "place.json"))

        self._pause_steps = int(kwargs.get("pause_steps", 45))
        self._gripper_settle_steps = int(kwargs.get("gripper_settle_steps", 30))
        self._arm_waypoint_count = int(kwargs.get("arm_waypoint_count", 8))

        self._pythonpath_entries = self._normalize_pythonpath(kwargs.get("pythonpath", []))
        self._ensure_pythonpath()

        self._scene_narration_cn = ""
        self._last_action_summary = ""
        self._last_grasp_target: dict[str, Any] | None = None

        self._idle_step_enabled = bool(kwargs.get("idle_step_enabled", True))
        self._idle_steps_per_cycle = int(kwargs.get("idle_steps_per_cycle", 1))
        self._idle_step_interval_s = float(kwargs.get("idle_step_interval_s", 1.0 / 30.0))
        self._last_idle_step_ts = 0.0

        self._objects_spec = kwargs.get("objects", [])
        self._api_kwargs = dict(kwargs.get("api_kwargs", {}))

        # Post-start scene tweaks are opt-in via room_bootstrap.enabled (see examples JSON).
        self._room_lighting = str(kwargs.get("room_lighting", "none")).strip().lower()
        self._camera_eye_offset = tuple(kwargs.get("camera_eye_offset", (-2.8, -2.2, 1.8)))
        self._camera_target_z_offset = float(kwargs.get("camera_target_z_offset", -0.4))
        self._camera_target_min_z = float(kwargs.get("camera_target_min_z", 0.2))
        self._room_bootstrap = dict(kwargs.get("room_bootstrap", {}))
        self._robot_cameras_cfg: dict[str, Any] = dict(kwargs.get("robot_cameras", {}) or {})
        robot_camera_resolution = kwargs.get("robot_camera_resolution", [640, 400])
        if not isinstance(robot_camera_resolution, (list, tuple)) or len(robot_camera_resolution) < 2:
            robot_camera_resolution = [640, 400]
        self._robot_camera_resolution = (int(robot_camera_resolution[0]), int(robot_camera_resolution[1]))
        self._robot_camera_warmup_steps = int(kwargs.get("robot_camera_warmup_steps", 0))
        self._robot_camera_side_view_enabled = bool(kwargs.get("robot_camera_side_view_enabled", True))
        self._robot_camera_side_view_name = str(kwargs.get("robot_camera_side_view_name", "")).strip()
        self._robot_camera_side_view_title = str(kwargs.get("robot_camera_side_view_title", "Franka Camera")).strip()
        side_view_size = kwargs.get("robot_camera_side_view_size", [560, 320])
        if not isinstance(side_view_size, (list, tuple)) or len(side_view_size) < 2:
            side_view_size = [560, 320]
        self._robot_camera_side_view_size = (int(side_view_size[0]), int(side_view_size[1]))
        self._robot_camera_side_view_pos = tuple(kwargs.get("robot_camera_side_view_position", [1400, 700]))
        self._robot_camera_session: dict[str, Any] | None = None
        self._robot_camera_side_view_session: dict[str, Any] | None = None

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

    def get_profile_path(self) -> Path:
        return _PROFILES_DIR / "franka_simulation.md"

    def load_scene(self, scene: dict[str, dict]) -> None:
        pass

    def execute_action(self, action_type: str, params: dict) -> str:
        handlers = {
            "start": self._start_from_action,
            "enter_simulation": self._start_from_action,
            "close": self._close_api,
            "step": self._step_env,
            "api_call": self._api_call,
            "grasp": self._grasp,
            "pick": self._grasp,
            "grasp_and_place_aside": self._grasp_and_place_aside,
            "place": self._place,
            "release": self._place,
            "register_grasp_target": self._register_grasp_target,
            "register_place_target": self._register_place_target,
            "get_robot_state": self._get_robot_state,
        }
        handler = handlers.get(action_type)
        if handler is None:
            return f"Unknown action: {action_type}"
        try:
            return handler(params or {})
        except Exception as exc:
            return f"Error: {type(exc).__name__}: {exc}"

    def get_scene(self) -> dict[str, Any]:
        return {
            "manipulation_runtime": {
                "location": "sim",
                "state": "running" if self._api is not None else "idle",
                "robot_type": "franka",
                "scene_asset_path": self._scene_asset_path,
                "robot_position": self._robot_position,
                "grasp_targets": list(self._grasp_targets.keys()),
                "place_targets": list(self._place_targets.keys()),
                "last_action_summary": self._last_action_summary,
                "robot_cameras": self._robot_camera_snapshot(),
            }
        }

    def get_runtime_state(self) -> dict[str, Any]:
        robot_state: dict[str, Any] = {
            "connection_state": {
                "status": "connected" if self._api is not None else "idle",
                "transport": "isaac-sim",
                "last_error": None,
            },
            "nav_state": {
                "mode": "manipulation",
                "status": "running" if self._api is not None else "idle",
                "target_ref": {"kind": "object", "id": "franka_pick_cube", "label": "cube"},
            },
        }
        if self._api is None or self._env is None:
            return {"robots": {self.robot_id: robot_state}}
        try:
            obs = self._env.get_observations()
            if isinstance(obs, dict):
                eef_pos = obs.get("eef_position", [0.0, 0.0, 0.0])
                robot_state["eef_position"] = (
                    list(eef_pos) if hasattr(eef_pos, "__iter__") else [float(eef_pos)]
                )
            cameras = self._robot_camera_snapshot()
            if cameras:
                robot_state["cameras"] = cameras
        except Exception:
            pass
        return {"robots": {self.robot_id: robot_state}}

    def health_check(self) -> bool:
        if self._idle_step_enabled:
            self._idle_step_if_due()
        return self._api is not None

    def close(self) -> None:
        self._close_api({})

    def _start_from_action(self, params: dict[str, Any]) -> str:
        if self._api is not None:
            return "Franka Manipulation API already started."

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

        objects_spec = params.get("objects", self._objects_spec)
        robot_usd = str(
            (params or {}).get("robot_usd_path", self._robot_usd_path) or ""
        ).strip()

        self._api = self._build_api(
            scene_asset_path=scene_asset_path,
            robot_position=self._robot_position,
            robot_usd_path=robot_usd,
            objects_spec=objects_spec,
            api_kwargs=api_kwargs,
        )

        self._last_obs = self._api.start()
        self._env = getattr(self._api, "_env", None)

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
        camera_msg = self._maybe_attach_robot_cameras()
        if camera_msg:
            boot_steps.append(camera_msg)

        self._last_action_summary = "started" + (f" [{','.join(boot_steps)}]" if boot_steps else "")
        if boot_steps:
            return f"Franka Manipulation API started. bootstrap[{','.join(boot_steps)}]"
        return "Franka Manipulation API started."

    def _build_api(
        self,
        *,
        scene_asset_path: str,
        robot_position: tuple[float, ...],
        robot_usd_path: str = "",
        objects_spec: Any,
        api_kwargs: dict[str, Any],
    ):
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
        create_franka_robot_cfg = bridge.create_franka_robot_cfg

        robot_cfg = create_franka_robot_cfg(position=robot_position)
        robot_cfg.name = "franka"
        robot_cfg.prim_path = "/franka"
        if robot_usd_path:
            robot_cfg.usd_path = robot_usd_path

        pause_steps = int(api_kwargs.pop("pause_steps", self._pause_steps))
        gripper_settle_steps = int(api_kwargs.pop("gripper_settle_steps", self._gripper_settle_steps))
        arm_waypoint_count = int(api_kwargs.pop("arm_waypoint_count", self._arm_waypoint_count))

        # Piper-style configs use force_gui; FrankaManipulationAPI only accepts headless.
        force_gui = api_kwargs.pop("force_gui", None)
        headless = api_kwargs.pop("headless", None)
        if headless is None:
            if force_gui is not None:
                headless = not bool(force_gui)
            else:
                headless = not self._gui

        task_objects = _build_task_objects_from_spec(objects_spec)
        APICls = _franka_manipulation_api_with_objects_class()
        api = APICls(
            scene_asset_path,
            robot_cfg=robot_cfg,
            objects=task_objects,
            headless=headless,
            pause_steps=pause_steps,
            gripper_settle_steps=gripper_settle_steps,
            arm_waypoint_count=arm_waypoint_count,
            **api_kwargs,
        )

        for name, target in self._grasp_targets.items():
            api.register_grasp_target(name, target)

        for name, target in self._place_targets.items():
            api.register_place_target(name, target)

        return api

    def _close_api(self, _params: dict[str, Any]) -> str:
        if self._api is None:
            return "Franka Manipulation API already closed."
        try:
            self._api.close()
        finally:
            side_view = self._robot_camera_side_view_session or {}
            window = side_view.get("window") if isinstance(side_view, dict) else None
            try:
                if window is not None and hasattr(window, "destroy"):
                    window.destroy()
            except Exception:
                pass
            self._robot_camera_side_view_session = None
            self._robot_camera_session = None
            self._api = None
            self._env = None
        return "Franka Manipulation API closed."

    @staticmethod
    def _camera_prim_path(parent: str, camera_name: str, prefix: str = "paos_") -> str:
        parent_path = str(parent).rstrip("/")
        return f"{parent_path}/{prefix}{camera_name}"

    def _robot_camera_snapshot(self) -> dict[str, dict[str, Any]]:
        if not isinstance(self._robot_cameras_cfg, dict) or not self._robot_cameras_cfg:
            return {}
        session = self._robot_camera_session or {}
        attached = set((session.get("cameras") or {}).keys())
        out: dict[str, dict[str, Any]] = {}
        for camera_name, mount in sorted(self._robot_cameras_cfg.items()):
            if not isinstance(mount, dict):
                continue
            parent = str(mount.get("parent", "")).strip()
            if not parent:
                continue
            out[str(camera_name)] = {
                "status": "attached" if camera_name in attached else "configured",
                "parent": parent,
                "prim_path": self._camera_prim_path(parent, str(camera_name)),
                "resolution": [int(self._robot_camera_resolution[0]), int(self._robot_camera_resolution[1])],
            }
        return out

    def _maybe_attach_robot_cameras(self) -> str:
        if not isinstance(self._robot_cameras_cfg, dict) or not self._robot_cameras_cfg:
            return ""
        if self._robot_camera_session is not None and self._robot_camera_session.get("cameras"):
            return ""
        if self._env is None or self._api is None:
            return "robot_cameras_skipped:api_not_started"

        try:
            from hal.simulation import vla_pick as _camera_utils
        except Exception as exc:
            return f"robot_cameras_skipped:{exc}"

        stage = None
        try:
            stage = self._env.runner._world.stage  # type: ignore[attr-defined]
        except Exception as exc:
            return f"robot_cameras_skipped:{exc}"
        if stage is None:
            return "robot_cameras_skipped:no_stage"

        cameras, _ = _camera_utils.attach_cameras(
            stage=stage,
            env=self._env,
            env_lock=self._env_lock,
            hold_action={},
            mounts=self._robot_cameras_cfg,
            resolution=self._robot_camera_resolution,
            warmup_steps=max(0, self._robot_camera_warmup_steps),
            prim_name_prefix="paos_",
            camera_name_prefix="paos_",
        )
        if not cameras:
            return "robot_cameras_skipped:none_attached"
        self._robot_camera_session = {"cameras": cameras}
        side_view_msg = self._ensure_robot_camera_side_view(cameras)
        base_msg = "robot_cameras:" + ",".join(sorted(cameras))
        if side_view_msg:
            return f"{base_msg} {side_view_msg}".strip()
        return base_msg

    def _ensure_robot_camera_side_view(self, cameras: dict[str, Any]) -> str:
        if not self._robot_camera_side_view_enabled:
            return ""
        if not isinstance(cameras, dict) or not cameras:
            return "robot_camera_side_view_skipped:no_cameras"
        if self._robot_camera_side_view_session is not None:
            return ""

        preferred = self._robot_camera_side_view_name
        camera_name = preferred if preferred in cameras else sorted(cameras.keys())[0]
        mount = self._robot_cameras_cfg.get(camera_name, {}) if isinstance(self._robot_cameras_cfg, dict) else {}
        if not isinstance(mount, dict):
            mount = {}
        parent = str(mount.get("parent", "")).strip()
        if not parent:
            return "robot_camera_side_view_skipped:no_parent"
        camera_prim_path = self._camera_prim_path(parent, str(camera_name))

        viewport_window = None
        viewport_api = None
        try:
            import omni.kit.viewport.utility as vp_utils  # type: ignore
            import omni.ui as ui  # type: ignore

            create_viewport_window = getattr(vp_utils, "create_viewport_window", None)
            if callable(create_viewport_window):
                viewport_window = create_viewport_window(
                    self._robot_camera_side_view_title,
                    width=int(self._robot_camera_side_view_size[0]),
                    height=int(self._robot_camera_side_view_size[1]),
                )
            if viewport_window is None:
                viewport_window = ui.Workspace.get_window(self._robot_camera_side_view_title)
            if viewport_window is not None:
                viewport_api = getattr(viewport_window, "viewport_api", None)
        except Exception as exc:
            return f"robot_camera_side_view_skipped:{exc}"
        if viewport_window is None:
            return "robot_camera_side_view_skipped:no_viewport_window"
        if viewport_api is None or not hasattr(viewport_api, "set_active_camera"):
            return "robot_camera_side_view_skipped:no_viewport_api"

        try:
            viewport_api.set_active_camera(camera_prim_path)
            if hasattr(viewport_window, "width"):
                viewport_window.width = int(self._robot_camera_side_view_size[0])
            if hasattr(viewport_window, "height"):
                viewport_window.height = int(self._robot_camera_side_view_size[1])
            if hasattr(viewport_window, "position_x"):
                viewport_window.position_x = int(self._robot_camera_side_view_pos[0])
            if hasattr(viewport_window, "position_y"):
                viewport_window.position_y = int(self._robot_camera_side_view_pos[1])
        except Exception as exc:
            return f"robot_camera_side_view_skipped:{exc}"

        self._robot_camera_side_view_session = {
            "camera_name": str(camera_name),
            "camera_prim_path": str(camera_prim_path),
            "window": viewport_window,
        }
        return f"robot_camera_side_view:{camera_name}"

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

    def _grasp(self, params: dict[str, Any]) -> str:
        if self._api is None:
            return "Error: API not started. Dispatch action_type='start' first."

        target = params.get("target")
        if not target:
            target = self._resolve_blue_cube_target()
            if target is None:
                return "Error: target is required for grasp action."

        target_dict = self._resolve_target(target, self._grasp_targets)
        self._last_grasp_target = target_dict

        out_dir = Path(params.get("output_dir", self._output_dir))
        out_dir.mkdir(parents=True, exist_ok=True)
        dump_path = out_dir / params.get("dump_name", self._grasp_dump_name)

        result = self._api.grasp(target_dict, dump_path=dump_path)
        self._last_action_summary = (
            f"grasp success={result.success} steps={result.steps}"
        )
        pos = target_dict.get("position")
        if isinstance(pos, (list, tuple)) and len(pos) >= 3:
            self._last_action_summary += (
                f" target_pos=({float(pos[0]):.3f}, {float(pos[1]):.3f}, {float(pos[2]):.3f})"
            )
        return self._last_action_summary

    def _place(self, params: dict[str, Any]) -> str:
        if self._api is None:
            return "Error: API not started. Dispatch action_type='start' first."

        target = params.get("target")
        if not target:
            target = self._resolve_place_aside_target()
            if target is None:
                return "Error: target is required for place action."

        target_dict = self._resolve_target(target, self._place_targets)

        out_dir = Path(params.get("output_dir", self._output_dir))
        out_dir.mkdir(parents=True, exist_ok=True)
        dump_path = out_dir / params.get("dump_name", self._place_dump_name)

        result = self._api.place(target_dict, dump_path=dump_path)
        self._last_action_summary = (
            f"place success={result.success} steps={result.steps}"
        )
        pos = target_dict.get("position")
        if isinstance(pos, (list, tuple)) and len(pos) >= 3:
            self._last_action_summary += (
                f" target_pos=({float(pos[0]):.3f}, {float(pos[1]):.3f}, {float(pos[2]):.3f})"
            )
        return self._last_action_summary

    def _grasp_and_place_aside(self, params: dict[str, Any]) -> str:
        if self._api is None:
            return "Error: API not started. Dispatch action_type='start' first."
        grasp_result = self._grasp(dict(params))
        if grasp_result.startswith("Error:"):
            return grasp_result
        place_params = dict(params)
        place_params.pop("target", None)
        place_result = self._place(place_params)
        if place_result.startswith("Error:"):
            return f"{grasp_result}; {place_result}"
        return f"{grasp_result}; {place_result}"

    def _resolve_blue_cube_target(self) -> dict[str, Any] | None:
        cube_candidates: list[dict[str, Any]] = []
        for item in self._objects_spec if isinstance(self._objects_spec, list) else []:
            if not isinstance(item, dict):
                continue
            name = str(item.get("name", "")).strip().lower()
            if "cube" not in name:
                continue
            pos = item.get("position")
            if not (isinstance(pos, (list, tuple)) and len(pos) >= 3):
                continue
            color = item.get("color")
            blue_score = -1.0
            if isinstance(color, (list, tuple)) and len(color) >= 3:
                try:
                    blue_score = float(color[2]) - 0.5 * float(color[0]) - 0.5 * float(color[1])
                except (TypeError, ValueError):
                    blue_score = -1.0
            cube_candidates.append({
                "name": item.get("name", "cube"),
                "position": [float(pos[0]), float(pos[1]), float(pos[2])],
                "scale": item.get("scale"),
                "blue_score": blue_score,
            })

        if not cube_candidates:
            return None
        chosen = sorted(cube_candidates, key=lambda x: x["blue_score"], reverse=True)[0]
        x, y, z = chosen["position"]
        scale = chosen.get("scale")
        cube_height = 0.05
        if isinstance(scale, (list, tuple)) and len(scale) >= 3:
            try:
                cube_height = float(scale[2])
            except (TypeError, ValueError):
                cube_height = 0.05
        contact_z = z + cube_height * 0.45
        return {
            "name": str(chosen.get("name") or "blue_cube"),
            "position": [x, y, contact_z],
            "pre_position": [x, y, contact_z + 0.18],
            "post_position": [x, y, contact_z + 0.12],
            "orientation": [0.0, 0.0, 1.0, 0.0],
            "metadata": {"auto_resolved": True, "source": "objects_spec"},
        }

    def _resolve_place_aside_target(self) -> dict[str, Any] | None:
        source = self._last_grasp_target or self._resolve_blue_cube_target()
        if not isinstance(source, dict):
            return None
        pos = source.get("position")
        if not (isinstance(pos, (list, tuple)) and len(pos) >= 3):
            return None
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        # Place "to one side": shift along +Y on the same table height.
        y_side = y + 0.18
        return {
            "name": "auto_place_aside",
            "position": [x, y_side, z],
            "pre_position": [x, y_side, z + 0.22],
            "post_position": [x, y_side, z + 0.20],
            "orientation": [0.0, 0.0, 1.0, 0.0],
            "metadata": {"auto_resolved": True, "source": "place_aside_offset"},
        }

    def _resolve_target(
        self,
        target: str | dict[str, Any],
        registry: dict[str, dict[str, Any]],
    ) -> dict[str, Any]:
        if isinstance(target, dict):
            return target
        if isinstance(target, str):
            if target in registry:
                return registry[target]
            raise KeyError(f"Unknown target: {target}. Available: {list(registry.keys())}")
        raise TypeError(f"Unsupported target type: {type(target)}")

    def _register_grasp_target(self, params: dict[str, Any]) -> str:
        name = str(params.get("name", "")).strip()
        if not name:
            return "Error: name is required for register_grasp_target."
        target = params.get("target", {})
        if not isinstance(target, dict):
            return "Error: target must be a dict."
        self._grasp_targets[name] = target
        if self._api is not None:
            self._api.register_grasp_target(name, target)
        return f"grasp_target registered: {name}"

    def _register_place_target(self, params: dict[str, Any]) -> str:
        name = str(params.get("name", "")).strip()
        if not name:
            return "Error: name is required for register_place_target."
        target = params.get("target", {})
        if not isinstance(target, dict):
            return "Error: target must be a dict."
        self._place_targets[name] = target
        if self._api is not None:
            self._api.register_place_target(name, target)
        return f"place_target registered: {name}"

    def _get_robot_state(self, params: dict[str, Any]) -> str:
        if self._api is None:
            return "Error: API not started. Dispatch action_type='start' first."
        try:
            obs = self._env.get_observations() if self._env else None
            if isinstance(obs, dict):
                eef_pos = obs.get("eef_position", [0.0, 0.0, 0.0])
                if hasattr(eef_pos, "__iter__"):
                    eef_pos = list(eef_pos)
                else:
                    eef_pos = [float(eef_pos)]
                state = {
                    "eef_position": eef_pos,
                    "last_action": self._last_action_summary,
                    "grasp_targets": list(self._grasp_targets.keys()),
                    "place_targets": list(self._place_targets.keys()),
                }
                return f"Robot State: {self._safe_json(state)}"
            return "Robot State: observation format unknown"
        except Exception as exc:
            return f"Error getting robot state: {exc}"

    def _idle_step_if_due(self) -> None:
        if self._api is None or self._env is None:
            return
        now = time.monotonic()
        if now - self._last_idle_step_ts < self._idle_step_interval_s:
            return
        self._last_idle_step_ts = now
        steps = max(1, self._idle_steps_per_cycle)
        for _ in range(steps):
            with self._env_lock:
                self._last_obs, _, _, _, _ = self._env.step(action={})

    @staticmethod
    def _safe_json(value: Any) -> str:
        try:
            return json.dumps(value, ensure_ascii=False, default=str)
        except TypeError:
            return str(value)
