"""G1 navigation HAL driver backed by InternUtopia simulation."""

from __future__ import annotations

import importlib
from pathlib import Path
from typing import Any

from hal.drivers.pipergo2_manipulation_driver import PiperGo2ManipulationDriver

_PROFILES_DIR = Path(__file__).resolve().parent.parent / "profiles"


class _G1NavigationAPI:
    def __init__(
        self,
        *,
        scene_asset_path: str,
        robot_start: tuple[float, float, float],
        robot_usd_path: str,
        objects: list[Any],
        headless: bool,
        **kwargs: Any,
    ) -> None:
        self._scene_asset_path = str(scene_asset_path)
        self._robot_start = tuple(float(x) for x in robot_start[:3])
        self._robot_usd_path = str(robot_usd_path or "").strip()
        self._objects = list(objects)
        self._headless = bool(headless)
        self._kwargs = dict(kwargs)
        self._env = None

    def start(self) -> Any:
        from internutopia.core.config import Config, SimConfig
        from internutopia.core.gym_env import Env
        from internutopia_extension import import_extensions
        from internutopia_extension.configs.robots.g1 import (
            G1RobotCfg,
            move_along_path_cfg,
            move_by_speed_cfg,
            move_to_point_cfg,
            rotate_cfg,
        )
        from internutopia_extension.configs.tasks import SingleInferenceTaskCfg

        physics_dt = float(self._kwargs.pop("physics_dt", 1 / 240))
        rendering_dt = float(self._kwargs.pop("rendering_dt", physics_dt))
        use_fabric = bool(self._kwargs.pop("use_fabric", False))
        webrtc = bool(self._kwargs.pop("webrtc", self._headless))

        local_move_by_speed_cfg = move_by_speed_cfg.copy(deep=True)
        local_move_to_point_cfg = move_to_point_cfg.copy(deep=True)
        local_move_along_path_cfg = move_along_path_cfg.copy(deep=True)
        local_rotate_cfg = rotate_cfg.copy(deep=True)

        if self._robot_usd_path:
            local_asset_root = Path(self._robot_usd_path).expanduser().resolve().parent
            local_policy_path = local_asset_root / "policy" / "move_by_speed" / "g1_15000.onnx"
            if local_policy_path.exists():
                local_move_by_speed_cfg.policy_weights_path = str(local_policy_path)

        local_move_to_point_cfg.sub_controllers = [local_move_by_speed_cfg]
        local_move_along_path_cfg.sub_controllers = [local_move_to_point_cfg]
        local_rotate_cfg.sub_controllers = [local_move_by_speed_cfg]

        robot_cfg = G1RobotCfg(
            position=self._robot_start,
            controllers=[
                local_move_to_point_cfg,
                local_move_by_speed_cfg,
                local_move_along_path_cfg,
                local_rotate_cfg,
            ],
        )
        if self._robot_usd_path:
            robot_cfg.usd_path = self._robot_usd_path

        config = Config(
            simulator=SimConfig(
                physics_dt=physics_dt,
                rendering_dt=rendering_dt,
                use_fabric=use_fabric,
                headless=self._headless,
                webrtc=webrtc,
            ),
            task_configs=[
                SingleInferenceTaskCfg(
                    scene_asset_path=self._scene_asset_path,
                    robots=[robot_cfg],
                    objects=self._objects,
                )
            ],
        )

        import_extensions()
        self._env = Env(config)
        obs, _ = self._env.reset()
        return obs

    def close(self) -> None:
        if self._env is None:
            return
        self._env.close()
        self._env = None


class G1NavigationDriver(PiperGo2ManipulationDriver):
    """Navigation-only G1 simulator driver with robot-mounted cameras."""

    def __init__(self, gui: bool = False, *, robot_id: str | None = None, **kwargs: Any) -> None:
        kwargs = dict(kwargs)
        kwargs.setdefault("robot_start", (0.58, 7.50704, 0.8))
        kwargs.setdefault("robot_camera_warmup_steps", 8)
        super().__init__(gui=gui, robot_id=robot_id or "g1_001", **kwargs)

    def get_profile_path(self) -> Path:
        return _PROFILES_DIR / "g1_navigation.md"

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
            "run_vla_pick_and_return": self._unsupported_vla_action,
            "run_pick_place": self._unsupported_vla_action,
        }
        handler = handlers.get(action_type)
        if handler is None:
            return f"Unknown action: {action_type}"
        try:
            return handler(params or {})
        except Exception as exc:  # pragma: no cover - runtime dependency bridge
            import sys
            import traceback

            print(
                f"[g1] action {action_type!r} failed:",
                file=sys.stderr,
                flush=True,
            )
            traceback.print_exc()
            return f"Error: {type(exc).__name__}: {exc}"

    def get_scene(self) -> dict[str, dict]:
        scene = super().get_scene()
        runtime = dict(scene.pop("manipulation_runtime", {}) or {})
        runtime.pop("last_pick_place_cn", None)
        runtime["state"] = "running" if self._api is not None else "idle"
        if self._vla_cfg:
            runtime["smolvla_status"] = "unsupported_for_navigation_only"
        scene["navigation_runtime"] = runtime
        return scene

    def get_runtime_state(self) -> dict[str, Any]:
        runtime = super().get_runtime_state()
        robot_state = runtime.get("robots", {}).get(self.robot_id, {})
        nav_state = robot_state.get("nav_state")
        if isinstance(nav_state, dict):
            nav_state["mode"] = "navigation"
        return runtime

    def _start_from_action(self, params: dict[str, Any]) -> str:
        return super()._start_from_action(params).replace("Manipulation API", "G1 navigation API")

    def _close_api(self, params: dict[str, Any]) -> str:
        return super()._close_api(params).replace("Manipulation API", "G1 navigation API")

    def _maybe_preheat_vla_session(self) -> str:
        if self._vla_cfg:
            return "smolvla:navigation_only_unsupported"
        return ""

    def _unsupported_vla_action(self, _params: dict[str, Any]) -> str:
        return "Error: G1 navigation driver does not support manipulation or SmolVLA deploy actions."

    def _resolve_nav_action_name(self) -> str:
        if self._navigation_action_name:
            return str(self._navigation_action_name)
        self._ensure_pythonpath()
        rob = importlib.import_module("internutopia_extension.configs.robots.g1")
        return rob.move_to_point_cfg.name

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
        # G1 is navigation-only; if API is not ready, lazily bootstrap once
        # so plain language commands like "move to the table" can run directly.
        if self._env is None:
            bootstrap_msg = self._start_from_action({})
            if self._env is None:
                return bootstrap_msg
        return super()._navigate_xy(
            xy,
            max_steps=max_steps,
            threshold=threshold,
            action_name_override=action_name_override,
            arm_target=arm_target,
            arm_action_name=arm_action_name,
        )

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
        del arm_mass_scale

        pythonpath = api_kwargs.pop("pythonpath", None)
        if pythonpath:
            if isinstance(pythonpath, str):
                pythonpath = [pythonpath]
            import sys

            for entry in reversed(pythonpath):
                ep = str(Path(str(entry)).expanduser().resolve())
                if ep not in sys.path:
                    sys.path.insert(0, ep)

        objects_mod = importlib.import_module("internutopia_extension.configs.objects")
        DynamicCubeCfg = objects_mod.DynamicCubeCfg
        VisualCubeCfg = objects_mod.VisualCubeCfg

        rs = tuple(float(x) for x in (robot_start if isinstance(robot_start, (list, tuple)) else tuple(robot_start)))
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

        return _G1NavigationAPI(
            scene_asset_path=scene_asset_path,
            robot_start=rs,
            robot_usd_path=robot_usd_path,
            objects=objects,
            headless=bool(headless),
            **api_kwargs,
        )

    @staticmethod
    def _obs_brief(obs_data: Any) -> dict[str, Any] | None:
        robot = G1NavigationDriver._extract_robot_obs(obs_data)
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
    def _extract_robot_obs(obs_data: Any) -> dict[str, Any] | None:
        if isinstance(obs_data, dict) and "position" in obs_data:
            return obs_data
        if isinstance(obs_data, dict) and "g1" in obs_data:
            return obs_data["g1"]
        if isinstance(obs_data, dict) and "g1_0" in obs_data:
            return obs_data["g1_0"]
        if isinstance(obs_data, (list, tuple)) and len(obs_data) > 0:
            first = obs_data[0]
            if isinstance(first, dict) and "g1" in first:
                return first["g1"]
            if isinstance(first, dict) and "g1_0" in first:
                return first["g1_0"]
            if isinstance(first, dict):
                return first
        return None