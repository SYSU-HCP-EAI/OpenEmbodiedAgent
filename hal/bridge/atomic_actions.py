from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import copy
import json
import re
from typing import Any, Sequence

import numpy as np

from internutopia.core.util.joint import create_joint
from .piper_ik_solver import PiperIKSolver


_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_REPO_ROOT = Path(__file__).resolve().parents[2]
_LOCAL_ASSET_ROOT = _PACKAGE_ROOT / "assets"
_PG2_USD_ASSERTS = _REPO_ROOT / "asserts" / "robots" / "pipergo2" / "pipergo2.usd"
_DEFAULT_PIPERGO2_USD = str(
    _PG2_USD_ASSERTS
    if _PG2_USD_ASSERTS.is_file()
    else _REPO_ROOT / "examples" / "robots" / "pipergo2" / "pipergo2.usd"
)

_SINGLE_ACTION_RE = re.compile(
    r"^<(?P<agent_name>[^>]+)>\((?P<agent_id>\d+)\): "
    r"\[(?P<action>[a-z_]+)\] "
    r"<(?P<object_name>[^>]+)>\((?P<object_id>\d+)\)$"
)
_PLACE_ACTION_RE = re.compile(
    r"^<(?P<agent_name>[^>]+)>\((?P<agent_id>\d+)\): "
    r"\[(?P<action>puton|putinto)\] "
    r"<(?P<object_name>[^>]+)>\((?P<object_id>\d+)\) "
    r"(?P<relation>on|into) "
    r"<(?P<target_name>[^>]+)>\((?P<target_id>\d+)\)$"
)


@dataclass(slots=True)
class NavigationTarget:
    position: tuple[float, float, float]
    name: str | None = None


@dataclass(slots=True)
class ManipulationTarget:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    pre_position: tuple[float, float, float] | None = None
    post_position: tuple[float, float, float] | None = None
    name: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass(slots=True)
class AtomicAction:
    agent_name: str
    agent_id: int
    action: str
    object_name: str
    object_id: int
    relation: str | None = None
    target_name: str | None = None
    target_id: int | None = None
    raw: str | None = None

    @property
    def bridge_action(self) -> str:
        mapping = {
            "movetowards": "navigate",
            "grab": "grasp",
            "puton": "place",
            "putinto": "place",
        }
        return mapping.get(self.action, self.action)


@dataclass(slots=True)
class ActionResult:
    action: str
    success: bool
    steps: int
    final_observation: dict[str, Any]
    trace: list[dict[str, Any]]
    metadata: dict[str, Any] = field(default_factory=dict)

    def dump_json(self, path: str | Path) -> None:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "action": self.action,
            "success": self.success,
            "steps": self.steps,
            "final_observation": _to_builtin(self.final_observation),
            "trace": _to_builtin(self.trace),
            "metadata": _to_builtin(self.metadata),
        }
        target.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def parse_coherent_action(action_text: str) -> AtomicAction:
    action_text = action_text.strip()
    place_match = _PLACE_ACTION_RE.match(action_text)
    if place_match:
        groups = place_match.groupdict()
        return AtomicAction(
            agent_name=groups["agent_name"],
            agent_id=int(groups["agent_id"]),
            action=groups["action"],
            object_name=groups["object_name"],
            object_id=int(groups["object_id"]),
            relation=groups["relation"],
            target_name=groups["target_name"],
            target_id=int(groups["target_id"]),
            raw=action_text,
        )

    single_match = _SINGLE_ACTION_RE.match(action_text)
    if single_match:
        groups = single_match.groupdict()
        return AtomicAction(
            agent_name=groups["agent_name"],
            agent_id=int(groups["agent_id"]),
            action=groups["action"],
            object_name=groups["object_name"],
            object_id=int(groups["object_id"]),
            raw=action_text,
        )

    raise ValueError(f"Unsupported coherent action format: {action_text}")


def _to_builtin(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: _to_builtin(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_builtin(v) for v in value]
    return value


def _normalize_xyz(position: Sequence[float]) -> tuple[float, float, float]:
    if len(position) != 3:
        raise ValueError(f"Expected xyz with length 3, got {position}")
    return tuple(float(v) for v in position)


def _normalize_quat(orientation: Sequence[float] | None) -> tuple[float, float, float, float]:
    if orientation is None:
        return (1.0, 0.0, 0.0, 0.0)
    if len(orientation) != 4:
        raise ValueError(f"Expected quaternion with length 4, got {orientation}")
    return tuple(float(v) for v in orientation)


def _local_asset_path(relative_path: str) -> str:
    """Resolve ``asserts/<relative>`` then ``examples/<relative>``, then ``hal/bridge/assets``."""
    relative_path = relative_path.lstrip("/")
    for sub in ("asserts", "examples"):
        cand = _REPO_ROOT / sub / relative_path
        if cand.is_file():
            return str(cand)
    return str(_LOCAL_ASSET_ROOT / relative_path)


def _rewrite_asset_path(value: Any) -> Any:
    if not isinstance(value, str):
        return value
    marker = "/internutopia/assets/"
    if marker in value:
        relative = value.split(marker, 1)[1]
        return _local_asset_path(relative)
    return value


def _rewrite_controller_asset_paths(config: Any) -> Any:
    if config is None:
        return config

    for attr_name in dir(config):
        if attr_name.startswith("_"):
            continue
        try:
            attr_value = getattr(config, attr_name)
        except Exception:
            continue

        if isinstance(attr_value, str):
            rewritten = _rewrite_asset_path(attr_value)
            if rewritten != attr_value:
                try:
                    setattr(config, attr_name, rewritten)
                except Exception:
                    pass
        elif isinstance(attr_value, list):
            rewritten_items = []
            changed = False
            for item in attr_value:
                if isinstance(item, str):
                    rewritten_item = _rewrite_asset_path(item)
                    rewritten_items.append(rewritten_item)
                    changed = changed or rewritten_item != item
                else:
                    rewritten_items.append(_rewrite_controller_asset_paths(item))
            if changed:
                try:
                    setattr(config, attr_name, rewritten_items)
                except Exception:
                    pass

    return config


def create_h1_robot_cfg(
    *,
    position: Sequence[float] = (0.0, 0.0, 1.05),
    include_camera: bool = False,
    camera_name: str = "camera",
    camera_resolution: tuple[int, int] = (320, 240),
) -> Any:
    from internutopia_extension.configs.robots.h1 import (
        H1RobotCfg,
        h1_camera_cfg,
        move_by_speed_cfg,
        move_to_point_cfg,
        rotate_cfg,
    )

    move_by_speed = _rewrite_controller_asset_paths(copy.deepcopy(move_by_speed_cfg))
    move_to_point = _rewrite_controller_asset_paths(copy.deepcopy(move_to_point_cfg))
    rotate = _rewrite_controller_asset_paths(copy.deepcopy(rotate_cfg))

    move_to_point.sub_controllers = [move_by_speed]
    rotate.sub_controllers = [move_by_speed]

    sensors = []
    if include_camera:
        camera_cfg = copy.deepcopy(h1_camera_cfg)
        camera_cfg.name = camera_name
        camera_cfg.resolution = camera_resolution
        sensors.append(camera_cfg)

    return H1RobotCfg(
        position=tuple(position),
        usd_path=_local_asset_path("robots/h1/h1.usd"),
        controllers=[move_by_speed, move_to_point, rotate],
        sensors=sensors,
    )


def create_aliengo_robot_cfg(
    *,
    position: Sequence[float] = (0.0, 0.0, 1.05),
) -> Any:
    from internutopia_extension.configs.robots.aliengo import (
        AliengoRobotCfg,
        move_by_speed_cfg,
        move_to_point_cfg,
        rotate_cfg,
    )

    move_by_speed = _rewrite_controller_asset_paths(copy.deepcopy(move_by_speed_cfg))
    move_to_point = _rewrite_controller_asset_paths(copy.deepcopy(move_to_point_cfg))
    rotate = _rewrite_controller_asset_paths(copy.deepcopy(rotate_cfg))

    move_to_point.sub_controllers = [move_by_speed]
    rotate.sub_controllers = [move_by_speed]

    return AliengoRobotCfg(
        position=tuple(position),
        usd_path=_local_asset_path("robots/aliengo/aliengo_camera.usd"),
        controllers=[move_by_speed, move_to_point, rotate],
    )


def create_g1_robot_cfg(
    *,
    position: Sequence[float] = (0.0, 0.0, 0.78),
) -> Any:
    from internutopia_extension.configs.robots.g1 import (
        G1RobotCfg,
        move_by_speed_cfg,
        move_to_point_cfg,
        rotate_cfg,
    )

    move_by_speed = _rewrite_controller_asset_paths(copy.deepcopy(move_by_speed_cfg))
    move_to_point = _rewrite_controller_asset_paths(copy.deepcopy(move_to_point_cfg))
    rotate = _rewrite_controller_asset_paths(copy.deepcopy(rotate_cfg))

    move_to_point.sub_controllers = [move_by_speed]
    rotate.sub_controllers = [move_by_speed]

    return G1RobotCfg(
        position=tuple(position),
        usd_path=_local_asset_path("robots/g1/g1_29dof_color.usd"),
        controllers=[move_by_speed, move_to_point, rotate],
    )


def create_franka_robot_cfg(
    *,
    position: Sequence[float] = (0.0, 0.0, 0.0),
) -> Any:
    from internutopia_extension.configs.robots.franka import FrankaRobotCfg, arm_ik_cfg, gripper_cfg

    arm_ik = _rewrite_controller_asset_paths(copy.deepcopy(arm_ik_cfg))
    gripper = copy.deepcopy(gripper_cfg)

    return FrankaRobotCfg(
        position=tuple(position),
        usd_path=_local_asset_path("robots/franka/franka.usd"),
        controllers=[arm_ik, gripper],
    )


def create_pipergo2_robot_cfg(
    *,
    position: Sequence[float] = (0.0, 0.0, 0.55),
    arm_mass_scale: float = 0.35,
) -> Any:
    from internutopia_extension.configs.controllers import JointControllerCfg
    from internutopia_extension.configs.robots.pipergo2 import (
        PiperGo2RobotCfg,
        move_by_speed_cfg,
        move_to_point_cfg,
        rotate_cfg,
    )

    move_by_speed = _rewrite_controller_asset_paths(copy.deepcopy(move_by_speed_cfg))
    move_to_point = _rewrite_controller_asset_paths(copy.deepcopy(move_to_point_cfg))
    rotate = _rewrite_controller_asset_paths(copy.deepcopy(rotate_cfg))

    move_to_point.sub_controllers = [move_by_speed]
    rotate.sub_controllers = [move_by_speed]

    arm_joint_controller = JointControllerCfg(
        name='arm_joint_controller',
        joint_names=[
            'piper_j1',
            'piper_j2',
            'piper_j3',
            'piper_j4',
            'piper_j5',
            'piper_j6',
            'piper_j7',
            'piper_j8',
        ],
    )

    return PiperGo2RobotCfg(
        position=tuple(position),
        usd_path=_DEFAULT_PIPERGO2_USD,
        controllers=[move_by_speed, move_to_point, rotate, arm_joint_controller],
        arm_mass_scale=arm_mass_scale,
    )


def create_xlerobot_robot_cfg(
    *,
    position: Sequence[float] = (0.0, 0.0, 0.02),
    arm_kps: Sequence[float] | None = None,
    arm_kds: Sequence[float] | None = None,
    gripper_kps: Sequence[float] | None = None,
    gripper_kds: Sequence[float] | None = None,
    head_kps: Sequence[float] | None = None,
    head_kds: Sequence[float] | None = None,
    base_kps: Sequence[float] | None = None,
    base_kds: Sequence[float] | None = None,
    solver_position_iteration_count: int | None = None,
    solver_velocity_iteration_count: int | None = None,
    enable_self_collisions: bool | None = None,
    enable_manipulator_pose_hold: bool | None = None,
) -> Any:
    from internutopia_extension.configs.robots.xlerobot import (
        XLEROBOT_DEFAULT_PHYSICS,
        XlerobotRobotCfg,
        move_to_point_cfg,
    )

    move_to_point = copy.deepcopy(move_to_point_cfg)
    physics_defaults = XLEROBOT_DEFAULT_PHYSICS

    return XlerobotRobotCfg(
        position=tuple(position),
        controllers=[move_to_point],
        arm_joint_names=tuple(physics_defaults["arm_joint_names"]),
        gripper_joint_names=tuple(physics_defaults["gripper_joint_names"]),
        head_joint_names=tuple(physics_defaults["head_joint_names"]),
        base_joint_names=tuple(physics_defaults["base_joint_names"]),
        arm_kps=tuple(arm_kps or physics_defaults["arm_kps"]),
        arm_kds=tuple(arm_kds or physics_defaults["arm_kds"]),
        gripper_kps=tuple(gripper_kps or physics_defaults["gripper_kps"]),
        gripper_kds=tuple(gripper_kds or physics_defaults["gripper_kds"]),
        head_kps=tuple(head_kps or physics_defaults["head_kps"]),
        head_kds=tuple(head_kds or physics_defaults["head_kds"]),
        base_kps=tuple(base_kps or physics_defaults["base_kps"]),
        base_kds=tuple(base_kds or physics_defaults["base_kds"]),
        solver_position_iteration_count=(
            physics_defaults["solver_position_iteration_count"]
            if solver_position_iteration_count is None
            else solver_position_iteration_count
        ),
        solver_velocity_iteration_count=(
            physics_defaults["solver_velocity_iteration_count"]
            if solver_velocity_iteration_count is None
            else solver_velocity_iteration_count
        ),
        enable_self_collisions=(
            physics_defaults["enable_self_collisions"]
            if enable_self_collisions is None
            else enable_self_collisions
        ),
        enable_manipulator_pose_hold=True if enable_manipulator_pose_hold is None else bool(enable_manipulator_pose_hold),
    )


class H1NavigateAPI:
    def __init__(
        self,
        scene_asset_path: str,
        robot_cfg: Any | None = None,
        headless: bool | None = None,
        max_steps_per_action: int = 2000,
    ) -> None:
        self.scene_asset_path = scene_asset_path
        self.max_steps_per_action = max_steps_per_action
        self._targets: dict[str, NavigationTarget] = {}
        self._env = None
        self._move_controller_name = "move_to_point"

        if headless is None:
            from internutopia.core.util import has_display

            self.headless = not has_display()
        else:
            self.headless = headless
        self.robot_cfg = robot_cfg or self._build_default_robot_cfg()

    def _build_default_robot_cfg(self) -> Any:
        return create_h1_robot_cfg()

    def start(self) -> dict[str, Any]:
        if self._env is not None:
            obs = self._env.get_observations()
            return _to_builtin(obs)

        from internutopia.core.config import Config, SimConfig
        from internutopia.core.gym_env import Env
        from internutopia_extension import import_extensions
        from internutopia_extension.configs.tasks import SingleInferenceTaskCfg

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
                ),
            ],
        )
        import_extensions()
        self._env = Env(config)
        obs, _ = self._env.reset()
        return _to_builtin(obs)

    def close(self) -> None:
        if self._env is not None:
            self._env.close()
            self._env = None

    def register_target(self, name: str, position: Sequence[float]) -> None:
        self._targets[name] = NavigationTarget(position=_normalize_xyz(position), name=name)

    def reset(self) -> dict[str, Any]:
        self._require_env()
        obs, _ = self._env.reset()
        return _to_builtin(obs)

    def navigate(
        self,
        target: str | Sequence[float] | NavigationTarget,
        max_steps: int | None = None,
        dump_path: str | Path | None = None,
        step_callback: Any | None = None,
    ) -> ActionResult:
        self._require_env()
        nav_target = self._resolve_target(target)
        command = {self._move_controller_name: [np.array(nav_target.position)]}
        step_limit = max_steps or self.max_steps_per_action

        trace: list[dict[str, Any]] = []
        final_obs: dict[str, Any] = {}
        success = False

        for step in range(1, step_limit + 1):
            obs, _, terminated, _, _ = self._env.step(action=command)
            if step_callback is not None:
                step_callback(step, obs, nav_target)
            final_obs = _to_builtin(obs)
            trace.append(
                {
                    "step": step,
                    "action": "navigate",
                    "target": nav_target.name or nav_target.position,
                    "observation": final_obs,
                }
            )

            controller_obs = obs.get("controllers", {}).get(self._move_controller_name, {})
            if controller_obs.get("finished", False):
                success = True
                break
            if terminated:
                break

        result = ActionResult(
            action="navigate",
            success=success,
            steps=len(trace),
            final_observation=final_obs,
            trace=trace,
            metadata={
                "target_name": nav_target.name,
                "target_position": nav_target.position,
                "controller": self._move_controller_name,
            },
        )
        if dump_path is not None:
            result.dump_json(dump_path)
        return result

    def _resolve_target(self, target: str | Sequence[float] | NavigationTarget) -> NavigationTarget:
        if isinstance(target, NavigationTarget):
            return target
        if isinstance(target, str):
            if target not in self._targets:
                raise KeyError(f"Unknown navigation target: {target}")
            return self._targets[target]
        return NavigationTarget(position=_normalize_xyz(target))

    def _require_env(self) -> None:
        if self._env is None:
            self.start()

    @property
    def env(self):
        self._require_env()
        return self._env


class MultiRobotNavigateAPI:
    def __init__(
        self,
        scene_asset_path: str,
        robot_cfgs: Sequence[Any],
        headless: bool | None = None,
        max_steps_per_action: int = 2000,
    ) -> None:
        self.scene_asset_path = scene_asset_path
        self.max_steps_per_action = max_steps_per_action
        self._robot_cfgs = list(robot_cfgs)
        self._targets: dict[str, dict[str, NavigationTarget]] = {}
        self._env = None
        self._move_controller_name = "move_to_point"

        if headless is None:
            from internutopia.core.util import has_display

            self.headless = not has_display()
        else:
            self.headless = headless

    def start(self) -> dict[str, Any]:
        if self._env is not None:
            observations = self._env.get_observations()
            return _to_builtin(observations[0] if observations else {})

        from internutopia.core.config import Config, SimConfig
        from internutopia.core.vec_env import Env
        from internutopia_extension import import_extensions
        from internutopia_extension.configs.tasks import SingleInferenceTaskCfg

        config = Config(
            simulator=SimConfig(
                physics_dt=1 / 240,
                rendering_dt=1 / 240,
                use_fabric=False,
                rendering_interval=0,
                headless=self.headless,
                native=self.headless,
                webrtc=self.headless,
            ),
            env_num=1,
            metrics_save_path="none",
            task_configs=[
                SingleInferenceTaskCfg(
                    scene_asset_path=self.scene_asset_path,
                    robots=self._robot_cfgs,
                )
            ],
        )
        import_extensions()
        self._env = Env(config)
        observations, _ = self._env.reset()
        return _to_builtin(observations[0] if observations else {})

    def close(self) -> None:
        if self._env is not None:
            self._env.close()
            self._env = None

    def register_target(self, robot_name: str, name: str, position: Sequence[float]) -> None:
        if robot_name not in self._targets:
            self._targets[robot_name] = {}
        self._targets[robot_name][name] = NavigationTarget(position=_normalize_xyz(position), name=name)

    def navigate_all(
        self,
        targets: dict[str, str | Sequence[float] | NavigationTarget],
        max_steps: int | None = None,
        dump_path: str | Path | None = None,
        step_callback: Any | None = None,
    ) -> ActionResult:
        self._require_env()

        resolved_targets = {
            robot_name: self._resolve_target(robot_name, target)
            for robot_name, target in targets.items()
        }
        env_action = [
            {
                robot_name: {
                    self._move_controller_name: [np.array(nav_target.position)],
                }
                for robot_name, nav_target in resolved_targets.items()
            }
        ]

        trace: list[dict[str, Any]] = []
        final_obs: dict[str, Any] = {}
        success = False
        step_limit = max_steps or self.max_steps_per_action

        for step in range(1, step_limit + 1):
            env_obs, _, _, _, _ = self._env.step(action=env_action)
            obs = env_obs[0]
            if step_callback is not None:
                step_callback(step, obs, resolved_targets)

            final_obs = _to_builtin(obs)
            trace.append(
                {
                    "step": step,
                    "action": "navigate_all",
                    "targets": {
                        robot_name: nav_target.name or nav_target.position
                        for robot_name, nav_target in resolved_targets.items()
                    },
                    "observation": final_obs,
                }
            )

            finished = []
            for robot_name in resolved_targets:
                controller_obs = obs.get(robot_name, {}).get("controllers", {}).get(self._move_controller_name, {})
                finished.append(controller_obs.get("finished", False))
            if finished and all(finished):
                success = True
                break

        result = ActionResult(
            action="navigate_all",
            success=success,
            steps=len(trace),
            final_observation=final_obs,
            trace=trace,
            metadata={
                "targets": {
                    robot_name: nav_target.position
                    for robot_name, nav_target in resolved_targets.items()
                },
                "controller": self._move_controller_name,
            },
        )
        if dump_path is not None:
            result.dump_json(dump_path)
        return result

    def _resolve_target(
        self,
        robot_name: str,
        target: str | Sequence[float] | NavigationTarget,
    ) -> NavigationTarget:
        if isinstance(target, NavigationTarget):
            return target
        if isinstance(target, str):
            if robot_name not in self._targets or target not in self._targets[robot_name]:
                raise KeyError(f"Unknown navigation target for {robot_name}: {target}")
            return self._targets[robot_name][target]
        return NavigationTarget(position=_normalize_xyz(target))

    def _require_env(self) -> None:
        if self._env is None:
            self.start()

    @property
    def env(self):
        self._require_env()
        return self._env


class FrankaManipulationAPI:
    def __init__(
        self,
        scene_asset_path: str,
        robot_cfg: Any | None = None,
        headless: bool | None = None,
        max_steps_per_phase: int = 600,
        gripper_settle_steps: int = 30,
        pause_steps: int = 45,
        arm_waypoint_count: int = 4,
    ) -> None:
        self.scene_asset_path = scene_asset_path
        self.max_steps_per_phase = max_steps_per_phase
        self.gripper_settle_steps = gripper_settle_steps
        self.pause_steps = pause_steps
        self.arm_waypoint_count = arm_waypoint_count
        self._env = None
        self._grasp_targets: dict[str, ManipulationTarget] = {}
        self._place_targets: dict[str, ManipulationTarget] = {}
        self._arm_controller_name = "arm_ik_controller"
        self._gripper_controller_name = "gripper_controller"

        if headless is None:
            from internutopia.core.util import has_display

            self.headless = not has_display()
        else:
            self.headless = headless
        self.robot_cfg = robot_cfg or self._build_default_robot_cfg()

    def _build_default_robot_cfg(self) -> Any:
        return create_franka_robot_cfg()

    def start(self) -> dict[str, Any]:
        if self._env is not None:
            return _to_builtin(self._env.get_observations())

        from internutopia.core.config import Config, SimConfig
        from internutopia.core.gym_env import Env
        from internutopia_extension import import_extensions
        from internutopia_extension.configs.tasks import ManipulationTaskCfg

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
                ManipulationTaskCfg(
                    scene_asset_path=self.scene_asset_path,
                    robots=[self.robot_cfg],
                    prompt="coherent atomic action bridge",
                    target="atomic_action_bridge",
                    episode_idx=0,
                    max_steps=100000,
                ),
            ],
        )
        import_extensions()
        self._env = Env(config)
        obs, _ = self._env.reset()
        return _to_builtin(obs)

    def close(self) -> None:
        if self._env is not None:
            self._env.close()
            self._env = None

    def reset(self) -> dict[str, Any]:
        self._require_env()
        obs, _ = self._env.reset()
        return _to_builtin(obs)

    def register_grasp_target(self, name: str, target: ManipulationTarget | dict[str, Any]) -> None:
        self._grasp_targets[name] = self._coerce_manipulation_target(target, default_name=name)

    def register_place_target(self, name: str, target: ManipulationTarget | dict[str, Any]) -> None:
        self._place_targets[name] = self._coerce_manipulation_target(target, default_name=name)

    def grasp(
        self,
        target: str | ManipulationTarget | dict[str, Any],
        dump_path: str | Path | None = None,
    ) -> ActionResult:
        self._require_env()
        grasp_target = self._resolve_manipulation_target(target, self._grasp_targets)
        result = self._run_pick_or_place(
            action_name="grasp",
            target=grasp_target,
            gripper_command="close",
            dump_path=dump_path,
        )
        return result

    def pick(
        self,
        target: str | ManipulationTarget | dict[str, Any],
        dump_path: str | Path | None = None,
    ) -> ActionResult:
        return self.grasp(target=target, dump_path=dump_path)

    def place(
        self,
        target: str | ManipulationTarget | dict[str, Any],
        dump_path: str | Path | None = None,
    ) -> ActionResult:
        self._require_env()
        place_target = self._resolve_manipulation_target(target, self._place_targets)
        result = self._run_pick_or_place(
            action_name="place",
            target=place_target,
            gripper_command="open",
            dump_path=dump_path,
        )
        return result

    def release(
        self,
        target: str | ManipulationTarget | dict[str, Any],
        dump_path: str | Path | None = None,
    ) -> ActionResult:
        return self.place(target=target, dump_path=dump_path)

    def _run_pick_or_place(
        self,
        action_name: str,
        target: ManipulationTarget,
        gripper_command: str,
        dump_path: str | Path | None,
    ) -> ActionResult:
        trace: list[dict[str, Any]] = []
        final_obs: dict[str, Any] = {}

        approach = target.pre_position or (
            target.position[0],
            target.position[1],
            target.position[2] + 0.1,
        )
        retreat = target.post_position or approach

        # For place/release, enforce an explicit "open -> wait -> lift" order.
        # This prevents lifting while fingers are still closing/opening.
        if gripper_command == "open":
            release_pause_steps = max(self.pause_steps, self.gripper_settle_steps)
            phases = [
                ("approach", ("motion", approach, target.orientation)),
                ("pre_gripper_pause", ("pause", None, None)),
                ("target", ("motion", target.position, target.orientation)),
                ("target_pause", ("pause", None, None)),
                ("gripper", ("settle", gripper_command, None)),
                ("release_settle", ("pause_custom", release_pause_steps, None)),
            ]
        else:
            phases = [
                ("approach", ("motion", approach, target.orientation)),
                ("pre_gripper_pause", ("pause", None, None)),
                ("target", ("motion", target.position, target.orientation)),
                ("target_pause", ("pause", None, None)),
                ("gripper", ("settle", gripper_command, None)),
                ("post_gripper_pause", ("pause", None, None)),
                ("retreat", ("motion", retreat, target.orientation)),
            ]

        success = True
        phase_steps: dict[str, int] = {}

        for phase_name, phase_payload in phases:
            phase_type, phase_value, phase_orientation = phase_payload
            if phase_type == "motion":
                phase_success, phase_trace, phase_final = self._run_cartesian_motion(
                    destination=phase_value,
                    orientation=phase_orientation,
                    phase=phase_name,
                    target=target,
                )
            elif phase_type == "settle":
                phase_success, phase_trace, phase_final = self._run_fixed_steps(
                    action=self._gripper_action(phase_value),
                    steps=self.gripper_settle_steps,
                    phase=phase_name,
                    target=target,
                )
            elif phase_type == "pause_custom":
                phase_success, phase_trace, phase_final = self._run_fixed_steps(
                    action={},
                    steps=max(1, int(phase_value or 1)),
                    phase=phase_name,
                    target=target,
                )
            else:
                phase_success, phase_trace, phase_final = self._run_fixed_steps(
                    action={},
                    steps=self.pause_steps,
                    phase=phase_name,
                    target=target,
                )
            trace.extend(phase_trace)
            final_obs = phase_final
            phase_steps[phase_name] = len(phase_trace)
            if not phase_success:
                success = False
                break

        result = ActionResult(
            action=action_name,
            success=success,
            steps=len(trace),
            final_observation=final_obs,
            trace=trace,
            metadata={
                "target_name": target.name,
                "target_position": target.position,
                "target_orientation": target.orientation,
                "phase_steps": phase_steps,
            },
        )
        if dump_path is not None:
            result.dump_json(dump_path)
        return result

    def _run_until_controller_finished(
        self,
        controller_name: str,
        action: dict[str, Any],
        phase: str,
        target: ManipulationTarget,
    ) -> tuple[bool, list[dict[str, Any]], dict[str, Any]]:
        trace: list[dict[str, Any]] = []
        final_obs: dict[str, Any] = {}

        for step in range(1, self.max_steps_per_phase + 1):
            obs, _, terminated, _, _ = self._env.step(action=action)
            final_obs = _to_builtin(obs)
            trace.append(
                {
                    "step": step,
                    "phase": phase,
                    "target": target.name or target.position,
                    "observation": final_obs,
                }
            )
            controller_obs = obs.get("controllers", {}).get(controller_name, {})
            if controller_obs.get("finished", False):
                return True, trace, final_obs
            if terminated:
                return False, trace, final_obs

        return False, trace, final_obs

    def _run_cartesian_motion(
        self,
        destination: Sequence[float],
        orientation: Sequence[float],
        phase: str,
        target: ManipulationTarget,
    ) -> tuple[bool, list[dict[str, Any]], dict[str, Any]]:
        current_obs = self._env.get_observations()
        if not current_obs or "eef_position" not in current_obs:
            waypoint_positions = [np.array(_normalize_xyz(destination), dtype=float)]
        else:
            start = np.array(current_obs["eef_position"], dtype=float)
            end = np.array(_normalize_xyz(destination), dtype=float)
            waypoint_count = max(1, self.arm_waypoint_count)
            waypoint_positions = [
                start + (end - start) * (idx / waypoint_count)
                for idx in range(1, waypoint_count + 1)
            ]

        total_trace: list[dict[str, Any]] = []
        final_obs: dict[str, Any] = {}
        for waypoint_idx, waypoint in enumerate(waypoint_positions, start=1):
            phase_success, phase_trace, phase_final = self._run_until_controller_finished(
                controller_name=self._arm_controller_name,
                action=self._arm_action(waypoint, orientation),
                phase=f"{phase}_{waypoint_idx}",
                target=target,
            )
            total_trace.extend(phase_trace)
            final_obs = phase_final
            if not phase_success:
                return False, total_trace, final_obs
        return True, total_trace, final_obs

    def _run_fixed_steps(
        self,
        action: dict[str, Any],
        steps: int,
        phase: str,
        target: ManipulationTarget,
    ) -> tuple[bool, list[dict[str, Any]], dict[str, Any]]:
        trace: list[dict[str, Any]] = []
        final_obs: dict[str, Any] = {}

        for step in range(1, steps + 1):
            obs, _, terminated, _, _ = self._env.step(action=action)
            final_obs = _to_builtin(obs)
            trace.append(
                {
                    "step": step,
                    "phase": phase,
                    "target": target.name or target.position,
                    "observation": final_obs,
                }
            )
            if terminated:
                return False, trace, final_obs

        return True, trace, final_obs

    def _arm_action(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
    ) -> dict[str, list[np.ndarray]]:
        return {
            self._arm_controller_name: [
                np.array(_normalize_xyz(position)),
                np.array(_normalize_quat(orientation)),
            ]
        }

    def _gripper_action(self, command: str) -> dict[str, list[str]]:
        return {self._gripper_controller_name: [command]}

    def _coerce_manipulation_target(
        self,
        target: ManipulationTarget | dict[str, Any],
        default_name: str | None = None,
    ) -> ManipulationTarget:
        if isinstance(target, ManipulationTarget):
            return target
        if not isinstance(target, dict):
            raise TypeError(f"Unsupported manipulation target type: {type(target)}")

        return ManipulationTarget(
            position=_normalize_xyz(target["position"]),
            orientation=_normalize_quat(target.get("orientation", (1.0, 0.0, 0.0, 0.0))),
            pre_position=None if target.get("pre_position") is None else _normalize_xyz(target["pre_position"]),
            post_position=None if target.get("post_position") is None else _normalize_xyz(target["post_position"]),
            name=target.get("name", default_name),
            metadata=dict(target.get("metadata", {})),
        )

    def _resolve_manipulation_target(
        self,
        target: str | ManipulationTarget | dict[str, Any],
        registry: dict[str, ManipulationTarget],
    ) -> ManipulationTarget:
        if isinstance(target, str):
            if target not in registry:
                raise KeyError(f"Unknown manipulation target: {target}")
            return registry[target]
        return self._coerce_manipulation_target(target)

    def _require_env(self) -> None:
        if self._env is None:
            self.start()


class PiperGo2ManipulationAPI:
    ARM_JOINT_NAMES = [
        'piper_j1',
        'piper_j2',
        'piper_j3',
        'piper_j4',
        'piper_j5',
        'piper_j6',
        'piper_j7',
        'piper_j8',
    ]

    def __init__(
        self,
        scene_asset_path: str,
        robot_cfg: Any | None = None,
        extra_robot_cfgs: Sequence[Any] | None = None,
        objects: Sequence[Any] | None = None,
        headless: bool | None = None,
        max_steps_per_action: int = 1800,
        arm_settle_steps: int = 90,
        arm_motion_steps: int = 36,
        pause_steps: int = 60,
        navigation_offset: float = 0.42,
        enable_arm_ik: bool = True,
        ik_urdf_path: str | None = None,
        allow_arm_heuristic_fallback: bool = True,
    ) -> None:
        self.scene_asset_path = scene_asset_path
        self.max_steps_per_action = max_steps_per_action
        self.arm_settle_steps = arm_settle_steps
        self.arm_motion_steps = arm_motion_steps
        self.pause_steps = pause_steps
        self.navigation_offset = navigation_offset
        self.enable_arm_ik = enable_arm_ik
        self.ik_urdf_path = ik_urdf_path
        self.allow_arm_heuristic_fallback = allow_arm_heuristic_fallback
        self.objects = list(objects) if objects is not None else []
        self.extra_robot_cfgs = list(extra_robot_cfgs) if extra_robot_cfgs is not None else []
        self._env = None
        self._robot_view = None
        self._joint_indices = None
        self._ik_solver = None
        self._ik_failed = False
        self._last_arm_plan_mode = 'unknown'
        self._grasp_targets: dict[str, ManipulationTarget] = {}
        self._place_targets: dict[str, ManipulationTarget] = {}
        self._move_controller_name = 'move_to_point'
        self._arm_controller_name = 'arm_joint_controller'
        self._active_grasp_joint_path: str | None = None
        self._active_grasp_object_path: str | None = None

        if headless is None:
            from internutopia.core.util import has_display

            self.headless = not has_display()
        else:
            self.headless = headless
        self.robot_cfg = robot_cfg or self._build_default_robot_cfg()

    def _build_default_robot_cfg(self) -> Any:
        return create_pipergo2_robot_cfg()

    def start(self) -> dict[str, Any]:
        if self._env is not None:
            return _to_builtin(self._env.get_observations())

        from internutopia.core.config import Config, SimConfig
        from internutopia.core.gym_env import Env
        from internutopia_extension import import_extensions
        from internutopia_extension.configs.tasks import SingleInferenceTaskCfg

        config = Config(
            simulator=SimConfig(
                physics_dt=1 / 240,
                rendering_dt=1 / 240,
                use_fabric=False,
                headless=self.headless,
                webrtc=self.headless,
            ),
            metrics_save_path='none',
            task_configs=[
                SingleInferenceTaskCfg(
                    scene_asset_path=self.scene_asset_path,
                    # Old single-robot behavior for rollback:
                    # robots=[self.robot_cfg],
                    robots=[self.robot_cfg, *self.extra_robot_cfgs],
                    objects=self.objects,
                ),
            ],
        )
        import_extensions()
        self._env = Env(config)
        obs, _ = self._env.reset()

        try:
            from isaacsim.core.prims import Articulation as ArticulationView  # type: ignore
        except ImportError:  # pragma: no cover - Isaac 4.x / deprecated shim
            from omni.isaac.core.articulations import ArticulationView  # type: ignore

        self._robot_view = ArticulationView(prim_paths_expr='/World/env_0/robots/pipergo2', name='pipergo2_view')
        self._robot_view.initialize()
        self._ensure_robot_articulation_physics()
        self._joint_indices = {name: idx for idx, name in enumerate(self._robot_view.dof_names)}
        return _to_builtin(obs)

    def close(self) -> None:
        if self._env is not None:
            self._env.close()
            self._env = None
            self._robot_view = None
            self._joint_indices = None

    def reset(self) -> dict[str, Any]:
        self._require_env()
        obs, _ = self._env.reset()
        self._active_grasp_joint_path = None
        self._active_grasp_object_path = None
        self._ensure_robot_articulation_physics()
        if self._robot_view is not None and self._robot_view.dof_names:
            self._joint_indices = {name: idx for idx, name in enumerate(self._robot_view.dof_names)}
        return _to_builtin(obs)

    def register_grasp_target(self, name: str, target: ManipulationTarget | dict[str, Any]) -> None:
        self._grasp_targets[name] = self._coerce_manipulation_target(target, default_name=name)

    def register_place_target(self, name: str, target: ManipulationTarget | dict[str, Any]) -> None:
        self._place_targets[name] = self._coerce_manipulation_target(target, default_name=name)

    def _get_robot_root_path(self) -> str:
        return "/World/env_0/robots/pipergo2"


    def _find_robot_root_body(self) -> str:
        """
        ⚠️ 关键：直接锁 articulation root（整只狗）
        """
        # � 这是 Isaac Sim / InternUtopia 标准 root
        return "/World/env_0/robots/pipergo2"

    def _ensure_robot_articulation_physics(self) -> None:
        """Re-bind PhysX articulation view after timeline STOP / prim invalidation (Isaac Sim 5.x)."""
        rv = self._robot_view
        if rv is None:
            return
        if not hasattr(rv, "_physics_view"):
            rv._physics_view = None  # type: ignore[attr-defined]
        try:
            from isaacsim.core.simulation_manager import SimulationManager  # type: ignore
        except Exception:
            return
        if SimulationManager.get_physics_sim_view() is None:
            return
        try:
            if rv.is_physics_handle_valid():
                return
        except Exception:
            pass
        try:
            rv.initialize()
        except Exception:
            pass

    def _freeze_robot_pose(self):
        if self._robot_view is None:
            return
        self._ensure_robot_articulation_physics()

        try:
            pos, rot = self._robot_view.get_world_poses()
            self._frozen_pose = (pos.copy(), rot.copy())

            print(f"[pipergo2] FREEZE pose: {pos[0]}")

        except Exception as e:
            print(f"[freeze_pose] failed: {e}")

    def _apply_frozen_pose(self):
        if not hasattr(self, "_frozen_pose"):
            return

        try:
            import numpy as np

            self._ensure_robot_articulation_physics()
            pos, rot = self._frozen_pose

            # � 锁 base 位姿（不会跳）
            self._robot_view.set_world_poses(pos, rot)

            # � 清速度（防止被 arm 推倒）
            self._robot_view.set_linear_velocities(np.zeros((1, 3)))
            self._robot_view.set_angular_velocities(np.zeros((1, 3)))

        except Exception as e:
            print(f"[apply_frozen_pose] failed: {e}")

    def _clear_frozen_pose(self):
        if hasattr(self, "_frozen_pose"):
            del self._frozen_pose
            print("[pipergo2] UNFREEZE robot")




    def pick(
        self,
        target: str | ManipulationTarget | dict[str, Any],
        dump_path: str | Path | None = None,
    ) -> ActionResult:
        self._require_env()
        grasp_target = self._resolve_manipulation_target(target, self._grasp_targets)
        result = self._run_pick_or_place(
            action_name='pick',
            target=grasp_target,
            close_gripper=True,
            dump_path=dump_path,
        )
        return result

    def grasp(
        self,
        target: str | ManipulationTarget | dict[str, Any],
        dump_path: str | Path | None = None,
    ) -> ActionResult:
        return self.pick(target=target, dump_path=dump_path)

    def release(
        self,
        target: str | ManipulationTarget | dict[str, Any],
        dump_path: str | Path | None = None,
    ) -> ActionResult:
        self._require_env()
        place_target = self._resolve_manipulation_target(target, self._place_targets)
        result = self._run_pick_or_place(
            action_name='release',
            target=place_target,
            close_gripper=False,
            dump_path=dump_path,
        )
        return result

    def place(
        self,
        target: str | ManipulationTarget | dict[str, Any],
        dump_path: str | Path | None = None,
    ) -> ActionResult:
        return self.release(target=target, dump_path=dump_path)
    

    def _run_pick_or_place(
        self,
        action_name: str,
        target: ManipulationTarget,
        close_gripper: bool,
        dump_path: str | Path | None,
    ) -> ActionResult:

        trace = []
        final_obs = {}
        success = True

        stance_position = target.metadata.get('base_position')
        if stance_position is None:
            stance_position = self._compute_navigation_target(target.position)

        # Old behavior: detach immediately when entering release().
        # This made the robot effectively "empty-handed" during the placement motion
        # and often caused the object to drop before reaching the target pose.
        #
        # if not close_gripper:
        #     self._detach_grasped_object()

        try:
            # =========================
            # 1. navigation（狗可以动）
            # =========================
            success, t, final_obs = self._run_navigation(
                target_position=stance_position,
                phase='navigate',
                target=target,
            )
            trace.extend(t)
            if not success:
                raise RuntimeError("navigation failed")

            # =========================
            # � 关键：锁整只狗
            # =========================
            self._freeze_robot_pose()

            # =========================
            # 2. pause
            # =========================
            success, t, final_obs = self._run_fixed_steps({}, self.pause_steps, 'pause', target)
            trace.extend(t)
            if not success:
                raise RuntimeError("pause failed")

            # =========================
            # 3. IK
            # =========================
            grasp_ori = (0.707, 0.0, 0.707, 0.0)

            pre_pose = self._plan_arm_pose(
                target.pre_position or target.position,
                gripper_open=True,
                orientation=grasp_ori,
            )

            target_pose = self._plan_arm_pose(
                target.position,
                gripper_open=True,
                orientation=grasp_ori,
            )

            post_pose = self._plan_arm_pose(
                target.post_position or target.position,
                gripper_open=not close_gripper,
                orientation=grasp_ori,
            )

            # =========================
            # 4. approach
            # =========================
            success, t, final_obs = self._run_arm_motion(pre_pose, 'approach', target)
            trace.extend(t)
            if not success:
                raise RuntimeError("approach failed")

            # =========================
            # 5. target
            # =========================
            success, t, final_obs = self._run_arm_motion(target_pose, 'target', target)
            trace.extend(t)
            if not success:
                raise RuntimeError("target motion failed")

            # =========================
            # 6. gripper
            # =========================
            success, t, final_obs = self._run_gripper(close=close_gripper, target=target)
            trace.extend(t)
            if not success:
                raise RuntimeError("gripper motion failed")

            if close_gripper:
                print('[pipergo2][pick] entering attach step')
                self._attach_object_if_requested(target)
                print('[pipergo2][pick] attach step finished')

                # =========================
                # 7. retreat (pick)
                # =========================
                print('[pipergo2][pick] entering retreat')
                success, t, final_obs = self._run_arm_motion(post_pose, 'retreat', target)
                trace.extend(t)
                if not success:
                    raise RuntimeError("retreat failed")
                print('[pipergo2][pick] retreat finished')
            else:
                # New release behavior:
                # 1) keep the assisted grasp attached while moving to the place target
                # 2) open gripper at the place pose
                # 3) settle briefly
                # 4) detach the assisted grasp
                # 5) pause so the object can fall / settle onto the support
                # 6) retreat with an open gripper
                success, t, final_obs = self._run_fixed_steps({}, self.pause_steps, 'release_settle', target)
                trace.extend(t)
                if not success:
                    raise RuntimeError("release settle failed")

                self._detach_grasped_object()

                success, t, final_obs = self._run_fixed_steps({}, self.pause_steps, 'post_detach_pause', target)
                trace.extend(t)
                if not success:
                    raise RuntimeError("post-detach pause failed")

                # =========================
                # 7. retreat (release)
                # =========================
                success, t, final_obs = self._run_arm_motion(post_pose, 'retreat', target)
                trace.extend(t)
                if not success:
                    raise RuntimeError("retreat failed")

        except RuntimeError as e:
            print(f"[pipergo2] aborted: {e}")
            success = False

        finally:
            # � 解锁
            self._clear_frozen_pose()

        result = ActionResult(
            action=action_name,
            success=success,
            steps=len(trace),
            final_observation=final_obs,
            trace=trace,
        )

        if dump_path:
            result.dump_json(dump_path)

        return result

    def _run_navigation(
        self,
        target_position: Sequence[float],
        phase: str,
        target: ManipulationTarget,
    ) -> tuple[bool, list[dict[str, Any]], dict[str, Any]]:
        trace: list[dict[str, Any]] = []
        final_obs: dict[str, Any] = {}
        stable = 0
        command = {self._move_controller_name: [np.array(_normalize_xyz(target_position), dtype=float)]}

        for step in range(1, self.max_steps_per_action + 1):
            obs, _, terminated, _, _ = self._env.step(action=command)
            final_obs = _to_builtin(obs)
            trace.append(
                {
                    'step': step,
                    'phase': phase,
                    'target': target.name or target.position,
                    'observation': final_obs,
                }
            )
            finished = obs.get('controllers', {}).get(self._move_controller_name, {}).get('finished', False)
            stable = stable + 1 if finished else 0
            if stable >= 20:
                self._print_navigation_debug(
                    phase=phase,
                    target_position=target_position,
                    final_obs=final_obs,
                    target=target,
                )
                return True, trace, final_obs
            if terminated:
                self._print_navigation_debug(
                    phase=phase,
                    target_position=target_position,
                    final_obs=final_obs,
                    target=target,
                )
                return False, trace, final_obs

        self._print_navigation_debug(
            phase=phase,
            target_position=target_position,
            final_obs=final_obs,
            target=target,
        )
        return False, trace, final_obs

    def _run_arm_motion(
        self,
        target_pose: np.ndarray,
        phase: str,
        target: ManipulationTarget,
    ):
        trace = []
        final_obs = {}

        current = self._get_current_arm_pose()

        for waypoint in self._interpolate_joint_waypoints(
            current, target_pose, count=self.arm_motion_steps
        ):
            obs, _, terminated, _, _ = self._env.step(
                action={self._arm_controller_name: [waypoint]}
            )
            self._apply_frozen_pose()
            final_obs = _to_builtin(obs)
            self._update_eef_debug_marker(final_obs)

            trace.append({
                "phase": phase,
                "observation": final_obs
            })

            if terminated:
                return False, trace, final_obs

        # settle（只做一次）
        success, t, final_obs = self._run_fixed_steps(
            {self._arm_controller_name: [target_pose]},
            self.arm_settle_steps,
            phase,
            target,
        )
        trace.extend(t)

        self._print_reach_debug(phase, target, final_obs)

        return True, trace, final_obs
    
    def _run_gripper(self, close: bool, target: ManipulationTarget):
        trace = []
        final_obs = {}

        # Old behavior: jump directly from the current finger width to the final
        # width in a single command, then hold for arm_settle_steps.
        # This abrupt contact often destabilizes the grasp and can terminate the
        # episode right when the fingers touch the object / pedestal.
        #
        # pose = self._get_current_arm_pose()
        # pose[6] = 0.010 if close else 0.032
        # pose[7] = -0.010 if close else -0.032
        #
        # success, t, final_obs = self._run_fixed_steps(
        #     {self._arm_controller_name: [pose]},
        #     self.arm_settle_steps,
        #     'gripper',
        #     target,
        # )
        #
        # trace.extend(t)
        # return success, trace, final_obs

        start_pose = self._get_current_arm_pose()
        target_pose = start_pose.copy()
        target_pose[6] = 0.010 if close else 0.032
        target_pose[7] = -0.010 if close else -0.032

        # Move only the two finger joints gradually so contact is introduced
        # smoothly instead of as a single large positional jump.
        finger_motion_steps = max(12, min(36, self.arm_motion_steps // 4))
        finger_waypoints = self._interpolate_joint_waypoints(start_pose, target_pose, count=finger_motion_steps)

        for step_index, waypoint in enumerate(finger_waypoints, start=1):
            obs, _, terminated, _, _ = self._env.step(action={self._arm_controller_name: [waypoint]})
            self._apply_frozen_pose()
            final_obs = _to_builtin(obs)
            self._update_eef_debug_marker(final_obs)
            trace.append(
                {
                    'step': step_index,
                    'phase': 'gripper_ramp',
                    'target': target.name or target.position,
                    'observation': final_obs,
                }
            )
            if terminated:
                print(
                    '[pipergo2][gripper] terminated during ramp: '
                    f'step={step_index}/{finger_motion_steps} '
                    f'close={close} '
                    f'j7={float(waypoint[6]):.4f} '
                    f'j8={float(waypoint[7]):.4f} '
                    f'eef={tuple(np.round(final_obs.get("eef_position", []), 4))}'
                )
                return False, trace, final_obs

        success, t, final_obs = self._run_fixed_steps(
            {self._arm_controller_name: [target_pose]},
            self.arm_settle_steps,
            'gripper',
            target,
        )
        trace.extend(t)
        joint_positions = final_obs.get('joint_positions', [])
        j7_index = self._joint_indices.get('piper_j7')
        j8_index = self._joint_indices.get('piper_j8')
        logged_j7 = (
            joint_positions[j7_index]
            if j7_index is not None and len(joint_positions) > j7_index
            else 'na'
        )
        logged_j8 = (
            joint_positions[j8_index]
            if j8_index is not None and len(joint_positions) > j8_index
            else 'na'
        )
        if not success:
            print(
                '[pipergo2][gripper] hold failed: '
                f'close={close} '
                f'j7={logged_j7} '
                f'j8={logged_j8} '
                f'eef={tuple(np.round(final_obs.get("eef_position", []), 4))}'
            )
        else:
            print(
                '[pipergo2][gripper] complete: '
                f'close={close} '
                f'j7={logged_j7} '
                f'j8={logged_j8} '
                f'eef={tuple(np.round(final_obs.get("eef_position", []), 4))}'
            )
        return success, trace, final_obs

    def _run_fixed_steps(
        self,
        action: dict[str, Any],
        steps: int,
        phase: str,
        target: ManipulationTarget,
    ) -> tuple[bool, list[dict[str, Any]], dict[str, Any]]:
        trace: list[dict[str, Any]] = []
        final_obs: dict[str, Any] = {}
        for step in range(1, steps + 1):
            obs, _, terminated, _, _ = self._env.step(action=action)
            self._apply_frozen_pose()
            final_obs = _to_builtin(obs)
            self._update_eef_debug_marker(final_obs)
            trace.append(
                {
                    'step': step,
                    'phase': phase,
                    'target': target.name or target.position,
                    'observation': final_obs,
                }
            )
            if terminated:
                return False, trace, final_obs
        return True, trace, final_obs

    def _get_current_arm_pose(self) -> np.ndarray:
        self._ensure_robot_articulation_physics()
        current = self._robot_view.get_joint_positions()[0]
        return np.array([current[self._joint_indices[name]] for name in self.ARM_JOINT_NAMES], dtype=float)

    def _get_rest_arm_pose(self, gripper_open: bool) -> np.ndarray:
        pose = self._get_current_arm_pose()
        pose[:6] = np.array([0.0, 0.7, -1.4, 0.0, 0.75, 0.0], dtype=float)
        pose[6] = 0.032 if gripper_open else 0.010
        pose[7] = -0.032 if gripper_open else -0.010
        return pose

    def _plan_arm_pose(
        self,
        world_position: Sequence[float],
        gripper_open: bool,
        orientation: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    ) -> np.ndarray:
        ik_pose = self._plan_arm_pose_with_ik(
            world_position=world_position,
            orientation=orientation,
            gripper_open=gripper_open,
        )
        if ik_pose is not None:
            self._last_arm_plan_mode = 'ik'
            return ik_pose

        if self.enable_arm_ik and not self.allow_arm_heuristic_fallback:
            raise RuntimeError(
                'IK-only mode is enabled and no valid IK solution was found for '
                f'target position={tuple(float(v) for v in world_position)} '
                f'orientation={tuple(float(v) for v in orientation)}'
            )

        self._last_arm_plan_mode = 'heuristic'

        target = np.array(_normalize_xyz(world_position), dtype=float)
        obs = self._env.get_observations()
        base_position = np.array(obs['position'], dtype=float)
        base_orientation = np.array(obs['orientation'], dtype=float)

        yaw = self._quat_to_yaw(base_orientation)
        rel_world = target - base_position
        cos_yaw = np.cos(-yaw)
        sin_yaw = np.sin(-yaw)
        rel_x = cos_yaw * rel_world[0] - sin_yaw * rel_world[1]
        rel_y = sin_yaw * rel_world[0] + cos_yaw * rel_world[1]
        rel_z = rel_world[2] - 0.25

        radial = np.clip(np.hypot(rel_x, rel_y), 0.18, 0.60)
        height = np.clip(rel_z, -0.12, 0.28)

        pose = self._get_current_arm_pose()
        pose[0] = np.clip(np.arctan2(rel_y, rel_x), -1.4, 1.4)
        pose[1] = np.clip(1.55 - 1.8 * (radial - 0.22) - 1.0 * height, 0.35, 1.70)
        pose[2] = np.clip(-2.55 + 2.2 * (radial - 0.22) + 0.8 * height, -2.75, -1.10)
        pose[3] = 0.0
        pose[4] = np.clip(1.10 - 0.45 * (radial - 0.22) - 0.35 * height, -1.10, 1.20)
        pose[5] = np.clip(-pose[0] * 0.35, -1.2, 1.2)
        pose[6] = 0.032 if gripper_open else 0.010
        pose[7] = -0.032 if gripper_open else -0.010
        return pose

    def _plan_arm_pose(
        self,
        world_position: Sequence[float],
        gripper_open: bool,
        orientation: Sequence[float] | None = None,
    ) -> np.ndarray:
        solver = self._get_ik_solver()
        if solver is None:
            return None

        obs = self._env.get_observations()
        arm_base_position = np.array(obs.get('arm_base_position', (0.0, 0.0, 0.0)), dtype=float)
        arm_base_orientation = np.array(obs.get('arm_base_orientation', (1.0, 0.0, 0.0, 0.0)), dtype=float)
        target_world_position = np.array(_normalize_xyz(world_position), dtype=float)

        target_local_position = self._world_to_local_point(
            world_point=target_world_position,
            frame_position=arm_base_position,
            frame_orientation=arm_base_orientation,
        )

        if orientation is None:
            target_local_orientation = None
        else:
            target_world_orientation = np.array(_normalize_quat(orientation), dtype=float)
            target_local_orientation = self._world_to_local_quaternion(
                world_orientation=target_world_orientation,
                frame_orientation=arm_base_orientation,
            )

        current = self._get_current_arm_pose()
        solution = solver.solve(
            position=target_local_position,
            orientation_wxyz=target_local_orientation,
            initial_q=current[:6],
        )
        if solution is None:
            print(
                '[pipergo2][ik] failed: '
                f'world_target={tuple(np.round(target_world_position, 4))} '
                f'local_target={tuple(np.round(target_local_position, 4))} '
                f'orientation={target_local_orientation} '
                'fallback=heuristic'
            )
            return None

        pose = current.copy()
        pose[:6] = solution
        pose[6] = 0.032 if gripper_open else 0.010
        pose[7] = -0.032 if gripper_open else -0.010
        return pose

    def _compute_navigation_target(self, grasp_position: Sequence[float]) -> tuple[float, float, float]:
        obs = self._env.get_observations()
        base_position = np.array(obs['position'], dtype=float)
        target = np.array(_normalize_xyz(grasp_position), dtype=float)
        delta = target[:2] - base_position[:2]
        dist = np.linalg.norm(delta)
        if dist < 1e-6:
            xy = base_position[:2]
        else:
            xy = target[:2] - delta / dist * self.navigation_offset
        return (float(xy[0]), float(xy[1]), 0.0)

    def _interpolate_joint_waypoints(self, start: np.ndarray, end: np.ndarray, count: int) -> list[np.ndarray]:
        waypoints = []
        for idx in range(1, count + 1):
            alpha = idx / count
            smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha)
            waypoints.append(start + (end - start) * smooth_alpha)
        return waypoints

    def _attach_object_if_requested(self, target: ManipulationTarget) -> None:
        object_name = target.metadata.get('object_name')
        if not object_name:
            print('[pipergo2][attach] skipped: missing object_name in target metadata')
            return
        obj = self._env.runner.get_obj(object_name)
        object_path = self._resolve_prim_path(obj)
        joint_path = '/World/pipergo2_grasp_joint'

        # Old behavior: create a fixed joint directly between the gripper base and the
        # object without constraining the joint frame. This preserves whatever relative
        # offset the object happens to have at attach time, which is exactly why the
        # cube can end up "hanging" from a non-grasping spot behind the gripper.
        #
        # create_joint(
        #     prim_path=joint_path,
        #     joint_type='FixedJoint',
        #     body0='/World/env_0/robots/pipergo2/piper_gripper_base',
        #     body1=self._resolve_prim_path(obj),
        #     enabled=True,
        # )

        current_obs = self._env.get_observations()
        eef_position = current_obs.get('eef_position')
        eef_orientation = current_obs.get('eef_orientation')
        if eef_position is None or eef_orientation is None:
            print('[pipergo2][attach] skipped: missing eef pose in observations')
            return

        # Buggy intermediate attempt: this class does not own `self._robot`, so using
        # self._robot.get_end_effector() raises immediately once we enter attach.
        #
        # gripper_base_position, gripper_base_orientation = self._robot.get_end_effector().get_pose()
        # object_position, object_orientation = obj.get_world_pose()

        object_position, object_orientation = obj.get_world_pose()

        eef_position = np.array(eef_position, dtype=float)
        eef_orientation = np.array(eef_orientation, dtype=float)
        object_position = np.array(object_position, dtype=float)
        object_orientation = np.array(object_orientation, dtype=float)

        # Only attach if the object is already close to the intended grasp center.
        # This avoids forcefully snapping a badly placed object onto the back side
        # of the gripper base.
        object_in_eef = self._world_to_local_point(
            world_point=object_position,
            frame_position=eef_position,
            frame_orientation=eef_orientation,
        )
        distance_to_eef = float(np.linalg.norm(object_position - eef_position))
        within_grasp_window = (
            distance_to_eef <= 0.14
            and abs(float(object_in_eef[0])) <= 0.08
            and abs(float(object_in_eef[1])) <= 0.08
            and abs(float(object_in_eef[2])) <= 0.08
        )
        if not within_grasp_window:
            print(
                '[pipergo2][attach] skipped: '
                f'object too far from grasp center '
                f'dist={distance_to_eef:.4f} '
                f'local_offset={tuple(np.round(object_in_eef, 4))}'
            )
            return

        print(
            '[pipergo2][attach] accepted: '
            f'dist={distance_to_eef:.4f} '
            f'local_offset={tuple(np.round(object_in_eef, 4))}'
        )

        # The observed eef pose is defined as the gripper-base pose plus a fixed local
        # offset along the gripper z axis. In PiperGo2Robot that offset is [0, 0, 0.12].
        # So the corresponding joint frame in the parent (gripper_base) frame is simply
        # that fixed local offset, and the local orientation is identity.
        parent_local_pos = np.array([0.0, 0.0, 0.12], dtype=float)
        parent_local_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        # Old behavior in the intermediate fix: bind the child joint frame to the
        # object's origin (0, 0, 0). If the object center is not already exactly at
        # the desired grasp center, PhysX reports "disjointed body transforms" and
        # snaps the bodies together, which can violently kick nearby objects away.
        #
        # child_local_pos = np.zeros(3, dtype=float)

        child_local_pos = self._world_to_local_point(
            world_point=eef_position,
            frame_position=object_position,
            frame_orientation=object_orientation,
        )
        child_local_quat = self._world_to_local_quaternion(
            world_orientation=eef_orientation,
            frame_orientation=object_orientation,
        )

        create_joint(
            prim_path=joint_path,
            joint_type='FixedJoint',
            body0='/World/env_0/robots/pipergo2/piper_gripper_base',
            body1=object_path,
            enabled=True,
            joint_frame_in_parent_frame_pos=parent_local_pos,
            joint_frame_in_parent_frame_quat=parent_local_quat,
            joint_frame_in_child_frame_pos=child_local_pos,
            joint_frame_in_child_frame_quat=child_local_quat,
        )
        self._set_object_collision_enabled(object_path=object_path, enabled=False)
        print(
            '[pipergo2][attach] created fixed joint: '
            f'parent_local_pos={tuple(np.round(parent_local_pos, 4))} '
            f'child_local_pos={tuple(np.round(child_local_pos, 4))}'
        )
        self._active_grasp_joint_path = joint_path
        self._active_grasp_object_path = object_path

    def _detach_grasped_object(self) -> None:
        if self._active_grasp_object_path is not None:
            self._set_object_collision_enabled(object_path=self._active_grasp_object_path, enabled=True)
        if self._active_grasp_joint_path is None:
            return
        import omni

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self._active_grasp_joint_path)
        if prim and prim.IsValid():
            attr = prim.GetAttribute('physics:jointEnabled')
            if attr.IsValid():
                attr.Set(False)
        self._active_grasp_joint_path = None
        self._active_grasp_object_path = None

    def _set_object_collision_enabled(self, object_path: str, enabled: bool) -> None:
        try:
            import omni
            from internutopia.core.util.physics import activate_collider, deactivate_collider

            prim = omni.isaac.core.utils.prims.get_prim_at_path(object_path)
            if prim is None or not prim.IsValid():
                print(f'[pipergo2][collision] skipped: invalid prim path {object_path}')
                return

            # Old behavior: keep the grasped cube collidable during carry.
            # That makes it act like a rigid hammer while attached, so it can
            # sweep into nearby supports and launch them away.
            if enabled:
                activate_collider(prim)
            else:
                deactivate_collider(prim)
            print(f'[pipergo2][collision] object={object_path} enabled={enabled}')
        except Exception as exc:
            print(f'[pipergo2][collision] toggle failed for {object_path}: {exc}')

    @staticmethod
    def _resolve_prim_path(rigid_body) -> str:
        raw = rigid_body.unwrap()
        for attr in ('prim_path', '_prim_path'):
            value = getattr(raw, attr, None)
            if value is not None:
                return value
        return rigid_body._param['prim_path']

    @staticmethod
    def _quat_to_yaw(quat_wxyz: Sequence[float]) -> float:
        w, x, y, z = [float(v) for v in quat_wxyz]
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return float(np.arctan2(siny_cosp, cosy_cosp))

    def _get_ik_solver(self) -> PiperIKSolver | None:
        if not self.enable_arm_ik or self._ik_failed:
            return None
        if self._ik_solver is not None:
            return self._ik_solver
        try:
            self._ik_solver = PiperIKSolver(urdf_path=self.ik_urdf_path)
        except Exception as exc:
            self._ik_failed = True
            self._ik_solver = None
            print(f'[pipergo2][ik] unavailable: {exc}')
        return self._ik_solver

    @staticmethod
    def _quat_to_rotmat(quat_wxyz: Sequence[float]) -> np.ndarray:
        w, x, y, z = [float(v) for v in quat_wxyz]
        norm = np.sqrt(w * w + x * x + y * y + z * z)
        if norm < 1e-8:
            return np.eye(3, dtype=float)
        w /= norm
        x /= norm
        y /= norm
        z /= norm
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=float,
        )

    @staticmethod
    def _quat_multiply(lhs_wxyz: Sequence[float], rhs_wxyz: Sequence[float]) -> np.ndarray:
        w1, x1, y1, z1 = [float(v) for v in lhs_wxyz]
        w2, x2, y2, z2 = [float(v) for v in rhs_wxyz]
        return np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ],
            dtype=float,
        )

    @staticmethod
    def _quat_conjugate(quat_wxyz: Sequence[float]) -> np.ndarray:
        w, x, y, z = [float(v) for v in quat_wxyz]
        return np.array([w, -x, -y, -z], dtype=float)

    def _world_to_local_point(
        self,
        world_point: Sequence[float],
        frame_position: Sequence[float],
        frame_orientation: Sequence[float],
    ) -> np.ndarray:
        rotation = self._quat_to_rotmat(frame_orientation)
        return rotation.T @ (np.array(world_point, dtype=float) - np.array(frame_position, dtype=float))

    def _world_to_local_quaternion(
        self,
        world_orientation: Sequence[float],
        frame_orientation: Sequence[float],
    ) -> np.ndarray:
        local = self._quat_multiply(self._quat_conjugate(frame_orientation), world_orientation)
        norm = np.linalg.norm(local)
        if norm < 1e-8:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        return local / norm

    def _print_reach_debug(
        self,
        phase: str,
        target: ManipulationTarget,
        final_obs: dict[str, Any],
    ) -> None:
        if phase not in {'approach', 'target', 'gripper', 'retreat'}:
            return

        if phase == 'approach':
            expected = np.array(target.pre_position or target.position, dtype=float)
        elif phase in {'target', 'gripper'}:
            expected = np.array(target.position, dtype=float)
        else:
            expected = np.array(target.post_position or target.position, dtype=float)

        eef_position = final_obs.get('eef_position')
        if eef_position is None:
            return

        actual = np.array(eef_position, dtype=float)
        error = actual - expected
        distance = float(np.linalg.norm(error))
        status = 'ok' if distance <= 0.08 else 'unreached'
        print(
            f'[pipergo2][{phase}] '
            f'mode={self._last_arm_plan_mode} '
            f'status={status} '
            f'target={tuple(np.round(expected, 4))} '
            f'eef={tuple(np.round(actual, 4))} '
            f'error={tuple(np.round(error, 4))} '
            f'dist={distance:.4f}'
        )
        self._print_marker_debug(target=target, final_obs=final_obs, phase=phase)

    def _print_navigation_debug(
        self,
        phase: str,
        target_position: Sequence[float],
        final_obs: dict[str, Any],
        target: ManipulationTarget,
    ) -> None:
        base_position = final_obs.get('position')
        if base_position is None:
            return

        target_xyz = np.array(_normalize_xyz(target_position), dtype=float)
        actual_xyz = np.array(base_position, dtype=float)
        error_xyz = actual_xyz - target_xyz
        xy_error = error_xyz[:2]
        xy_dist = float(np.linalg.norm(xy_error))
        status = 'ok' if xy_dist <= 0.10 else 'unreached'
        print(
            f'[pipergo2][{phase}] '
            f'base_status={status} '
            f'base_target={tuple(np.round(target_xyz, 4))} '
            f'base={tuple(np.round(actual_xyz, 4))} '
            f'xy_error={tuple(np.round(xy_error, 4))} '
            f'xy_dist={xy_dist:.4f}'
        )
        self._print_marker_debug(target=target, final_obs=final_obs, phase=f'{phase}:base')

    def _print_marker_debug(
        self,
        target: ManipulationTarget,
        final_obs: dict[str, Any],
        phase: str,
    ) -> None:
        marker_entries = target.metadata.get('debug_markers', [])
        if not marker_entries:
            return

        base_position = final_obs.get('position')
        if base_position is None:
            return

        base_xy = np.array(base_position[:2], dtype=float)
        for marker in marker_entries:
            marker_name = marker.get('name', 'marker')
            marker_position = marker.get('position')
            if marker_position is None:
                continue
            marker_xy = np.array(_normalize_xyz(marker_position)[:2], dtype=float)
            xy_error = base_xy - marker_xy
            xy_dist = float(np.linalg.norm(xy_error))
            status = 'reached' if xy_dist <= 0.10 else 'not_reached'
            print(
                f'[pipergo2][{phase}][marker] '
                f'name={marker_name} '
                f'status={status} '
                f'marker_xy={tuple(np.round(marker_xy, 4))} '
                f'base_xy={tuple(np.round(base_xy, 4))} '
                f'xy_error={tuple(np.round(xy_error, 4))} '
                f'xy_dist={xy_dist:.4f}'
            )

    def _update_eef_debug_marker(self, final_obs: dict[str, Any]) -> None:
        marker_path = "/World/debug_eef_live_marker"
        eef_position = final_obs.get("eef_position")
        if eef_position is None:
            return
        try:
            import omni
            from pxr import Gf, UsdGeom

            stage = omni.usd.get_context().get_stage()
            sphere = UsdGeom.Sphere.Define(stage, marker_path)
            sphere.GetRadiusAttr().Set(0.025)
            sphere.CreateDisplayColorAttr([Gf.Vec3f(0.05, 1.0, 0.05)])
            UsdGeom.XformCommonAPI(sphere.GetPrim()).SetTranslate(tuple(float(v) for v in eef_position))
        except Exception as exc:
            print(f"[pipergo2][eef_marker] update failed: {exc}")
            return

    def _coerce_manipulation_target(
        self,
        target: ManipulationTarget | dict[str, Any],
        default_name: str | None = None,
    ) -> ManipulationTarget:
        if isinstance(target, ManipulationTarget):
            return target
        if not isinstance(target, dict):
            raise TypeError(f'Unsupported manipulation target type: {type(target)}')

        return ManipulationTarget(
            position=_normalize_xyz(target['position']),
            orientation=_normalize_quat(target.get('orientation', (1.0, 0.0, 0.0, 0.0))),
            pre_position=None if target.get('pre_position') is None else _normalize_xyz(target['pre_position']),
            post_position=None if target.get('post_position') is None else _normalize_xyz(target['post_position']),
            name=target.get('name', default_name),
            metadata=dict(target.get('metadata', {})),
        )

    def _resolve_manipulation_target(
        self,
        target: str | ManipulationTarget | dict[str, Any],
        registry: dict[str, ManipulationTarget],
    ) -> ManipulationTarget:
        if isinstance(target, str):
            if target not in registry:
                raise KeyError(f'Unknown manipulation target: {target}')
            return registry[target]
        return self._coerce_manipulation_target(target)

    def _require_env(self) -> None:
        if self._env is None:
            self.start()
