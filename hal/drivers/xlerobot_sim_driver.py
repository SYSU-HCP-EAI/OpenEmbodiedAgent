"""XLerobot mobile-manipulation simulator driver."""

from __future__ import annotations

import importlib
import threading
from pathlib import Path
from typing import Any

import numpy as np

from hal.drivers.pipergo2_manipulation_driver import PiperGo2ManipulationDriver

_PROFILES_DIR = Path(__file__).resolve().parent.parent / 'profiles'


def _coerce_xyz(values: Any) -> tuple[float, float, float] | None:
    if not isinstance(values, (list, tuple)) or len(values) < 3:
        return None
    return (float(values[0]), float(values[1]), float(values[2]))


def _coerce_quat(values: Any) -> tuple[float, float, float, float] | None:
    if not isinstance(values, (list, tuple)) or len(values) < 4:
        return None
    return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))


def _safe_float_list(values: Any) -> list[float] | None:
    if values is None:
        return None
    if hasattr(values, 'tolist'):
        values = values.tolist()
    if not isinstance(values, (list, tuple)):
        return None
    out: list[float] = []
    for item in values:
        out.append(float(item))
    return out


class _XLerobotSimAPI:
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
        self._robot_usd_path = str(robot_usd_path or '').strip()
        self._objects = list(objects)
        self._headless = bool(headless)
        self._kwargs = dict(kwargs)
        self._robot_start_orientation = _coerce_quat(self._kwargs.pop('robot_start_orientation', None))
        startup_joint_targets = self._kwargs.pop('startup_joint_targets', None)
        self._startup_joint_targets = {
            str(name): float(value)
            for name, value in dict(startup_joint_targets or {}).items()
        }
        self._startup_settle_steps = int(self._kwargs.pop('startup_settle_steps', 24))
        self._startup_upright_enabled = bool(self._kwargs.pop('startup_upright_enabled', True))
        self._startup_upright_min_head_height = float(self._kwargs.pop('startup_upright_min_head_height', 0.9))
        self._startup_upright_retry_settle_steps = int(self._kwargs.pop('startup_upright_retry_settle_steps', 32))
        self._env = None

    def start(self) -> Any:
        from internutopia.core.config import Config, SimConfig
        from internutopia.core.gym_env import Env
        from internutopia_extension import import_extensions
        from internutopia_extension.configs.robots.xlerobot import (
            XLerobotRobotCfg,
            base_joint_controller_cfg,
            head_joint_controller_cfg,
            left_arm_joint_controller_cfg,
            right_arm_joint_controller_cfg,
        )
        from internutopia_extension.configs.tasks import SingleInferenceTaskCfg

        physics_dt = float(self._kwargs.pop('physics_dt', 1 / 120))
        rendering_dt = float(self._kwargs.pop('rendering_dt', physics_dt))
        use_fabric = bool(self._kwargs.pop('use_fabric', False))
        webrtc = bool(self._kwargs.pop('webrtc', self._headless))

        robot_cfg = XLerobotRobotCfg(
            position=self._robot_start,
            orientation=self._robot_start_orientation,
            controllers=[
                base_joint_controller_cfg.copy(deep=True),
                right_arm_joint_controller_cfg.copy(deep=True),
                left_arm_joint_controller_cfg.copy(deep=True),
                head_joint_controller_cfg.copy(deep=True),
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
        self._apply_startup_pose(settle_steps=self._startup_settle_steps)
        return obs

    def _get_robot(self):
        if self._env is None:
            return None
        tasks = getattr(getattr(self._env, 'runner', None), 'current_tasks', {}) or {}
        if not tasks:
            return None
        task = next(iter(tasks.values()))
        robots = getattr(task, 'robots', {}) or {}
        if 'xlerobot' in robots:
            return robots['xlerobot']
        if robots:
            return next(iter(robots.values()))
        return None

    def _get_articulation(self):
        robot = self._get_robot()
        return getattr(robot, 'articulation', None)

    def _settle(self, steps: int = 12) -> None:
        if self._env is None:
            return
        for _ in range(max(1, int(steps))):
            self._env.step(action={})

    def _get_rigid_body(self, link_name: str):
        robot = self._get_robot()
        rb_map = getattr(robot, '_rigid_body_map', {}) or {}
        suffix = f'/{str(link_name).strip()}'
        for path, rigid_body in rb_map.items():
            if str(path).endswith(suffix):
                return rigid_body
        return None

    def _apply_startup_pose(self, settle_steps: int = 24) -> None:
        articulation = self._get_articulation()
        if articulation is None:
            return
        if self._robot_start_orientation is not None:
            articulation.set_world_pose(
                position=np.array(self._robot_start, dtype=np.float32),
                orientation=np.array(self._robot_start_orientation, dtype=np.float32),
            )
        if self._startup_joint_targets:
            joint_positions = np.array(articulation.get_joint_positions(), dtype=np.float32)
            joint_velocities = np.array(articulation.get_joint_velocities(), dtype=np.float32)
            for joint_name, value in self._startup_joint_targets.items():
                joint_index = articulation.get_dof_index(str(joint_name))
                joint_positions[joint_index] = float(value)
                joint_velocities[joint_index] = 0.0
            articulation.set_joint_positions(joint_positions)
            articulation.set_joint_velocities(joint_velocities)
        self._settle(steps=settle_steps)
        if self._startup_upright_enabled:
            self._ensure_startup_upright()

    def _head_height(self) -> float | None:
        rigid_body = self._get_rigid_body('head_tilt_link')
        if rigid_body is None:
            return None
        position, _orientation = rigid_body.get_world_pose()
        xyz = _coerce_xyz(position)
        if xyz is None:
            return None
        return float(xyz[2])

    def _ensure_startup_upright(self) -> None:
        articulation = self._get_articulation()
        if articulation is None:
            return
        head_height = self._head_height()
        if head_height is not None and head_height >= self._startup_upright_min_head_height:
            return

        joint_positions = np.array(articulation.get_joint_positions(), dtype=np.float32)
        joint_velocities = np.zeros_like(np.array(articulation.get_joint_velocities(), dtype=np.float32))
        if self._startup_joint_targets:
            for joint_name, value in self._startup_joint_targets.items():
                joint_index = articulation.get_dof_index(str(joint_name))
                joint_positions[joint_index] = float(value)
        else:
            joint_positions = np.zeros_like(joint_positions)
        articulation.set_joint_positions(joint_positions)
        articulation.set_joint_velocities(joint_velocities)
        self._settle(steps=self._startup_upright_retry_settle_steps)

    def get_robot_world_pose(self) -> dict[str, list[float]]:
        articulation = self._get_articulation()
        if articulation is None:
            raise RuntimeError('robot articulation not initialized')
        position, orientation = articulation.get_world_pose()
        return {
            'position': _safe_float_list(position) or [],
            'orientation': _safe_float_list(orientation) or [],
        }

    def get_link_world_pose(self, link_name: str) -> dict[str, list[float]]:
        rigid_body = self._get_rigid_body(link_name)
        if rigid_body is None:
            raise RuntimeError(f'link not found: {link_name}')
        position, orientation = rigid_body.get_world_pose()
        return {
            'position': _safe_float_list(position) or [],
            'orientation': _safe_float_list(orientation) or [],
        }

    def get_key_link_poses(self, link_names: list[str] | None = None) -> dict[str, dict[str, list[float]]]:
        names = list(link_names or ['base_link', 'head_pan_link', 'head_tilt_link'])
        out: dict[str, dict[str, list[float]]] = {}
        for link_name in names:
            try:
                out[str(link_name)] = self.get_link_world_pose(str(link_name))
            except Exception as exc:
                out[str(link_name)] = {'error': [str(exc)]}
        return out

    def get_joint_positions_map(self, joint_names: list[str] | None = None) -> dict[str, float]:
        articulation = self._get_articulation()
        if articulation is None:
            raise RuntimeError('robot articulation not initialized')
        dof_names = list(getattr(articulation, 'dof_names', []) or [])
        joint_positions = np.array(articulation.get_joint_positions(), dtype=np.float32)
        if joint_names is None:
            names = dof_names
        else:
            wanted = {str(name).strip() for name in joint_names}
            names = [name for name in dof_names if name in wanted]
        return {str(name): float(joint_positions[articulation.get_dof_index(str(name))]) for name in names}

    def set_robot_world_pose(
        self,
        position: list[float] | None = None,
        orientation: list[float] | None = None,
        settle_steps: int = 24,
    ) -> dict[str, Any]:
        articulation = self._get_articulation()
        if articulation is None:
            raise RuntimeError('robot articulation not initialized')
        current_position, current_orientation = articulation.get_world_pose()
        target_position = _coerce_xyz(position) or _coerce_xyz(current_position) or self._robot_start
        target_orientation = _coerce_quat(orientation) or _coerce_quat(current_orientation) or (1.0, 0.0, 0.0, 0.0)
        articulation.set_world_pose(
            position=np.array(target_position, dtype=np.float32),
            orientation=np.array(target_orientation, dtype=np.float32),
        )
        joint_velocities = np.array(articulation.get_joint_velocities(), dtype=np.float32)
        articulation.set_joint_velocities(np.zeros_like(joint_velocities))
        self._settle(steps=settle_steps)
        return {
            'robot': self.get_robot_world_pose(),
            'links': self.get_key_link_poses(),
        }

    def apply_startup_pose(self, settle_steps: int | None = None) -> dict[str, Any]:
        self._apply_startup_pose(settle_steps=self._startup_settle_steps if settle_steps is None else int(settle_steps))
        return {
            'robot': self.get_robot_world_pose(),
            'links': self.get_key_link_poses(),
        }

    def close(self) -> None:
        if self._env is None:
            return
        self._env.close()
        self._env = None


class XLerobotSimDriver(PiperGo2ManipulationDriver):
    _BASE_JOINTS = ('root_x_axis_joint', 'root_y_axis_joint', 'root_z_rotation_joint')
    _RIGHT_ARM_JOINTS = ('Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll', 'Jaw')
    _LEFT_ARM_JOINTS = ('Rotation_2', 'Pitch_2', 'Elbow_2', 'Wrist_Pitch_2', 'Wrist_Roll_2', 'Jaw_2')
    _HEAD_JOINTS = ('head_pan_joint', 'head_tilt_joint')

    def __init__(self, gui: bool = False, *, robot_id: str | None = None, **kwargs: Any) -> None:
        kwargs = dict(kwargs)
        api_kwargs = dict(kwargs.get('api_kwargs', {}) or {})
        self._experimental_navigation_enabled = bool(kwargs.get('experimental_navigation_enabled', False))
        kwargs.setdefault('robot_start', (0.58, 7.50704, 0.8))
        kwargs.setdefault('room_lighting', 'grey_studio')
        kwargs.setdefault('camera_eye_offset', [2.8, 2.8, 2.2])
        kwargs.setdefault('camera_target_z_offset', 0.1)
        kwargs.setdefault('camera_target_min_z', 0.4)
        kwargs.setdefault('robot_camera_resolution', [640, 400])
        kwargs.setdefault('robot_camera_warmup_steps', 8)
        kwargs.setdefault(
            'robot_cameras',
            {
                'body_view': {
                    'parent': '/World/env_0/robots/xlerobot/head_tilt_link',
                    'translation': [0.18, 0.0, 0.08],
                    'orientation': [0.9813, 0.0, 0.192, 0.0],
                    'clipping_range': [0.05, 100.0],
                    'focal_length': 2.4,
                    'purpose': 'robot_body_view',
                }
            },
        )
        kwargs.setdefault('robot_camera_side_view_enabled', True)
        kwargs.setdefault('robot_camera_side_view_name', 'body_view')
        kwargs.setdefault('robot_camera_side_view_title', 'XLerobot Camera')
        kwargs.setdefault('robot_camera_side_view_size', [560, 320])
        kwargs.setdefault('experimental_navigation_enabled', False)
        kwargs.setdefault('startup_upright_enabled', True)
        kwargs.setdefault('startup_upright_min_head_height', 0.9)
        kwargs.setdefault('startup_upright_retry_settle_steps', 32)
        for key in ('robot_start_orientation', 'startup_joint_targets', 'startup_settle_steps'):
            if key in kwargs and key not in api_kwargs:
                api_kwargs[key] = kwargs[key]
        for key in ('startup_upright_enabled', 'startup_upright_min_head_height', 'startup_upright_retry_settle_steps'):
            if key in kwargs and key not in api_kwargs:
                api_kwargs[key] = kwargs[key]
        kwargs['api_kwargs'] = api_kwargs
        super().__init__(gui=gui, robot_id=robot_id or 'xlerobot_001', **kwargs)
        self._env_lock = threading.RLock()

    def get_profile_path(self) -> Path:
        return _PROFILES_DIR / 'xlerobot_sim.md'

    def execute_action(self, action_type: str, params: dict) -> str:
        handlers = {
            'start': self._start_from_action,
            'enter_simulation': self._start_from_action,
            'close': self._close_api,
            'step': self._step_env,
            'api_call': self._api_call,
            'navigate_to_waypoint': self._navigate_to_waypoint,
            'navigate_to_named': self._navigate_to_named,
            'describe_visible_scene': self._describe_visible_scene,
            'set_joint_targets': self._set_joint_targets,
            'set_gripper': self._set_gripper,
            'run_vla_pick_and_return': self._unsupported_action,
            'run_pick_place': self._unsupported_action,
        }
        handler = handlers.get(action_type)
        if handler is None:
            return f'Unknown action: {action_type}'
        try:
            return handler(params or {})
        except Exception as exc:  # pragma: no cover - runtime dependency bridge
            import sys
            import traceback

            print(f'[xlerobot] action {action_type!r} failed:', file=sys.stderr, flush=True)
            traceback.print_exc()
            return f'Error: {type(exc).__name__}: {exc}'

    def get_scene(self) -> dict[str, dict]:
        scene = super().get_scene()
        runtime = dict(scene.pop('manipulation_runtime', {}) or {})
        runtime['state'] = 'running' if self._api is not None else 'idle'
        scene['xlerobot_runtime'] = runtime
        return scene

    def get_runtime_state(self) -> dict[str, Any]:
        runtime = super().get_runtime_state()
        robot_state = runtime.get('robots', {}).get(self.robot_id, {})
        nav_state = robot_state.get('nav_state')
        if isinstance(nav_state, dict):
            nav_state['mode'] = 'mobile_manipulation'
        return runtime

    def _start_from_action(self, params: dict[str, Any]) -> str:
        return super()._start_from_action(params).replace('Manipulation API', 'XLerobot simulation API')

    def _resolve_nav_action_name(self) -> str:
        if self._navigation_action_name:
            return str(self._navigation_action_name)
        self._ensure_pythonpath()
        rob = importlib.import_module('internutopia_extension.configs.robots.xlerobot')
        return rob.base_joint_controller_cfg.name

    def _close_api(self, params: dict[str, Any]) -> str:
        return super()._close_api(params).replace('Manipulation API', 'XLerobot simulation API')

    def _maybe_attach_robot_cameras(self) -> str:
        return super()._maybe_attach_robot_cameras()

    def _maybe_preheat_vla_session(self) -> str:
        return ''

    def _unsupported_action(self, _params: dict[str, Any]) -> str:
        return 'Error: XLerobot sim driver currently supports navigation plus direct joint/gripper control, not pick-place or SmolVLA.'

    def _navigation_disabled_message(self) -> str:
        return (
            'Error: xlerobot navigation is disabled. The current XLerobot asset uses '
            'root prismatic joints for base motion, and direct waypoint commands can '
            'destabilize the articulation, blow the robot out of the scene, and wash out '
            'the head camera. Keep xlerobot in manipulation-only mode until a stable base '
            'controller is added.'
        )

    def _navigate_to_named(self, params: dict[str, Any]) -> str:
        if not self._experimental_navigation_enabled:
            return self._navigation_disabled_message()
        return super()._navigate_to_named(params)

    def _navigate_to_waypoint(self, params: dict[str, Any]) -> str:
        if not self._experimental_navigation_enabled:
            return self._navigation_disabled_message()
        return super()._navigate_to_waypoint(params)

    def _get_robot(self):
        if self._env is None:
            return None
        tasks = getattr(getattr(self._env, 'runner', None), 'current_tasks', {}) or {}
        if not tasks:
            return None
        task = next(iter(tasks.values()))
        robots = getattr(task, 'robots', {}) or {}
        if 'xlerobot' in robots:
            return robots['xlerobot']
        if robots:
            return next(iter(robots.values()))
        return None

    def _get_articulation(self):
        robot = self._get_robot()
        return getattr(robot, 'articulation', None)

    def _settle_env(self, steps: int = 12) -> None:
        if self._env is None:
            return
        for _ in range(max(1, steps)):
            with self._env_lock:
                self._last_obs, _, _, _, _ = self._env.step(action={})

    def _set_joint_positions_dict(self, updates: dict[str, float], settle_steps: int = 12) -> str:
        articulation = self._get_articulation()
        if articulation is None:
            return "Error: API not started. Dispatch action_type='enter_simulation' first."

        joint_positions = np.array(articulation.get_joint_positions(), dtype=np.float32)
        joint_velocities = np.array(articulation.get_joint_velocities(), dtype=np.float32)
        changed = 0
        for joint_name, value in updates.items():
            joint_index = articulation.get_dof_index(str(joint_name))
            joint_positions[joint_index] = float(value)
            joint_velocities[joint_index] = 0.0
            changed += 1
        articulation.set_joint_positions(joint_positions)
        articulation.set_joint_velocities(joint_velocities)
        self._settle_env(steps=settle_steps)
        return f'Applied {changed} joint target(s).'

    def _set_joint_targets(self, params: dict[str, Any]) -> str:
        joints = params.get('joints')
        if not isinstance(joints, dict) or not joints:
            return 'Error: joints must be a non-empty object.'
        updates = {str(name): float(value) for name, value in joints.items()}
        return self._set_joint_positions_dict(updates, settle_steps=int(params.get('settle_steps', 12)))

    def _set_gripper(self, params: dict[str, Any]) -> str:
        if 'value' not in params:
            return 'Error: value is required.'
        side = str(params.get('side', 'right')).strip().lower()
        value = float(params['value'])
        if side == 'right':
            updates = {'Jaw': value}
        elif side == 'left':
            updates = {'Jaw_2': value}
        elif side == 'both':
            updates = {'Jaw': value, 'Jaw_2': value}
        else:
            return "Error: side must be 'left', 'right', or 'both'."
        msg = self._set_joint_positions_dict(updates, settle_steps=int(params.get('settle_steps', 12)))
        return msg.replace('Applied', 'Set gripper(s); applied')

    def _navigate_xy(
        self,
        xy: list[float],
        *,
        max_steps: int,
        threshold: float,
        action_name_override: str | None = None,
        arm_target: list[float] | None = None,
        arm_action_name: str = 'arm_joint_controller',
    ) -> str:
        del max_steps, threshold, action_name_override, arm_target, arm_action_name
        if not self._experimental_navigation_enabled:
            return self._navigation_disabled_message()
        articulation = self._get_articulation()
        if articulation is None:
            return "Error: API not started. Dispatch action_type='enter_simulation' first."
        joint_positions = np.array(articulation.get_joint_positions(), dtype=np.float32)
        joint_velocities = np.array(articulation.get_joint_velocities(), dtype=np.float32)
        x_index = articulation.get_dof_index(self._BASE_JOINTS[0])
        y_index = articulation.get_dof_index(self._BASE_JOINTS[1])
        joint_positions[x_index] = float(xy[0])
        joint_positions[y_index] = float(xy[1])
        joint_velocities[x_index] = 0.0
        joint_velocities[y_index] = 0.0
        articulation.set_joint_positions(joint_positions)
        articulation.set_joint_velocities(joint_velocities)
        self._settle_env(steps=12)
        return f'navigate ok: reached xy=({xy[0]:.4f},{xy[1]:.4f}), dist=0.0000'

    @staticmethod
    def _build_api(
        *,
        scene_asset_path: str,
        robot_start: Any,
        arm_mass_scale: float,
        objects_spec: Any,
        api_kwargs: dict[str, Any],
        robot_usd_path: str = '',
    ):
        del arm_mass_scale
        pythonpath = api_kwargs.pop('pythonpath', None)
        if pythonpath:
            if isinstance(pythonpath, str):
                pythonpath = [pythonpath]
            import sys

            for entry in reversed(pythonpath):
                ep = str(Path(str(entry)).expanduser().resolve())
                if ep not in sys.path:
                    sys.path.insert(0, ep)

        objects_mod = importlib.import_module('internutopia_extension.configs.objects')
        DynamicCubeCfg = objects_mod.DynamicCubeCfg
        VisualCubeCfg = objects_mod.VisualCubeCfg

        rs = tuple(float(x) for x in (robot_start if isinstance(robot_start, (list, tuple)) else tuple(robot_start)))
        objects = []
        if isinstance(objects_spec, list):
            for item in objects_spec:
                if not isinstance(item, dict):
                    continue
                kind = str(item.get('kind', 'dynamic_cube')).strip().lower()
                cfg = {
                    'name': item['name'],
                    'prim_path': item['prim_path'],
                    'position': tuple(float(x) for x in item['position']),
                    'scale': tuple(float(x) for x in item['scale']),
                    'color': item.get('color', (0.5, 0.5, 0.5)),
                }
                if kind == 'visual_cube':
                    cfg['color'] = list(cfg['color'])
                    objects.append(VisualCubeCfg(**cfg))
                else:
                    objects.append(DynamicCubeCfg(**cfg))

        headless = api_kwargs.pop('headless', None)
        if headless is None:
            headless = not bool(api_kwargs.pop('force_gui', False))

        return _XLerobotSimAPI(
            scene_asset_path=scene_asset_path,
            robot_start=rs,
            robot_usd_path=robot_usd_path,
            objects=objects,
            headless=bool(headless),
            **api_kwargs,
        )

    @staticmethod
    def _extract_robot_obs(obs_data: Any):
        if isinstance(obs_data, dict) and 'xlerobot' in obs_data:
            return obs_data['xlerobot']
        if isinstance(obs_data, dict) and 'xlerobot_0' in obs_data:
            return obs_data['xlerobot_0']
        if isinstance(obs_data, list) and obs_data:
            first = obs_data[0]
            if isinstance(first, dict) and 'xlerobot' in first:
                return first['xlerobot']
            if isinstance(first, dict) and 'xlerobot_0' in first:
                return first['xlerobot_0']
        return None