from collections import OrderedDict

import numpy as np

from internutopia.core.robot.articulation import IArticulation
from internutopia.core.robot.articulation_subset import ArticulationSubset
from internutopia.core.robot.rigid_body import IRigidBody
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia.core.util import log
from internutopia_extension.configs.robots.pipergo2 import PiperGo2RobotCfg


@BaseRobot.register('PiperGo2Robot')
class PiperGo2Robot(BaseRobot):
    EEF_CENTER_OFFSET = np.array([0.0, 0.0, 0.12], dtype=float)
    ARM_LINK_NAMES = [
        'piper_base_link',
        'piper_link1',
        'piper_link2',
        'piper_link3',
        'piper_link4',
        'piper_link5',
        'piper_link6',
        'piper_gripper_base',
        'piper_link7',
        'piper_link8',
    ]

    def __init__(self, config: PiperGo2RobotCfg, scene: IScene):
        super().__init__(config, scene)
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        log.debug(f'pipergo2 {config.name}: position    : {self._start_position}')
        log.debug(f'pipergo2 {config.name}: orientation : {self._start_orientation}')
        log.debug(f'pipergo2 {config.name}: usd_path         : {config.usd_path}')
        log.debug(f'pipergo2 {config.name}: config.prim_path : {config.prim_path}')

        self._robot_scale = np.array(config.scale) if config.scale is not None else np.array([1.0, 1.0, 1.0])
        self.articulation = IArticulation.create(
            prim_path=config.prim_path,
            name=config.name,
            position=self._start_position,
            orientation=self._start_orientation,
            usd_path=config.usd_path,
            scale=self._robot_scale,
        )

    def post_reset(self):
        super().post_reset()
        self._robot_base = self._resolve_robot_base()
        self._arm_base = self._resolve_arm_base()
        self._end_effector = self._resolve_end_effector()
        self._apply_arm_mass_scale()
        self.set_gains()

    def _resolve_robot_base(self) -> IRigidBody:
        preferred_names = []
        if getattr(self.config, 'base_link_name', None):
            preferred_names.append(self.config.base_link_name)
        preferred_names.extend(['trunk', 'base'])

        for link_name in preferred_names:
            rigid_body = self._rigid_body_map.get(self.config.prim_path + '/' + link_name)
            if rigid_body is not None:
                log.debug(f'pipergo2 {self.config.name}: using base link {link_name}')
                return rigid_body

        available_links = sorted(path.split('/')[-1] for path in self._rigid_body_map)
        raise KeyError(
            f'Cannot find base link for {self.config.name}. '
            f'Tried {preferred_names}. Available links: {available_links}'
        )

    def get_robot_scale(self):
        return self._robot_scale

    def get_robot_base(self) -> IRigidBody:
        return self._robot_base

    def get_arm_base(self) -> IRigidBody:
        return self._arm_base

    def get_end_effector(self) -> IRigidBody:
        return self._end_effector

    def get_pose(self):
        return self._robot_base.get_pose()

    def _resolve_end_effector(self) -> IRigidBody:
        eef_path = self.config.prim_path + '/piper_gripper_base'
        rigid_body = self._rigid_body_map.get(eef_path)
        if rigid_body is None:
            raise KeyError(f'Cannot find end effector link at {eef_path}')
        return rigid_body

    def _resolve_arm_base(self) -> IRigidBody:
        arm_base_path = self.config.prim_path + '/piper_base_link'
        rigid_body = self._rigid_body_map.get(arm_base_path)
        if rigid_body is None:
            raise KeyError(f'Cannot find arm base link at {arm_base_path}')
        return rigid_body

    def _apply_arm_mass_scale(self):
        arm_mass_scale = getattr(self.config, 'arm_mass_scale', None)
        if arm_mass_scale is None or arm_mass_scale >= 0.999:
            return

        for link_name in self.ARM_LINK_NAMES:
            rigid_body = self._rigid_body_map.get(self.config.prim_path + '/' + link_name)
            if rigid_body is None:
                continue
            try:
                current_mass = rigid_body.get_mass()
                rigid_body.set_mass(max(1e-4, current_mass * arm_mass_scale))
            except Exception as exc:
                log.warning(f'Failed to scale mass for {link_name}: {exc}')

    def apply_action(self, action: dict):
        for controller_name, controller_action in action.items():
            if controller_name not in self.controllers:
                log.warning(f'unknown controller {controller_name} in action')
                continue
            controller = self.controllers[controller_name]
            control = controller.action_to_control(controller_action)
            self.articulation.apply_action(control)

    def get_obs(self) -> OrderedDict:
        position, orientation = self._robot_base.get_pose()
        arm_base_position, arm_base_orientation = self._arm_base.get_pose()
        eef_base_position, eef_orientation = self._end_effector.get_pose()
        eef_position = self._compute_eef_center(
            base_position=eef_base_position,
            base_orientation=eef_orientation,
        )
        obs = {
            'position': position,
            'orientation': orientation,
            'joint_positions': self.articulation.get_joint_positions(),
            'joint_velocities': self.articulation.get_joint_velocities(),
            'arm_base_position': arm_base_position,
            'arm_base_orientation': arm_base_orientation,
            'eef_position': eef_position,
            'eef_orientation': eef_orientation,
            'controllers': {},
            'sensors': {},
        }

        for controller_name, controller in self.controllers.items():
            obs['controllers'][controller_name] = controller.get_obs()
        for sensor_name, sensor in self.sensors.items():
            obs['sensors'][sensor_name] = sensor.get_data()
        return self._make_ordered(obs)

    @classmethod
    def _compute_eef_center(
        cls,
        base_position: np.ndarray,
        base_orientation: np.ndarray,
    ) -> np.ndarray:
        rotation = cls._quat_to_rotmat(base_orientation)
        return np.array(base_position, dtype=float) + rotation @ cls.EEF_CENTER_OFFSET

    @staticmethod
    def _quat_to_rotmat(quat_wxyz: np.ndarray) -> np.ndarray:
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

    def set_gains(self):
        # Reuse the quadruped leg gains used by Aliengo for the 12 locomotion joints.
        leg_joint_names = [
            'FL_hip_joint',
            'FR_hip_joint',
            'RL_hip_joint',
            'RR_hip_joint',
            'FL_thigh_joint',
            'FR_thigh_joint',
            'RL_thigh_joint',
            'RR_thigh_joint',
            'FL_calf_joint',
            'FR_calf_joint',
            'RL_calf_joint',
            'RR_calf_joint',
        ]
        leg_joint_subset = ArticulationSubset(self.articulation, leg_joint_names)
        leg_kps = np.array([40.0] * len(leg_joint_names))
        leg_kds = np.array([2.0] * len(leg_joint_names))

        arm_joint_names = [
            'piper_j1',
            'piper_j2',
            'piper_j3',
            'piper_j4',
            'piper_j5',
            'piper_j6',
        ]
        arm_joint_subset = ArticulationSubset(self.articulation, arm_joint_names)
        arm_kps = np.array([120.0] * len(arm_joint_names))
        arm_kds = np.array([8.0] * len(arm_joint_names))

        gripper_joint_names = ['piper_j7', 'piper_j8']
        gripper_joint_subset = ArticulationSubset(self.articulation, gripper_joint_names)
        gripper_kps = np.array([80.0] * len(gripper_joint_names))
        gripper_kds = np.array([4.0] * len(gripper_joint_names))

        self.articulation.set_gains(
            kps=leg_kps,
            kds=leg_kds,
            joint_indices=leg_joint_subset.joint_indices,
        )
        self.articulation.set_gains(
            kps=arm_kps,
            kds=arm_kds,
            joint_indices=arm_joint_subset.joint_indices,
        )
        self.articulation.set_gains(
            kps=gripper_kps,
            kds=gripper_kds,
            joint_indices=gripper_joint_subset.joint_indices,
        )
        self.articulation.set_solver_position_iteration_count(8)
        self.articulation.set_solver_velocity_iteration_count(0)
        self.articulation.set_enabled_self_collisions(True)
