import os
from collections import OrderedDict

import numpy as np

from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia.core.util import log
from internutopia_extension.configs.robots.mocap_controlled_franka import (
    MocapControlledFrankaRobotCfg,
)


@BaseRobot.register('MocapControlledFrankaRobot')
class MocapControlledFrankaRobot(BaseRobot):
    def __init__(self, config: MocapControlledFrankaRobotCfg, scene: IScene):
        from omni.isaac.franka import Franka

        super().__init__(config, scene)
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        log.debug(f'franka {config.name}: position    : ' + str(self._start_position))
        log.debug(f'franka {config.name}: orientation : ' + str(self._start_orientation))

        usd_path = os.path.abspath(config.usd_path)

        log.debug(f'franka {config.name}: usd_path         : ' + str(usd_path))
        log.debug(f'franka {config.name}: config.prim_path : ' + str(config.prim_path))
        self.articulation = Franka(
            prim_path=config.prim_path,
            name=config.name,
            position=self._start_position,
            orientation=self._start_orientation,
            usd_path=usd_path,
        )

        self.articulation.set_solver_velocity_iteration_count(64)

        self._robot_scale = np.array([1.0, 1.0, 1.0])
        if config.scale is not None:
            self._robot_scale = np.array(config.scale)
            self.articulation.set_local_scale(self._robot_scale)

        self.last_action = []

    def get_robot_scale(self):
        return self._robot_scale

    def get_robot_ik_base(self):
        return self._robot_ik_base

    def post_reset(self):
        super().post_reset()
        self._robot_ik_base = self._rigid_body_map[self.config.prim_path + '/panda_link0']
        self.articulation._articulation_view.set_max_joint_velocities([1.0] * 9)
        stiffnesses = np.array([1e7, 1e7, 1e7, 1e7, 1e7, 1e7, 1e7, 1e4, 0])
        dampings = np.array([1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e3, 0])
        self.articulation._articulation_view.set_gains(kps=stiffnesses, kds=dampings)

    @staticmethod
    def action_to_dict(action):
        def numpy_to_list(array):
            return array.tolist() if isinstance(array, np.ndarray) else array

        return {
            'joint_efforts': numpy_to_list(action.joint_efforts),
            'joint_indices': numpy_to_list(action.joint_indices),
            'joint_positions': numpy_to_list(action.joint_positions),
            'joint_velocities': numpy_to_list(action.joint_velocities),
        }

    def apply_action(self, action: dict):
        """
        Args:
            action (dict): inputs for controllers.
        """
        self.last_action = []
        for controller_name, controller_action in action.items():
            if controller_name not in self.controllers:
                log.warn(f'unknown controller {controller_name} in action')
                continue
            controller = self.controllers[controller_name]
            control = controller.action_to_control(controller_action)
            self.articulation.apply_action(control)

            self.last_action.append(self.action_to_dict(control))

    def get_last_action(self):
        return self.last_action

    def get_obs(self) -> OrderedDict:
        position, orientation = self.articulation.get_pose()

        # custom
        obs = {
            'position': position,
            'orientation': orientation,
            'joint_action': self.get_last_action(),
            'controllers': {},
            'sensors': {},
        }

        eef_world_pose = self.articulation.end_effector.get_pose()
        obs['eef_position'] = eef_world_pose[0]
        obs['eef_orientation'] = eef_world_pose[1]

        # common
        for c_obs_name, controller_obs in self.controllers.items():
            obs['controllers'][c_obs_name] = controller_obs.get_obs()
        for sensor_name, sensor_obs in self.sensors.items():
            obs['sensors'][sensor_name] = sensor_obs.get_data()
        return self._make_ordered(obs)
