# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import re
from collections import OrderedDict
from typing import Optional

import numpy as np

from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.robot.isaacsim.articulation import IsaacsimArticulation
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia.core.util import log
from internutopia_extension.configs.robots.jetbot import JetbotRobotCfg


class WheeledRobot(IsaacsimArticulation):
    # TODO: change IsaacsimArticulation to IArticulation
    def __init__(
        self,
        prim_path: str,
        name: str = 'wheeled_robot',
        robot_path: Optional[str] = None,
        wheel_dof_names: Optional[str] = None,
        wheel_dof_indices: Optional[int] = None,
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
    ) -> None:
        if robot_path is not None:
            robot_path = '/' + robot_path
            # regex: remove all prefixing "/", need at least one prefix "/" to work
            robot_path = re.sub('^([^\/]*)\/*', '', '/' + robot_path)  # noqa
            prim_path = prim_path + '/' + robot_path

        super().__init__(
            usd_path=usd_path,
            prim_path=prim_path,
            name=name,
            position=position,
            orientation=orientation,
            scale=scale,
        )
        self._wheel_dof_names = wheel_dof_names
        self._wheel_dof_indices = wheel_dof_indices
        # TODO: check the default state and how to reset
        return

    @property
    def wheel_dof_indices(self):
        return self._wheel_dof_indices

    def get_wheel_positions(self):
        full_dofs_positions = self.get_joint_positions()
        wheel_joint_positions = [full_dofs_positions[i] for i in self._wheel_dof_indices]
        return wheel_joint_positions

    def set_wheel_positions(self, positions) -> None:
        full_dofs_positions = [None] * self.num_dof
        for i in range(self._num_wheel_dof):
            full_dofs_positions[self._wheel_dof_indices[i]] = positions[i]
        self.set_joint_positions(positions=np.array(full_dofs_positions))
        return

    def get_wheel_velocities(self):
        full_dofs_velocities = self.get_joint_velocities()
        wheel_dof_velocities = [full_dofs_velocities[i] for i in self._wheel_dof_indices]
        return wheel_dof_velocities

    def set_wheel_velocities(self, velocities) -> None:
        full_dofs_velocities = [None] * self.num_dof
        for i in range(self._num_wheel_dof):
            full_dofs_velocities[self._wheel_dof_indices[i]] = velocities[i]
        self.set_joint_velocities(velocities=np.array(full_dofs_velocities))
        return

    def apply_wheel_actions(self, actions: ArticulationAction) -> None:
        actions_length = actions.get_length()
        if actions_length is not None and actions_length != self._num_wheel_dof:
            raise Exception('ArticulationAction passed should be the same length as the number of wheels')
        joint_actions = ArticulationAction()
        if actions.joint_positions is not None:
            joint_actions.joint_positions = np.zeros(self.num_dof)  # for all dofs of the robot
            for i in range(self._num_wheel_dof):  # set only the ones that are the wheels
                joint_actions.joint_positions[self._wheel_dof_indices[i]] = actions.joint_positions[i]
        if actions.joint_velocities is not None:
            joint_actions.joint_velocities = np.zeros(self.num_dof)
            for i in range(self._num_wheel_dof):
                joint_actions.joint_velocities[self._wheel_dof_indices[i]] = actions.joint_velocities[i]
        if actions.joint_efforts is not None:
            joint_actions.joint_efforts = np.zeros(self.num_dof)
            for i in range(self._num_wheel_dof):
                joint_actions.joint_efforts[self._wheel_dof_indices[i]] = actions.joint_efforts[i]
        self.apply_action(control_actions=joint_actions)
        return

    def initialize(self, physics_sim_view=None) -> None:
        import carb

        self.unwrap().initialize(physics_sim_view=physics_sim_view)
        if self._wheel_dof_names is not None:
            self._wheel_dof_indices = [
                self.get_dof_index(self._wheel_dof_names[i]) for i in range(len(self._wheel_dof_names))
            ]
        elif self._wheel_dof_indices is None:
            carb.log_error('need to have either wheel names or wheel indices')

        self._num_wheel_dof = len(self._wheel_dof_indices)

        return

    def post_reset(self) -> None:
        self.unwrap().post_reset()
        self._articulation_controller.switch_control_mode(mode='velocity')
        return

    def get_articulation_controller_properties(self):
        return self._wheel_dof_names, self._wheel_dof_indices


@BaseRobot.register('JetbotRobot')
class JetbotRobot(BaseRobot):
    def __init__(self, config: JetbotRobotCfg, scene: IScene):
        super().__init__(config, scene)
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        log.debug(f'jetbot {config.name} position    : ' + str(self._start_position))
        log.debug(f'jetbot {config.name} orientation : ' + str(self._start_orientation))

        usd_path = config.usd_path

        log.debug(f'jetbot {config.name} usd_path         : ' + str(usd_path))
        log.debug(f'jetbot {config.name} config.prim_path : ' + str(config.prim_path))
        self.prim_path = str(config.prim_path)
        self._robot_scale = np.array([1.0, 1.0, 1.0])
        if config.scale is not None:
            self._robot_scale = np.array(config.scale)
        self.articulation = WheeledRobot(
            prim_path=config.prim_path,
            name=config.name,
            wheel_dof_names=['left_wheel_joint', 'right_wheel_joint'],
            position=self._start_position,
            orientation=self._start_orientation,
            usd_path=usd_path,
            scale=self._robot_scale,
        )

    def get_robot_scale(self):
        return self._robot_scale

    def get_pose(self):
        return self.articulation.get_pose()

    def apply_action(self, action: dict):
        """
        Args:
            action (dict): inputs for controllers.
        """
        for controller_name, controller_action in action.items():
            # print(controller_name, "=============", controller_action)
            if controller_name not in self.controllers:
                log.warning(f'unknown controller {controller_name} in action')
                continue
            controller = self.controllers[controller_name]
            control = controller.action_to_control(controller_action)
            self.articulation.apply_action(control)

    def get_obs(self) -> OrderedDict:
        position, orientation = self.articulation.get_pose()

        # custom
        obs = {
            'position': position,
            'orientation': orientation,
            'joint_positions': self.articulation.get_joint_positions(),
            'joint_velocities': self.articulation.get_joint_velocities(),
            'controllers': {},
            'sensors': {},
        }

        # common
        for c_obs_name, controller_obs in self.controllers.items():
            obs['controllers'][c_obs_name] = controller_obs.get_obs()
        for sensor_name, sensor_obs in self.sensors.items():
            obs['sensors'][sensor_name] = sensor_obs.get_data()
        return self._make_ordered(obs)
