# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import os
from collections import OrderedDict
from typing import Any, List, Optional

import numpy as np

from internutopia.core.robot.isaacsim.articulation import IsaacsimArticulation
from internutopia.core.robot.rigid_body import IRigidBody
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia.core.util import log
from internutopia_extension.configs.robots.franka import FrankaRobotCfg


class Franka(IsaacsimArticulation):
    # TODO: change IsaacsimArticulation to IArticulation
    def __init__(
        self,
        prim_path: str,
        name: str = 'franka_robot',
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        end_effector_prim_name: Optional[str] = None,
        gripper_dof_names: Optional[List[str]] = None,
        gripper_open_position: Optional[np.ndarray] = None,
        gripper_closed_position: Optional[np.ndarray] = None,
        deltas: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
    ) -> None:
        from isaacsim.core.utils.stage import get_stage_units
        from isaacsim.robot.manipulators.grippers.parallel_gripper import (
            ParallelGripper,
        )

        self._end_effector = None
        self._gripper = None
        self._end_effector_prim_name = end_effector_prim_name
        if self._end_effector_prim_name is None:
            self._end_effector_prim_path = prim_path + '/panda_rightfinger'
        else:
            self._end_effector_prim_path = prim_path + '/' + end_effector_prim_name
        if gripper_dof_names is None:
            gripper_dof_names = ['panda_finger_joint1', 'panda_finger_joint2']
        if gripper_open_position is None:
            gripper_open_position = np.array([0.05, 0.05]) / get_stage_units()
        if gripper_closed_position is None:
            gripper_closed_position = np.array([0.0, 0.0])
        super().__init__(
            usd_path=usd_path,
            prim_path=prim_path,
            name=name,
            position=position,
            orientation=orientation,
            scale=scale,
        )
        if gripper_dof_names is not None:
            if deltas is None:
                deltas = np.array([0.05, 0.05]) / get_stage_units()
            self._gripper = ParallelGripper(
                end_effector_prim_path=self._end_effector_prim_path,
                joint_prim_names=gripper_dof_names,
                joint_opened_positions=gripper_open_position,
                joint_closed_positions=gripper_closed_position,
                action_deltas=deltas,
            )
        return

    @property
    def end_effector(self) -> IRigidBody:
        return self._end_effector

    @property
    def gripper(self):
        return self._gripper

    def initialize(self, physics_sim_view=None) -> None:
        self.unwrap().initialize(physics_sim_view)
        self._end_effector = IRigidBody.create(prim_path=self._end_effector_prim_path, name=self.name + '_end_effector')
        self._end_effector.unwrap().initialize(physics_sim_view)
        self._gripper.initialize(
            physics_sim_view=physics_sim_view,
            articulation_apply_action_func=self.apply_action,
            get_joint_positions_func=self.get_joint_positions,
            set_joint_positions_func=self.set_joint_positions,
            dof_names=self.dof_names,
        )
        return

    def post_reset(self) -> None:
        self.unwrap().post_reset()
        self._gripper.post_reset()
        self._articulation_controller.switch_dof_control_mode(
            dof_index=self.gripper.joint_dof_indicies[0], mode='position'
        )
        self._articulation_controller.switch_dof_control_mode(
            dof_index=self.gripper.joint_dof_indicies[1], mode='position'
        )
        return


@BaseRobot.register('FrankaRobot')
class FrankaRobot(BaseRobot):
    def __init__(self, config: FrankaRobotCfg, scene: IScene):
        super().__init__(config, scene)
        self._robot_ik_base = None
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        log.debug(f'franka {config.name}: position    : ' + str(self._start_position))
        log.debug(f'franka {config.name}: orientation : ' + str(self._start_orientation))

        usd_path = config.usd_path

        log.debug(f'franka {config.name}: usd_path         : ' + str(usd_path))
        log.debug(f'franka {config.name}: config.prim_path : ' + str(config.prim_path))
        self._robot_scale = np.array([1.0, 1.0, 1.0])
        if config.scale is not None:
            self._robot_scale = np.array(config.scale)
        self.articulation = Franka(
            prim_path=config.prim_path,
            name=config.name,
            position=self._start_position,
            orientation=self._start_orientation,
            usd_path=os.path.abspath(usd_path),
            scale=self._robot_scale,
        )

        self.last_action = []

    def get_robot_scale(self):
        return self._robot_scale

    def get_robot_ik_base(self):
        return self._robot_ik_base

    def post_reset(self):
        super().post_reset()
        self._robot_ik_base = self._rigid_body_map[self.config.prim_path + '/panda_link0']

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

    def get_obs(self) -> OrderedDict[str, Any]:
        position, orientation = self.articulation.get_pose()

        # custom
        obs = {
            'position': position,
            'orientation': orientation,
            'joint_action': self.get_last_action(),
            'controllers': {},
            'sensors': {},
        }

        eef_pose = self.articulation.end_effector.get_pose()
        obs['eef_position'] = eef_pose[0]
        obs['eef_orientation'] = eef_pose[1]

        # common
        for c_obs_name, controller_obs in self.controllers.items():
            obs['controllers'][c_obs_name] = controller_obs.get_obs()
        for sensor_name, sensor_obs in self.sensors.items():
            obs['sensors'][sensor_name] = sensor_obs.get_data()
        return self._make_ordered(obs)
