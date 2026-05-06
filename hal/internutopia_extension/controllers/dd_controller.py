from typing import List

import numpy as np

from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.robot.controller import BaseController
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia_extension.configs.controllers import DifferentialDriveControllerCfg


@BaseController.register('DifferentialDriveController')
class DifferentialDriveController(BaseController):
    def __init__(self, config: DifferentialDriveControllerCfg, robot: BaseRobot, scene: IScene) -> None:
        super().__init__(config=config, robot=robot, scene=scene)
        self._robot_scale = self.robot.get_robot_scale()[0]
        self._wheel_base = config.wheel_base * self._robot_scale
        self._wheel_radius = config.wheel_radius * self._robot_scale

    def forward(
        self,
        forward_speed: float = 0,
        rotation_speed: float = 0,
    ) -> ArticulationAction:
        left_wheel_vel = ((2 * forward_speed) - (rotation_speed * self._wheel_base)) / (2 * self._wheel_radius)
        right_wheel_vel = ((2 * forward_speed) + (rotation_speed * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=[left_wheel_vel, right_wheel_vel])

    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        """
        Args:
            action (List | np.ndarray): n-element 1d array containing:
              0. forward_speed (float)
              1. rotation_speed (float)
        """
        assert len(action) == 2, 'action must contain 2 elements'
        return self.forward(
            forward_speed=action[0],
            rotation_speed=action[1],
        )
