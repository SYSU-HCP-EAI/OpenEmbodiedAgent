from collections import OrderedDict
from typing import Any, List

import numpy as np

from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.robot.controller import BaseController
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia_extension.configs.controllers import GripperControllerCfg


@BaseController.register('GripperController')
class GripperController(BaseController):
    def __init__(self, config: GripperControllerCfg, robot: BaseRobot, scene: IScene):
        self._gripper = robot.articulation.gripper  # for franka is OK
        super().__init__(config, robot, scene)

    def forward(self, action: str) -> ArticulationAction:
        return self._gripper.forward(action)

    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        """
        Args:
            action (List | np.ndarray): 1-element 1d array.
        """
        assert len(action) == 1 and action[0] in [
            'open',
            'close',
        ], f'action must be a list with one str elem, which is one of "open" / "close", but got {action}'
        return self.forward(action[0])

    def get_obs(self) -> OrderedDict[str, Any]:
        return OrderedDict({'gripper_pos': self._gripper.get_joint_positions()})
