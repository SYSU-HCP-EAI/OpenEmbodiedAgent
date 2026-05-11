from collections import OrderedDict
from typing import Any, List

import numpy as np

from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.robot.controller import BaseController
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia_extension.configs.controllers import RotateControllerCfg


@BaseController.register('RotateController')
class RotateController(BaseController):
    """Controller for turning to a certain orientation by utilizing a move-by-speed controller as sub-controller."""

    def __init__(self, config: RotateControllerCfg, robot: BaseRobot, scene: IScene) -> None:
        self._user_config = None
        self.goal_orientation: np.ndarray = None
        self.threshold: float = None

        self.rotation_speed = config.rotation_speed if config.rotation_speed is not None else 3.0
        self.threshold = config.threshold if config.threshold is not None else 0.02

        super().__init__(config=config, robot=robot, scene=scene)

    @staticmethod
    def get_delta_z_rot(
        start_orientation,
        goal_orientation,
    ) -> float:
        from omni.isaac.core.utils.rotations import quat_to_euler_angles

        delta_z_rot = quat_to_euler_angles(goal_orientation)[-1] - quat_to_euler_angles(start_orientation)[-1]
        delta_z_rot = delta_z_rot % (2 * np.pi)
        if delta_z_rot > np.pi:
            delta_z_rot = delta_z_rot - 2 * np.pi

        return delta_z_rot

    def forward(
        self,
        start_orientation: np.ndarray,
        goal_orientation: np.ndarray,
        rotation_speed: float = 3,
        threshold: float = 0.02,
    ) -> ArticulationAction:
        self.goal_orientation = goal_orientation
        self.threshold = threshold

        delta_z_rot = RotateController.get_delta_z_rot(
            start_orientation=start_orientation, goal_orientation=goal_orientation
        )
        if abs(delta_z_rot) < threshold:
            delta_z_rot = 0

        # Never rotate faster than rotation_speed.
        if abs(delta_z_rot) < 1.0:
            rotation_speed *= delta_z_rot
        else:
            rotation_speed *= np.sign(delta_z_rot)

        return self.sub_controllers[0].forward(
            forward_speed=0.0,
            rotation_speed=rotation_speed,
        )

    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        """
        Args:
            action (List | np.ndarray): n-element 1d array containing:
              0. goal_orientation in quat (np.ndarray)
        """
        assert len(action) == 1, 'action must contain 1 elements'
        start_orientation = self.robot.get_pose()[1]
        return self.forward(
            start_orientation=start_orientation,
            goal_orientation=action[0],
            rotation_speed=self.rotation_speed,
            threshold=self.threshold,
        )

    def get_obs(self) -> OrderedDict[str, Any]:
        if self.goal_orientation is None or self.threshold is None:
            return {}
        start_orientation = self.robot.get_pose()[1]
        delta_z_rot = RotateController.get_delta_z_rot(
            start_orientation=start_orientation, goal_orientation=self.goal_orientation
        )
        finished = True if abs(delta_z_rot) < self.threshold else False
        obs = {
            'finished': finished,
        }
        return self._make_ordered(obs)


# Use class-var inject controllers types' class
BaseController.controllers['RotateController'] = RotateController
