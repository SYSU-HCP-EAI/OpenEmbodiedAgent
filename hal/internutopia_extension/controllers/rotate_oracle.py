from collections import OrderedDict
from typing import Any, List

import numpy as np

from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.robot.controller import BaseController
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia_extension.configs.controllers import RotateControllerCfg


@BaseController.register('RotateOracleController')
class RotateOracleController(BaseController):
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
        def quat_conjugate(q):
            w, x, y, z = q
            return np.array([w, -x, -y, -z])

        def quat_multiply(q1, q2):
            w1, x1, y1, z1 = q1
            w2, x2, y2, z2 = q2
            w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
            x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
            y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
            z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
            return np.array([w, x, y, z])

        def quat_to_z_angle(q):
            w, x, y, z = q
            return 2 * np.arctan2(z, w)

        # Extract the rotation angle around the z-axis
        q_diff = quat_multiply(goal_orientation, quat_conjugate(start_orientation))
        theta = quat_to_z_angle(q_diff)

        return theta

    def forward(
        self,
        start_orientation: np.ndarray,
        goal_orientation: np.ndarray,
        rotation_speed: float = 3,
        threshold: float = 0.02,
    ) -> ArticulationAction:
        from omni.isaac.core.utils.rotations import quat_to_euler_angles

        theta = RotateOracleController.get_delta_z_rot(
            start_orientation=start_orientation, goal_orientation=goal_orientation
        )
        theta = (theta + np.pi) % (2 * np.pi) - np.pi
        if abs(theta) < threshold:
            return {'pos': None, 'rot': [0, 0, quat_to_euler_angles(goal_orientation)[2]]}

        # Check if the rotation angle exceeds the step size (rotation_speed)
        if abs(theta) > rotation_speed:
            theta = rotation_speed * (theta / abs(theta))  # Limit to the step size

        # Return the updated z-axis angle
        current_z_angle = quat_to_euler_angles(start_orientation)[2]
        new_z_angle = current_z_angle + theta

        return {'pos': None, 'rot': [0, 0, new_z_angle]}

    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        """
        Args:
            action (List | np.ndarray): n-element 1d array containing:
              0. goal_orientation (float)
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
            return self._make_ordered()
        start_orientation = self.robot.get_pose()[1]
        delta_z_rot = RotateOracleController.get_delta_z_rot(
            start_orientation=start_orientation, goal_orientation=self.goal_orientation
        )
        finished = True if abs(delta_z_rot) < self.threshold else False
        obs = {
            'finished': finished,
        }
        return self._make_ordered(obs)


# Use class-var inject controllers types' class
BaseController.controllers['RotateOracleController'] = RotateOracleController
