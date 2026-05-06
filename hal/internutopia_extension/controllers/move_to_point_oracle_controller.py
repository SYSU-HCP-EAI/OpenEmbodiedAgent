from collections import OrderedDict
from typing import Any, List

import numpy as np

from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.robot.controller import BaseController
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia_extension.configs.controllers import MoveToPointBySpeedControllerCfg


@BaseController.register('MoveToPointOracleController')
class MoveToPointOracleController(BaseController):
    """Controller for moving to a target point by utilizing a move-by-speed controller as sub-controller."""

    def __init__(self, config: MoveToPointBySpeedControllerCfg, robot: BaseRobot, scene: IScene) -> None:
        self.last_threshold = None
        self._user_config = None
        self.goal_position: np.ndarray | None = None
        self.threshold: float | None = None

        self.forward_speed = config.forward_speed if config.forward_speed is not None else 1.0
        self.rotation_speed = config.rotation_speed if config.rotation_speed is not None else 8.0
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
        start_position: np.ndarray,
        start_orientation: np.ndarray,
        goal_position: np.ndarray,
        forward_speed: float = 1,
        rotation_speed: float = 8,
        threshold: float = 0.02,
    ) -> ArticulationAction:
        from omni.isaac.core.utils.rotations import euler_angles_to_quat

        self.goal_position = goal_position
        self.goal_position[-1] = 0
        self.last_threshold = threshold

        # 1. Calculate the direction vector from start to goal
        distance_to_goal = np.linalg.norm(goal_position[:2] - start_position[:2])
        if distance_to_goal < threshold:
            goal_position[-1] = 1.6
            return {'pos': goal_position, 'rot': None}

        direction_to_goal = (goal_position[:2] - start_position[:2]) / distance_to_goal

        # 2. Get the current z-axis rotation difference between the start and goal directions
        goal_orientation = euler_angles_to_quat(
            [0, 0, np.arctan2(goal_position[1] - start_position[1], goal_position[0] - start_position[0])]
        )
        theta = MoveToPointOracleController.get_delta_z_rot(start_orientation, goal_orientation)
        delta_z_rotation = (theta + np.pi) % (2 * np.pi) - np.pi

        # Rotate and move forward, but limit the movement to avoid overshooting the goal
        delta_z_rotation = np.clip(delta_z_rotation, -rotation_speed, rotation_speed)

        # Ensure forward movement does not exceed the distance to the goal
        actual_forward_speed = min(forward_speed, distance_to_goal)

        # Move the object towards the goal by the calculated forward speed
        forward_movement = actual_forward_speed * direction_to_goal
        new_position = start_position + np.array(
            [forward_movement[0], forward_movement[1], 0]
        )  # Update position in xy-plane

        # current_z_angle = quat_to_euler_angles(start_orientation)[2]
        # new_z_angle = current_z_angle + delta_z_rotation
        new_position[2] = 1.6

        return {
            'pos': new_position,
            'rot': [0, 0, np.arctan2(goal_position[1] - start_position[1], goal_position[0] - start_position[0])],
        }

    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        """Convert input action (in 1d array format) to joint signals to apply.

        Args:
            action (List | np.ndarray): n-element 1d array containing
              0. goal_position (np.ndarray)

        Returns:
            ArticulationAction: joint signals to apply.
        """
        assert len(action) == 1, 'action must contain 1 elements'
        start_position, start_orientation = self.robot.get_pose()
        return self.forward(
            start_position=start_position,
            start_orientation=start_orientation,
            goal_position=np.array(action[0]),
            forward_speed=self.forward_speed,
            rotation_speed=self.rotation_speed,
            threshold=self.threshold * self.robot.get_robot_scale()[0],
        )

    def get_obs(self) -> OrderedDict[str, Any]:
        if self.goal_position is None or self.last_threshold is None:
            return self._make_ordered()
        start_position = self.robot.get_pose()[0]
        start_position[-1] = 0
        dist_from_goal = np.linalg.norm(start_position[:2] - self.goal_position[:2])
        finished = True if dist_from_goal < self.last_threshold else False
        obs = {
            'finished': finished,
        }
        return self._make_ordered(obs)
