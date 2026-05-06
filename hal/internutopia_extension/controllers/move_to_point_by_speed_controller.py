from collections import OrderedDict
from typing import Any, List

import numpy as np

from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.robot.controller import BaseController
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia_extension.configs.controllers import MoveToPointBySpeedControllerCfg


@BaseController.register('MoveToPointBySpeedController')
class MoveToPointBySpeedController(BaseController):
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
    def get_angle(
        start_position,
        start_orientation,
        goal_position,
    ):
        try:
            from isaacsim.core.utils.rotations import quat_to_euler_angles
        except ImportError:
            from omni.isaac.core.utils.rotations import quat_to_euler_angles

        normal_vec = np.array([0, 0, 1])
        robot_z_rot = quat_to_euler_angles(start_orientation)[-1]

        robot_vec = np.array([np.cos(robot_z_rot), np.sin(robot_z_rot), 0])
        robot_vec /= np.linalg.norm(robot_vec)

        target_vec = goal_position - start_position
        if np.linalg.norm(target_vec) == 0:
            return 0
        target_vec /= np.linalg.norm(target_vec)

        dot_prod = np.dot(robot_vec, target_vec)
        cross_prod = np.cross(robot_vec, target_vec)

        # Handle errors in floating-point arithmetic.
        if dot_prod > 1.0:
            dot_prod = 1.0
        angle = np.arccos(dot_prod)
        angle_sign = np.sign(np.dot(normal_vec, cross_prod))

        signed_angle = angle * angle_sign
        return signed_angle

    def forward(
        self,
        start_position: np.ndarray,
        start_orientation: np.ndarray,
        goal_position: np.ndarray,
        forward_speed: float = 1,
        rotation_speed: float = 8,
        threshold: float = 0.02,
    ) -> ArticulationAction:
        # Just make sure we ignore z components
        start_position[-1] = 0
        goal_position[-1] = 0

        self.goal_position = goal_position
        self.last_threshold = threshold

        dist_from_goal = np.linalg.norm(start_position - goal_position)
        angle_to_goal = MoveToPointBySpeedController.get_angle(start_position, start_orientation, goal_position)

        # Set forward_speed to 0 if we are heading the wrong direction.
        if abs(angle_to_goal) > np.pi / 2:
            forward_speed = 0

        if dist_from_goal < threshold:
            # We have reached the goal position.
            return self.sub_controllers[0].forward(
                forward_speed=0.0,
                rotation_speed=0.0,
            )

        if dist_from_goal < threshold * 2:
            forward_speed *= (dist_from_goal / (threshold * 2)) ** 3

        forward_speed *= (1 - (abs(angle_to_goal) * 2 / np.pi)) ** 3

        # Never rotate faster than rotation_speed.
        if abs(angle_to_goal) < 1:
            rotation_speed *= angle_to_goal
        else:
            rotation_speed *= np.sign(angle_to_goal)
        # Call sub controller
        return self.sub_controllers[0].forward(
            forward_speed=forward_speed,
            rotation_speed=rotation_speed,
        )

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
            threshold=self.threshold,
        )

    def get_obs(self) -> OrderedDict[str, Any]:
        if self.goal_position is None or self.last_threshold is None:
            return self._make_ordered()
        start_position = self.robot.get_pose()[0]
        start_position[-1] = 0
        dist_from_goal = np.linalg.norm(start_position - self.goal_position)
        finished = True if dist_from_goal < self.last_threshold else False
        obs = {
            'finished': finished,
        }
        return self._make_ordered(obs)
