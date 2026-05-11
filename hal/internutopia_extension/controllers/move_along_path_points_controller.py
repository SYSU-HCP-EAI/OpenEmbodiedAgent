from collections import OrderedDict
from copy import deepcopy
from typing import Any, List

import numpy as np

from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.robot.controller import BaseController
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia.core.util import log
from internutopia_extension.configs.controllers import MoveAlongPathPointsControllerCfg


@BaseController.register('MoveAlongPathPointsController')
class MoveAlongPathPointsController(BaseController):
    """Controller for moving alone a set of path points by utilizing a move-to-point controller as sub-controller."""

    def __init__(self, config: MoveAlongPathPointsControllerCfg, robot: BaseRobot, scene: IScene) -> None:
        self._user_config = None
        self.path_points: List[np.ndarray | List] = []
        self.path_point_idx = 0
        self.current_path_point: np.ndarray | None = None

        self.forward_speed = config.forward_speed if config.forward_speed is not None else 1.0
        self.rotation_speed = config.rotation_speed if config.rotation_speed is not None else 8.0
        self.threshold = config.threshold if config.threshold is not None else 0.02

        super().__init__(config=config, robot=robot, scene=scene)

    def forward(
        self,
        start_position: np.ndarray,
        start_orientation: np.ndarray,
        path_points: np.ndarray,
        forward_speed: float = 1,
        rotation_speed: float = 8,
        threshold: float = 0.02,
    ) -> ArticulationAction:

        if self.path_points != path_points:
            self.path_points = path_points
            self.path_point_idx = 0
            log.info('reset path points')
            self.current_path_point = np.array(deepcopy(self.path_points[self.path_point_idx]))
            self.current_path_point[-1] = 0

        # Just make sure we ignore z components
        start_position[-1] = 0
        dist_from_goal = np.linalg.norm(start_position - self.current_path_point)
        if dist_from_goal < threshold:
            if self.path_point_idx < len(self.path_points) - 1:
                self.path_point_idx += 1
                self.current_path_point = np.array(deepcopy(self.path_points[self.path_point_idx]))
                self.current_path_point[-1] = 0
                log.info(f'switch to next path point: {self.current_path_point}')

        return self.sub_controllers[0].forward(
            start_position=start_position,
            start_orientation=start_orientation,
            goal_position=self.current_path_point,
            forward_speed=forward_speed,
            rotation_speed=rotation_speed,
            threshold=threshold,
        )

    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        """Convert input action (in 1d array format) to joint signals to apply.

        Args:
            action (List | np.ndarray): 1-element 1d array containing
              0. path points list (List[np.ndarray])

        Returns:
            ArticulationAction: joint signals to apply.
        """
        assert len(action) == 1, 'action must contain 1 elements'
        assert len(action[0]) > 0, 'path points cannot be empty'
        start_position, start_orientation = self.robot.get_pose()
        return self.forward(
            start_position=start_position,
            start_orientation=start_orientation,
            path_points=action[0],
            forward_speed=self.forward_speed,
            rotation_speed=self.rotation_speed,
            threshold=self.threshold,
        )

    def get_obs(self) -> OrderedDict[str, Any]:
        finished = False
        total_points = len(self.path_points)
        if total_points > 0 and self.path_point_idx == total_points - 1:
            finished = self.sub_controllers[0].get_obs().get('finished', False)

        obs = {
            'current_index': self.path_point_idx,
            'current_point': self.current_path_point,
            'total_points': total_points,
            'finished': finished,
        }
        return self._make_ordered(obs)
