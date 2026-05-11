from typing import List

import numpy as np

from internutopia.core.robot.controller import BaseController
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia_extension.configs.controllers import RMPFlowControllerCfg


@BaseController.register('RMPFlowController')
class RMPFlowController(BaseController):
    def __init__(self, config: RMPFlowControllerCfg, robot: BaseRobot, scene: IScene):
        from omni.isaac.motion_generation import ArticulationMotionPolicy, RmpFlow

        super().__init__(config=config, robot=robot, scene=scene)
        self.rmpflow = RmpFlow(
            robot_description_path=config.robot_description_path,
            urdf_path=config.robot_urdf_path,
            rmpflow_config_path=config.rmpflow_config_path,
            end_effector_frame_name=config.end_effector_frame_name,
            maximum_substep_size=1 / 120,
        )
        self._articulation_rmpflow = ArticulationMotionPolicy(
            robot.articulation, self.rmpflow, default_physics_dt=1 / 120
        )

        self.success = False

    def forward(self, eef_target_position: np.ndarray, eef_target_orientation: np.ndarray):
        self.rmpflow.set_end_effector_target(np.asarray(eef_target_position), eef_target_orientation)

        return self._articulation_rmpflow.get_next_articulation_action(), True

    def action_to_control(self, action: List | np.ndarray):
        """
        Args:
            action (np.ndarray): n-element 1d array containing:
              0. eef_target_position
              1. eef_target_orientation
        """
        assert len(action) == 2, 'action must contain 2 elements'

        eef_target_position = None if action[0] is None else np.array(action[0])
        eef_target_orientation = None if action[1] is None else np.array(action[1])

        result, self.success = self.forward(
            eef_target_position=eef_target_position,
            eef_target_orientation=eef_target_orientation,
        )
        return result
