# @FileName : ik_controller.py
# @License :  (C) Copyright 2023-2024, PJLAB
# @Time :     2023/09/13 20:00:00
# yapf: disable
from collections import OrderedDict
from typing import List, Tuple

import numpy as np

from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.robot.controller import BaseController
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia_extension.configs.controllers import InverseKinematicsControllerCfg

# yapf: enable


@BaseController.register('InverseKinematicsController')
class InverseKinematicsController(BaseController):
    def __init__(self, config: InverseKinematicsControllerCfg, robot: BaseRobot, scene: IScene):

        from omni.isaac.motion_generation import (
            ArticulationKinematicsSolver,
            LulaKinematicsSolver,
        )

        class KinematicsSolver(ArticulationKinematicsSolver):
            """Kinematics Solver for robot.  This class loads a LulaKinematicsSovler object

            Args:
                robot_description_path (str): path to a robot description yaml file \
                    describing the cspace of the robot and other relevant parameters
                robot_urdf_path (str): path to a URDF file describing the robot
                end_effector_frame_name (str): The name of the end effector.
            """

            def __init__(
                self,
                robot_articulation,
                robot_description_path: str,
                robot_urdf_path: str,
                end_effector_frame_name: str,
            ):
                self._kinematics = LulaKinematicsSolver(robot_description_path, robot_urdf_path)

                ArticulationKinematicsSolver.__init__(
                    self, robot_articulation, self._kinematics, end_effector_frame_name
                )

                if hasattr(self._kinematics, 'set_max_iterations'):
                    self._kinematics.set_max_iterations(150)
                else:
                    self._kinematics.ccd_max_iterations = 150

                return

            def set_robot_base_pose(self, robot_position: np.array, robot_orientation: np.array):
                return self._kinematics.set_robot_base_pose(
                    robot_position=robot_position, robot_orientation=robot_orientation
                )

        super().__init__(config=config, robot=robot, scene=scene)
        self._kinematics_solver = KinematicsSolver(
            robot_articulation=robot.articulation,
            robot_description_path=config.robot_description_path,
            robot_urdf_path=config.robot_urdf_path,
            end_effector_frame_name=config.end_effector_frame_name,
        )
        self.joint_subset = self._kinematics_solver.get_joints_subset()
        if config.reference:
            assert config.reference in [
                'world',
                'robot',
                'arm_base',
            ], f'unknown ik controller reference {config.reference}'
            self._reference = config.reference
        else:
            self._reference = 'world'

        self.success = False
        self.last_action = None
        self.threshold = 0.01 if config.threshold is None else config.threshold

        self._robot_scale = robot.get_robot_scale()
        if self._reference == 'robot':
            # The local pose of ik base is assumed not to change during simulation for ik controlled parts.
            # However, the world pose won't change even its base link has moved for some robots like ridgeback franka,
            # so the ik base pose returned by get_local_pose may change during simulation, which is unexpected.
            # So the initial local pose of ik base is saved at first and used during the whole simulation.
            self._ik_base_local_pose = self.robot.get_robot_ik_base().get_local_pose()

    def get_ik_base_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        if self._reference == 'robot':
            ik_base_pose = self.robot.get_robot_ik_base().get_local_pose()
        elif self._reference == 'arm_base':
            # Robot base is always at the origin.
            ik_base_pose = (np.array([0, 0, 0]), np.array([1, 0, 0, 0]))
        else:
            ik_base_pose = self.robot.get_robot_ik_base().get_pose()
        return ik_base_pose

    def forward(
        self, eef_target_position: np.ndarray, eef_target_orientation: np.ndarray
    ) -> Tuple[ArticulationAction, bool]:
        self.last_action = [eef_target_position, eef_target_orientation]

        if eef_target_position is None:
            # Keep joint positions to lock pose.
            subset = self._kinematics_solver.get_joints_subset()
            return (
                subset.make_articulation_action(
                    joint_positions=subset.get_joint_positions(), joint_velocities=subset.get_joint_velocities()
                ),
                True,
            )

        ik_base_pose = self.get_ik_base_world_pose()
        self._kinematics_solver.set_robot_base_pose(
            robot_position=ik_base_pose[0] / self._robot_scale, robot_orientation=ik_base_pose[1]
        )
        return self._kinematics_solver.compute_inverse_kinematics(
            target_position=eef_target_position / self._robot_scale,
            target_orientation=eef_target_orientation,
        )

    def action_to_control(self, action: List | np.ndarray):
        """
        Args:
            action (np.ndarray): n-element 1d array containing:
              0. eef_target_position
              1. eef_target_orientation
        """
        assert len(action) == 2, 'action must contain 2 elements'
        assert self._kinematics_solver is not None, 'kinematics solver is not initialized'

        eef_target_position = None if action[0] is None else np.array(action[0])
        eef_target_orientation = None if action[1] is None else np.array(action[1])

        result, self.success = self.forward(
            eef_target_position=eef_target_position,
            eef_target_orientation=eef_target_orientation,
        )
        return result

    def get_obs(self) -> OrderedDict[str, np.ndarray]:
        """Compute the pose of the robot end effector using the simulated robot's current joint positions

        Returns:
            OrderedDict[str, np.ndarray]:
            - eef_position: eef position
            - eef_orientation: eef orientation quats
            - success: if solver converged successfully
            - finished: applied action has been finished
        """
        from omni.isaac.core.utils.numpy.rotations import rot_matrices_to_quats

        ik_base_pose = self.get_ik_base_world_pose()
        self._kinematics_solver.set_robot_base_pose(
            robot_position=ik_base_pose[0] / self._robot_scale, robot_orientation=ik_base_pose[1]
        )
        pos, ori = self._kinematics_solver.compute_end_effector_pose()

        finished = False
        if self.last_action is not None:
            if self.last_action[0] is not None:
                dist_from_goal = np.linalg.norm(pos - self.last_action[0])
                if dist_from_goal < self.threshold * self.robot.get_robot_scale()[0]:
                    finished = True

        obs = {
            'eef_position': pos * self._robot_scale,
            'eef_orientation': rot_matrices_to_quats(ori),
            'success': self.success,
            'finished': finished,
        }
        return self._make_ordered(obs)
