from collections import OrderedDict

import numpy as np

from internutopia.core.robot.articulation import IArticulation
from internutopia.core.robot.articulation_subset import ArticulationSubset
from internutopia.core.robot.rigid_body import IRigidBody
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia.core.util import log
from internutopia_extension.configs.robots.h1 import H1RobotCfg


@BaseRobot.register('H1Robot')
class H1Robot(BaseRobot):
    def __init__(self, config: H1RobotCfg, scene: IScene):
        super().__init__(config, scene)
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        log.debug(f'h1 {config.name}: position    : ' + str(self._start_position))
        log.debug(f'h1 {config.name}: orientation : ' + str(self._start_orientation))

        usd_path = config.usd_path

        log.debug(f'h1 {config.name}: usd_path         : ' + str(usd_path))
        log.debug(f'h1 {config.name}: config.prim_path : ' + str(config.prim_path))

        self._robot_scale = np.array([1.0, 1.0, 1.0])
        if config.scale is not None:
            self._robot_scale = np.array(config.scale)
        self.articulation = IArticulation.create(
            prim_path=config.prim_path,
            name=config.name,
            position=self._start_position,
            orientation=self._start_orientation,
            usd_path=usd_path,
            scale=self._robot_scale,
        )

        self.articulation.set_enabled_self_collisions(True)

    def post_reset(self):
        super().post_reset()
        self._robot_base = self._rigid_body_map[self.config.prim_path + '/pelvis']
        self._robot_right_ankle = self._rigid_body_map[self.config.prim_path + '/right_ankle_link']
        self._robot_left_ankle = self._rigid_body_map[self.config.prim_path + '/left_ankle_link']
        self.set_gains()

    def get_ankle_height(self):
        return np.min([self._robot_right_ankle.get_pose()[0][2], self._robot_left_ankle.get_pose()[0][2]])

    def get_robot_scale(self):
        return self._robot_scale

    def get_robot_base(self) -> IRigidBody:
        return self._robot_base

    def get_pose(self):
        return self._robot_base.get_pose()

    def apply_action(self, action: dict):
        """
        Args:
            action (dict): inputs for controllers.
        """
        for controller_name, controller_action in action.items():
            if controller_name not in self.controllers:
                log.warning(f'unknown controller {controller_name} in action')
                continue
            controller = self.controllers[controller_name]
            control = controller.action_to_control(controller_action)
            self.articulation.apply_action(control)

    def get_obs(self) -> OrderedDict:
        position, orientation = self._robot_base.get_pose()

        controllers_obs, sensors_obs = super()._get_controllers_and_sensors_obs()
        # custom
        obs = {'position': position, 'orientation': orientation, 'controllers': controllers_obs, 'sensors': sensors_obs}
        return self._make_ordered(obs)

    def set_gains(self):
        """
        Set default stiffness (kps) and damping (kds) for joints.
        """
        joint_names = np.array(
            [
                'left_hip_yaw_joint',
                'right_hip_yaw_joint',
                'torso_joint',
                'left_hip_roll_joint',
                'right_hip_roll_joint',
                'left_shoulder_pitch_joint',
                'right_shoulder_pitch_joint',
                'left_hip_pitch_joint',
                'right_hip_pitch_joint',
                'left_shoulder_roll_joint',
                'right_shoulder_roll_joint',
                'left_knee_joint',
                'right_knee_joint',
                'left_shoulder_yaw_joint',
                'right_shoulder_yaw_joint',
                'left_ankle_joint',
                'right_ankle_joint',
                'left_elbow_joint',
                'right_elbow_joint',
            ]
        )

        joint_subset = ArticulationSubset(self.articulation, joint_names)

        kps = np.array(
            [
                200.0,
                200.0,
                300.0,
                200.0,
                200.0,
                100.0,
                100.0,
                200.0,
                200.0,
                100.0,
                100.0,
                300.0,
                300.0,
                100.0,
                100.0,
                40.0,
                40.0,
                100.0,
                100.0,
            ]
        )
        kds = np.array([5.0, 5.0, 6.0, 5.0, 5.0, 2.0, 2.0, 5.0, 5.0, 2.0, 2.0, 6.0, 6.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0])

        self.articulation.set_gains(kps=kps, kds=kds, joint_indices=joint_subset.joint_indices)
        # VERY important!!! additional physics parameter
        self.articulation.set_solver_position_iteration_count(4)
        self.articulation.set_solver_velocity_iteration_count(0)
