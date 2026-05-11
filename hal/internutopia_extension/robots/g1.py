from collections import OrderedDict

import numpy as np

import internutopia.core.util.gym as gymutil
from internutopia.core.robot.articulation import IArticulation
from internutopia.core.robot.rigid_body import IRigidBody
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia.core.util import log
from internutopia_extension.configs.robots.g1 import G1RobotCfg


@BaseRobot.register('G1Robot')
class G1Robot(BaseRobot):
    def __init__(self, config: G1RobotCfg, scene: IScene):
        super().__init__(config, scene)
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        log.debug(f'G1 {config.name}: position    : ' + str(self._start_position))
        log.debug(f'G1 {config.name}: orientation : ' + str(self._start_orientation))

        usd_path = config.usd_path

        log.debug(f'G1 {config.name}: usd_path         : ' + str(usd_path))
        log.debug(f'G1 {config.name}: config.prim_path : ' + str(config.prim_path))

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

    def post_reset(self):
        super().post_reset()
        self._robot_base = self._rigid_body_map[self.config.prim_path + '/pelvis']
        self._imu_in_torso = self._rigid_body_map[self.config.prim_path + '/imu_link']
        self.set_gains()

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

        # custom
        obs = {
            'position': position,
            'orientation': orientation,
            'joint_positions': self.articulation.get_joint_positions(),
            'joint_velocities': self.articulation.get_joint_velocities(),
            'controllers': {},
            'sensors': {},
        }

        # common
        for c_obs_name, controller_obs in self.controllers.items():
            obs[c_obs_name] = controller_obs.get_obs()
        for sensor_name, sensor_obs in self.sensors.items():
            obs[sensor_name] = sensor_obs.get_data()
        return self._make_ordered(obs)

    def set_gains(self):
        """
        Set default stiffness (kps) and damping (kds) for joints.
        """
        joint_names_sim = [
            'left_hip_pitch_joint',
            'right_hip_pitch_joint',
            'waist_yaw_joint',
            'left_hip_roll_joint',
            'right_hip_roll_joint',
            'left_hip_yaw_joint',
            'right_hip_yaw_joint',
            'left_knee_joint',
            'right_knee_joint',
            'left_shoulder_pitch_joint',
            'right_shoulder_pitch_joint',
            'left_ankle_pitch_joint',
            'right_ankle_pitch_joint',
            'left_shoulder_roll_joint',
            'right_shoulder_roll_joint',
            'left_ankle_roll_joint',
            'right_ankle_roll_joint',
            'left_shoulder_yaw_joint',
            'right_shoulder_yaw_joint',
            'left_elbow_joint',
            'right_elbow_joint',
            'left_wrist_roll_joint',
            'right_wrist_roll_joint',
            'left_wrist_pitch_joint',
            'right_wrist_pitch_joint',
            'left_wrist_yaw_joint',
            'right_wrist_yaw_joint',
        ]

        joint_names_gym = [
            'left_hip_pitch_joint',
            'left_hip_roll_joint',
            'left_hip_yaw_joint',
            'left_knee_joint',
            'left_ankle_pitch_joint',
            'left_ankle_roll_joint',
            'right_hip_pitch_joint',
            'right_hip_roll_joint',
            'right_hip_yaw_joint',
            'right_knee_joint',
            'right_ankle_pitch_joint',
            'right_ankle_roll_joint',
            'waist_yaw_joint',
            'left_shoulder_pitch_joint',
            'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint',
            'left_elbow_joint',
            'left_wrist_roll_joint',
            'left_wrist_pitch_joint',
            'left_wrist_yaw_joint',
            'right_shoulder_pitch_joint',
            'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint',
            'right_elbow_joint',
            'right_wrist_roll_joint',
            'right_wrist_pitch_joint',
            'right_wrist_yaw_joint',
        ]
        self.gym_adapter = gymutil.gym_adapter(joint_names_gym, joint_names_sim)

        # num_joints = 27
        kps = self.gym_adapter.gym2sim(
            np.array(
                [
                    150.0,
                    150.0,
                    150.0,
                    300.0,
                    50.0,
                    50.0,
                    150.0,
                    150.0,
                    150.0,
                    300.0,
                    50.0,
                    50.0,
                    300.0,
                    200.0,
                    200.0,
                    200.0,
                    100.0,
                    20.0,
                    20.0,
                    20.0,
                    200.0,
                    200.0,
                    200.0,
                    100.0,
                    20.0,
                    20.0,
                    20.0,
                ]
            )
        )
        kds = self.gym_adapter.gym2sim(
            np.array(
                [
                    2.0000,
                    2.0000,
                    2.0000,
                    4.0000,
                    5.0000,
                    5.0000,
                    2.0000,
                    2.0000,
                    2.0000,
                    4.0000,
                    5.0000,
                    5.0000,
                    5.0000,
                    4.0000,
                    4.0000,
                    4.0000,
                    1.0000,
                    0.5000,
                    0.5000,
                    0.5000,
                    4.0000,
                    4.0000,
                    4.0000,
                    1.0000,
                    0.5000,
                    0.5000,
                    0.5000,
                ]
            )
        )
        self.articulation.set_gains(kps=kps, kds=kds)
