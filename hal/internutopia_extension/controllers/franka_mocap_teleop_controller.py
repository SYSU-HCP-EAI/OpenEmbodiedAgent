import math
from typing import List, Tuple

import numpy as np

from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.robot.controller import BaseController
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia_extension.configs.controllers import FrankaMocapTeleopControllerCfg

DISPLACEMENT_THRESHOLD = 0.05
NUM_SPECIFIC_POSE_FRAMES = 12


@BaseController.register('FrankaMocapTeleopController')
class FrankaMocapTeleopController(BaseController):
    def __init__(self, config: FrankaMocapTeleopControllerCfg, robot: BaseRobot, scene: IScene):
        from omni.isaac.core.utils.rotations import euler_angles_to_quat

        super().__init__(config=config, robot=robot, scene=scene)

        eef_init_angel = config.origin_xyz_angle if config.origin_xyz_angle is not None else (0, 0, 0)
        self.eef_init_orientation = euler_angles_to_quat(eef_init_angel)
        self.f_rot_matrix_left = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]])
        self.f_rot_matrix_right = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])

        self.eef_init_position = config.origin_xyz if config.origin_xyz is not None else (0, 0, 0)
        self.rh_init_position = None

        self.scale = config.scale

        self.last_gripper_action = 'open'

        self.change_base_position_qualified_count = 0
        self.change_orientation_qualified_count = 0
        self.half_position_gap_scale_count = 0

        self.controlled_camera = None

    def is_finger_pinch(self, lh_bones_kps, index_1, index_2, count):
        if lh_bones_kps is None:
            return False, 0

        finger_1 = lh_bones_kps[index_1]
        finger_2 = lh_bones_kps[index_2]
        distance = math.sqrt(
            (finger_1[0] - finger_2[0]) ** 2 + (finger_1[1] - finger_2[1]) ** 2 + (finger_1[2] - finger_2[2]) ** 2
        )

        if distance > DISPLACEMENT_THRESHOLD:
            return False, 0

        if count < NUM_SPECIFIC_POSE_FRAMES:
            count += 1
            return False, count
        else:
            return True, count

    def forward(self, mocap_info) -> Tuple[ArticulationAction, bool]:
        from omni.isaac.core.utils.rotations import rot_matrix_to_quat

        lh_bones_kps = mocap_info.get('lh_bones_kps', None)
        rh_bones_kps = mocap_info.get('rh_bones_kps', None)
        rh_cur_position = mocap_info.get('rh_bones_kps_cam', np.array([[0, 0, 0]]))[0]
        rh_global_orient = mocap_info.get('rh_global_orient', np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))

        # Register and control a camera
        if self.controlled_camera is None:
            self.controlled_camera = self.robot.sensors['lh_controlled_camera'].camera
            self.camera_mover = self.robot.sensors['lh_controlled_camera'].camera_mover
            self.camera_mover.set_target_position(self.config.target_position)

        camera_position, camera_orientation = self.controlled_camera.get_pose()
        new_camera_position, new_camera_orientation = self.camera_mover(camera_position, camera_orientation, mocap_info)
        self.controlled_camera.set_pose(new_camera_position, new_camera_orientation)

        # Get the current ee_pos and rot_mat
        rmp_controller = self.sub_controllers[0]
        joint_positions = rmp_controller.rmpflow.get_internal_robot_joint_states()[0]

        if rh_bones_kps is None or joint_positions is None:
            return self.sub_controllers[0].forward(self.eef_init_position, self.eef_init_orientation)

        ee_pos, rot_mat = rmp_controller.rmpflow.get_end_effector_pose(joint_positions)

        # Initialize lh base point
        if self.rh_init_position is None:
            self.rh_init_position = rh_cur_position

        # Determine whether to reset the position and orientation base points
        change_base_position_flag, self.change_base_position_qualified_count = self.is_finger_pinch(
            lh_bones_kps, 4, 20, self.change_base_position_qualified_count
        )
        if change_base_position_flag:
            self.rh_init_position = rh_cur_position

            rh_global_orient_temp = self.f_rot_matrix_left @ rh_global_orient @ self.f_rot_matrix_right
            self.eef_init_orientation = rot_matrix_to_quat(np.array(rh_global_orient_temp))

        # Calculate eef_target_position
        r_hand_position_gap = np.round([a - b for a, b in zip(rh_cur_position, self.rh_init_position)], 3)
        r_hand_position_gap = [
            -1 * r_hand_position_gap[2],
            r_hand_position_gap[0],
            -1 * r_hand_position_gap[1],
        ] * np.array(self.scale)

        half_position_gap_scale_flag, self.half_position_gap_scale_count = self.is_finger_pinch(
            lh_bones_kps, 4, 16, self.half_position_gap_scale_count
        )
        if half_position_gap_scale_flag:
            r_hand_position_gap = r_hand_position_gap * np.array([0.5, 0.5, 0.5])

        eef_target_position = [self.eef_init_position[i] + round(r_hand_position_gap[i], 3) for i in range(3)]

        self.rh_init_position = rh_cur_position
        self.eef_init_position = eef_target_position

        # Calculate eef_target_orientation
        temp_rot_mat = self.f_rot_matrix_left @ rh_global_orient @ self.f_rot_matrix_right
        eef_target_orientation = rot_matrix_to_quat(np.array(temp_rot_mat))

        change_orientation_flag, self.change_orientation_qualified_count = self.is_finger_pinch(
            lh_bones_kps, 4, 12, self.change_orientation_qualified_count
        )
        if change_orientation_flag:
            eef_target_orientation = rot_matrix_to_quat(np.array(rot_mat))

        # arm_action
        arm_action, success = self.sub_controllers[0].forward(eef_target_position, eef_target_orientation)

        # gripper_action
        gripper_action = None

        if rh_bones_kps is not None:
            # Satisfy pinch conditions
            thumb_finger = rh_bones_kps[4]
            index_finger = rh_bones_kps[8]
            distance = math.sqrt(
                (index_finger[0] - thumb_finger[0]) ** 2
                + (index_finger[1] - thumb_finger[1]) ** 2
                + (index_finger[2] - thumb_finger[2]) ** 2
            )

            cur_gripper_action = 'close' if distance < DISPLACEMENT_THRESHOLD else 'open'

            # Satisfy state changes
            if self.last_gripper_action and cur_gripper_action != self.last_gripper_action:
                gripper_action = cur_gripper_action
                self.last_gripper_action = gripper_action

        if gripper_action is not None:
            gripper_action = self.sub_controllers[1].forward(gripper_action)
            return self.merge_arm_and_gripper_actions(arm_action, gripper_action), success
        else:
            return arm_action, success

    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        """
        Args:
            action (List | np.ndarray): 1-element array containing mocap information.
        """
        assert len(action) == 1 and type(action[0]) is dict

        result, success = self.forward(action[0])
        return result

    def merge_arm_and_gripper_actions(
        self, arm_action: ArticulationAction, gripper_action: ArticulationAction
    ) -> ArticulationAction:
        joint_positions = np.concatenate((arm_action.joint_positions, gripper_action.joint_positions[-2:]))
        joint_indices = [i for i in range(len(joint_positions))]
        return ArticulationAction(joint_positions=joint_positions, joint_indices=joint_indices)
