import math
from typing import List, Tuple

import numpy as np

from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.robot.controller import BaseController
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia_extension.configs.controllers import LayoutEditMocapControllerCfg
from internutopia_extension.controllers.layout_edit_controller.fixjoint_hand import (
    FixjointHand,
)
from internutopia_extension.controllers.layout_edit_controller.hand_control import (
    HandControl,
)
from internutopia_extension.controllers.layout_edit_controller.hand_position_control import (
    HandPositionControl,
)
from internutopia_extension.controllers.layout_edit_controller.save_asset import (
    SaveAsset,
)

DISPLACEMENT_THRESHOLD = 0.05
NUM_SPECIFIC_POSE_FRAMES = 12


@BaseController.register('LayoutEditMocapController')
class LayoutEditMocapController(BaseController):
    def __init__(self, config: LayoutEditMocapControllerCfg, robot: BaseRobot, scene: IScene):
        from omni.isaac.core.utils.rotations import euler_angles_to_quat
        from omni.isaac.core.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        super().__init__(config=config, robot=robot, scene=scene)

        hand_init_angel = config.origin_xyz_angle if config.origin_xyz_angle is not None else (0, 0, 0)
        self.rh_exe_init_orientation = euler_angles_to_quat(hand_init_angel)
        self.lh_exe_init_orientation = euler_angles_to_quat(hand_init_angel)
        self.f_rot_matrix_left = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]])
        self.f_rot_matrix_right = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])

        self.rh_exe_init_position = config.rh_origin_xyz if hasattr(config, 'rh_origin_xyz') else (0, 0, 0)
        self.lh_exe_init_position = config.lh_origin_xyz if hasattr(config, 'lh_origin_xyz') else (0, 0, 0)
        self.rh_init_position = None
        self.lh_init_position = None

        self.scale = config.scale

        self.rh_change_base_position_qualified_count = 0
        self.rh_change_orientation_qualified_count = 0
        self.rh_half_position_gap_scale_count = 0

        self.lh_change_base_position_qualified_count = 0
        self.lh_change_orientation_qualified_count = 0
        self.lh_half_position_gap_scale_count = 0

        self.controlled_camera = None

        self.stage = get_current_stage()
        if self.stage is None:
            raise RuntimeError('Stage is None')

        self.rh_path = config.right_hand_path
        self.lh_path = config.left_hand_path
        self.lh_prim_path = config.left_hand_prim_path
        self.rh_prim_path = config.right_hand_prim_path

        self.object_all = self.get_obj_in_the_scene()
        attached_hand_prim = self.stage.GetPrimAtPath(self.object_all[0])
        xform_cache = UsdGeom.XformCache()
        xform_matrix, _ = xform_cache.GetLocalTransformation(attached_hand_prim)
        scale = Gf.Vec3d(*(v.GetLength() for v in xform_matrix.ExtractRotationMatrix()))
        self.object_scale = scale
        self.hand_scale = config.hand_scale

        self.temp_right_object = None
        self.temp_left_object = None
        self.flag_hand_act = False
        self.rh_appear_flag = False
        self.rh_appear_flag_count = 0
        self.lh_appear_flag = False
        self.lh_appear_flag_count = 0
        self.delete_hand_count = 0
        self.delete_hand = False
        self.asset_save_path = config.save_asset_path
        self.robot_save_bool = config.save_robot

        self.hand_position = HandPositionControl(self.stage, self.rh_prim_path, self.lh_prim_path, self.object_scale)
        self.bind_objects = FixjointHand(self.stage, self.object_all)
        self.hand_appear = HandControl(self.stage)

    # Obtain the rigidbody object in the scene that contains a folder with many Scope files
    def get_obj_in_the_scene(self):
        from pxr import UsdGeom, UsdPhysics

        xforms = []
        # First traverse the scene and find all UsdGeom.Scope
        for prim in self.stage.Traverse():
            if prim.IsA(UsdGeom.Scope):
                # Traverse the child nodes of the Scope
                for child in prim.GetChildren():
                    if child.IsA(UsdGeom.Xform) and UsdPhysics.RigidBodyAPI(child):
                        xforms.append(child.GetPath().pathString)
        if xforms:
            return xforms
        for prims in self.stage.Traverse():
            if prims.IsA(UsdGeom.Xform):
                xforms_falg = True
                temp_prim = []
                for prim in prims.GetChildren():
                    if (
                        prim.IsA(UsdGeom.Xform)
                        and UsdPhysics.RigidBodyAPI(prim)
                        and 'scene' in prim.GetPath().pathString
                    ):
                        temp_prim.append(prim.GetPath().pathString)
                    if prim.IsA(UsdGeom.Scope) or prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Plane):
                        temp_prim = []
                        xforms_falg = False
                        break
                if xforms_falg:
                    xforms.extend(temp_prim)
        return xforms

    def check_prim(self, prim_path):
        if not prim_path.IsValid():
            raise RuntimeError(f'Prim {prim_path} is not valid')

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
        lh_bones_kps = mocap_info.get('lh_bones_kps', None)
        rh_bones_kps = mocap_info.get('rh_bones_kps', None)
        rh_cur_position = mocap_info.get('rh_bones_kps_cam', np.array([[0, 0, 0]]))[0]
        rh_global_orient = mocap_info.get('rh_global_orient', np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))

        lh_cur_position = mocap_info.get('lh_bones_kps_cam', np.array([[0, 0, 0]]))[0]
        lh_global_orient = mocap_info.get('lh_global_orient', np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
        rh_exe_target_position = None
        lh_exe_target_position = None
        rh_exe_target_orientation = None
        lh_exe_target_orientation = None
        lh_grab_action_flag = False
        rh_grab_action_flag = False
        # Register and control a certain camera
        camera = 'layout_controlled_camera'
        if self.controlled_camera is None:
            self.controlled_camera = self.robot.sensors[camera].camera
            self.camera_mover = self.robot.sensors[camera].layout_camera_mover
            self.camera_mover.set_target_position(self.config.target_position)

        camera_position, camera_orientation = self.controlled_camera.get_pose()
        new_camera_position, new_camera_orientation = self.camera_mover(camera_position, camera_orientation, mocap_info)
        self.controlled_camera.set_pose(new_camera_position, new_camera_orientation)

        if rh_bones_kps is None and lh_bones_kps is None:
            return self.sub_controllers[0].forward(self.rh_exe_init_position, self.rh_exe_init_orientation)

        # When the left thumb and ring finger pinch together, start the virtual-real interaction,
        # and then the left hand appears in the virtual scene.
        if self.rh_appear_flag is False:
            self.rh_appear_flag, self.rh_appear_flag_count = self.is_finger_pinch(
                rh_bones_kps, 4, 16, self.rh_appear_flag_count
            )
            if self.rh_appear_flag:
                self.hand_appear.add_hand_to_stage(self.rh_path, self.rh_prim_path, self.hand_scale)
        if self.lh_appear_flag is False:
            self.lh_appear_flag, self.lh_appear_flag_count = self.is_finger_pinch(
                lh_bones_kps, 4, 16, self.lh_appear_flag_count
            )
            if self.lh_appear_flag:
                self.hand_appear.add_hand_to_stage(self.lh_path, self.lh_prim_path, self.hand_scale)

        if self.rh_appear_flag is False and self.lh_appear_flag is False:
            return self.sub_controllers[0].forward(self.rh_exe_init_position, self.rh_exe_init_orientation)

        # delete hand
        if self.delete_hand is False:
            self.delete_hand, self.delete_hand_count = self.is_finger_pinch(rh_bones_kps, 4, 12, self.delete_hand_count)
            if (self.rh_appear_flag or self.lh_appear_flag) and self.delete_hand:
                self.hand_appear.delete_hand(self.lh_prim_path, self.rh_prim_path)
                self.delete_hand = False
                if self.rh_appear_flag:
                    self.rh_appear_flag = False
                if self.lh_appear_flag:
                    self.lh_appear_flag = False
                SaveAsset(self.stage).save_entire_scene(self.asset_save_path, self.robot_save_bool)

        if (
            self.lh_appear_flag
            and not (camera_position == new_camera_position).all()
            or not (camera_orientation == new_camera_orientation).all()
        ):
            lh_temp_parameters = self.change_hand_position_and_orientation(
                self.lh_init_position,
                lh_cur_position,
                lh_global_orient,
                self.lh_exe_init_position,
                lh_exe_target_position,
                self.lh_exe_init_orientation,
                lh_exe_target_orientation,
                self.lh_change_base_position_qualified_count,
                self.lh_change_orientation_qualified_count,
                self.lh_half_position_gap_scale_count,
                rh_bones_kps,
                lh_bones_kps,
            )
            (
                self.lh_init_position,
                self.lh_exe_init_position,
                lh_exe_target_position,
                self.lh_exe_init_orientation,
                lh_exe_target_orientation,
                self.lh_change_base_position_qualified_count,
                self.lh_change_orientation_qualified_count,
                self.lh_half_position_gap_scale_count,
                lh_grab_action_flag,
            ) = (
                lh_temp_parameters[0],
                lh_temp_parameters[1],
                lh_temp_parameters[2],
                lh_temp_parameters[3],
                lh_temp_parameters[4],
                lh_temp_parameters[5],
                lh_temp_parameters[6],
                lh_temp_parameters[7],
                lh_temp_parameters[8],
            )

        if self.rh_appear_flag:
            rh_temp_parameters = self.change_hand_position_and_orientation(
                self.rh_init_position,
                rh_cur_position,
                rh_global_orient,
                self.rh_exe_init_position,
                rh_exe_target_position,
                self.rh_exe_init_orientation,
                rh_exe_target_orientation,
                self.rh_change_base_position_qualified_count,
                self.rh_change_orientation_qualified_count,
                self.rh_half_position_gap_scale_count,
                lh_bones_kps,
                rh_bones_kps,
            )
            (
                self.rh_init_position,
                self.rh_exe_init_position,
                rh_exe_target_position,
                self.rh_exe_init_orientation,
                rh_exe_target_orientation,
                self.rh_change_base_position_qualified_count,
                self.rh_change_orientation_qualified_count,
                self.rh_half_position_gap_scale_count,
                rh_grab_action_flag,
            ) = (
                rh_temp_parameters[0],
                rh_temp_parameters[1],
                rh_temp_parameters[2],
                rh_temp_parameters[3],
                rh_temp_parameters[4],
                rh_temp_parameters[5],
                rh_temp_parameters[6],
                rh_temp_parameters[7],
                rh_temp_parameters[8],
            )
        if self.lh_appear_flag:
            lh_temp_parameters = self.change_hand_position_and_orientation(
                self.lh_init_position,
                lh_cur_position,
                lh_global_orient,
                self.lh_exe_init_position,
                lh_exe_target_position,
                self.lh_exe_init_orientation,
                lh_exe_target_orientation,
                self.lh_change_base_position_qualified_count,
                self.lh_change_orientation_qualified_count,
                self.lh_half_position_gap_scale_count,
                rh_bones_kps,
                lh_bones_kps,
            )
            (
                self.lh_init_position,
                self.lh_exe_init_position,
                lh_exe_target_position,
                self.lh_exe_init_orientation,
                lh_exe_target_orientation,
                self.lh_change_base_position_qualified_count,
                self.lh_change_orientation_qualified_count,
                self.lh_half_position_gap_scale_count,
                lh_grab_action_flag,
            ) = (
                lh_temp_parameters[0],
                lh_temp_parameters[1],
                lh_temp_parameters[2],
                lh_temp_parameters[3],
                lh_temp_parameters[4],
                lh_temp_parameters[5],
                lh_temp_parameters[6],
                lh_temp_parameters[7],
                lh_temp_parameters[8],
            )

        # Dynamically bind and unbind the nearest rigidbody object to the robot based on the pinching state of the hand

        # Right hand pinching and left hand not pinching, bind to the right hand
        if rh_grab_action_flag and lh_grab_action_flag is False:
            self.temp_left_object = self.bind_objects.unbind_object(self.temp_left_object, self.lh_prim_path)
            self.temp_right_object = self.bind_objects.bind_object(self.temp_right_object, self.rh_prim_path)
            self.flag_hand_act = False
        # Right hand not pinching and left hand pinching, bind to the left hand
        elif rh_grab_action_flag is False and lh_grab_action_flag:
            self.temp_right_object = self.bind_objects.unbind_object(self.temp_right_object, self.rh_prim_path)
            self.temp_left_object = self.bind_objects.bind_object(self.temp_left_object, self.lh_prim_path)
            self.flag_hand_act = False
        # When both hands pinch at the same time, the hand that pinches later binds to the object.
        elif rh_grab_action_flag and lh_grab_action_flag:
            if self.flag_hand_act is False:
                if self.temp_left_object is not None and self.temp_right_object is None:
                    self.temp_right_object = self.temp_left_object
                    self.temp_left_object = self.bind_objects.unbind_object(self.temp_left_object, self.lh_prim_path)
                    self.bind_objects.bound_object_and_robot(self.temp_right_object, self.rh_prim_path)
                    self.flag_hand_act = True
                elif self.temp_right_object is not None and self.temp_left_object is None:
                    self.temp_left_object = self.temp_right_object
                    self.temp_right_object = self.bind_objects.unbind_object(self.temp_right_object, self.rh_prim_path)
                    self.bind_objects.bound_object_and_robot(self.temp_left_object, self.lh_prim_path)
                    self.flag_hand_act = True
        else:
            self.temp_left_object = self.bind_objects.unbind_object(self.temp_left_object, self.lh_prim_path)
            self.temp_right_object = self.bind_objects.unbind_object(self.temp_right_object, self.rh_prim_path)
            self.flag_hand_act = False

        if np.all(rh_cur_position != np.array([0, 0, 0])) and self.rh_appear_flag:
            self.hand_position.update_object_hand_pose(rh_exe_target_position, rh_exe_target_orientation, hand='right')
        if np.all(lh_cur_position != np.array([0, 0, 0])) and self.lh_appear_flag:
            self.hand_position.update_object_hand_pose(lh_exe_target_position, lh_exe_target_orientation, hand='left')

        if rh_exe_target_position is None and lh_exe_target_position is None:
            return self.sub_controllers[0].forward(self.rh_exe_init_position, self.rh_exe_init_orientation)
        else:
            if rh_exe_target_position is not None:
                arm_action, success = self.sub_controllers[0].forward(rh_exe_target_position, rh_exe_target_orientation)
            else:
                arm_action, success = self.sub_controllers[0].forward(lh_exe_target_position, lh_exe_target_orientation)

        return arm_action, success

    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        """
        Args:
            action (List | np.ndarray): 0-element 1d array.
        """
        assert len(action) == 1 and type(action[0]), 'action must be empty'
        result, success = self.forward(action[0])
        return result

    def merge_arm_and_gripper_actions(
        self, arm_action: ArticulationAction, gripper_action: ArticulationAction
    ) -> ArticulationAction:
        joint_positions = np.concatenate((arm_action.joint_positions, gripper_action.joint_positions[-2:]))
        joint_indices = [i for i in range(len(joint_positions))]
        return ArticulationAction(joint_positions=joint_positions, joint_indices=joint_indices)

    def change_hand_position_and_orientation(
        self,
        hand_init_position,
        hand_cur_position,
        hand_global_orientation,
        exe_init_position,
        exe_target_position,
        exe_init_orientation,
        exe_target_orientation,
        hand_change_base_position_qualified_count,
        hand_change_orientation_qualified_count,
        hand_half_position_gap_scale_count,
        init_hand_bones_kps,
        manipulate_hand_bones_kps,
    ):
        from omni.isaac.core.utils.rotations import rot_matrix_to_quat

        # Initialize the base point of the hand
        if hand_init_position is None:
            hand_init_position = hand_cur_position

        # Determine whether to reset the base point of position and orientation
        change_base_position_flag, hand_change_base_position_qualified_count = self.is_finger_pinch(
            init_hand_bones_kps, 4, 20, hand_change_base_position_qualified_count
        )
        if change_base_position_flag:
            hand_init_position = hand_cur_position

            global_orient_temp = self.f_rot_matrix_left @ hand_global_orientation @ self.f_rot_matrix_right
            exe_init_orientation = rot_matrix_to_quat(np.array(global_orient_temp))

        # Calculate exe_target_position
        hand_position_gap = np.round([a - b for a, b in zip(hand_cur_position, hand_init_position)], 3)
        hand_position_gap = [-1 * hand_position_gap[2], hand_position_gap[0], -1 * hand_position_gap[1]] * np.array(
            self.scale
        )

        half_position_gap_scale_flag, hand_half_position_gap_scale_count = self.is_finger_pinch(
            init_hand_bones_kps, 4, 16, hand_half_position_gap_scale_count
        )
        if half_position_gap_scale_flag:
            hand_position_gap = hand_position_gap * np.array([0.5, 0.5, 0.5])

        exe_target_position = [exe_init_position[i] + round(hand_position_gap[i], 3) for i in range(3)]
        hand_init_position = hand_cur_position
        exe_init_position = exe_target_position

        # Calculate exe_target_orientation
        temp_rot_mat = self.f_rot_matrix_left @ hand_global_orientation @ self.f_rot_matrix_right
        exe_target_orientation = rot_matrix_to_quat(np.array(temp_rot_mat))
        change_orientation_flag, hand_change_orientation_qualified_count = self.is_finger_pinch(
            manipulate_hand_bones_kps, 4, 8, hand_change_orientation_qualified_count
        )

        return [
            hand_init_position,
            exe_init_position,
            exe_target_position,
            exe_init_orientation,
            exe_target_orientation,
            hand_change_base_position_qualified_count,
            hand_change_orientation_qualified_count,
            hand_half_position_gap_scale_count,
            change_orientation_flag,
        ]
