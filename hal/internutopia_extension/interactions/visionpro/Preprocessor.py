import numpy as np

from .constants_vuer import (
    align_transform_l,
    align_transform_r,
    grd_yup2grd_zup,
    hand2inspire_l_arm,
    hand2inspire_l_finger,
    hand2inspire_r_arm,
    hand2inspire_r_finger,
)
from .motion_utils import fast_mat_inv, mat_update


class VuerPreprocessor:
    def __init__(self):
        # by default, head at (0.2, 0, 1.5) of world frame, which is also z-up frame;
        # right wrist at (0.5, -0.5, 1.) left wrist at (0.5, 0.5, 1.)
        # yapf: disable
        self.vuer_head_mat = np.array([[1, 0, 0, 0],
                                       [0, 1, 0, 1.5],
                                       [0, 0, 1, -0.2],
                                       [0, 0, 0, 1]])
        self.vuer_right_wrist_mat = np.array([[1, 0, 0, 0.2],
                                              [0, 1, 0, 1],
                                              [0, 0, 1, -0.5],
                                              [0, 0, 0, 1]])
        self.vuer_left_wrist_mat = np.array([[1, 0, 0, -0.2],
                                             [0, 1, 0, 1],
                                             [0, 0, 1, -0.5],
                                             [0, 0, 0, 1]])
        # yapf: enable

    def process(self, tv):
        # first, y-up z-forward x-left axis
        # head at 1.5 height
        self.vuer_head_mat = mat_update(self.vuer_head_mat, tv.head_matrix.copy())
        self.vuer_right_wrist_mat = mat_update(self.vuer_right_wrist_mat, tv.right_hand.copy())
        self.vuer_left_wrist_mat = mat_update(self.vuer_left_wrist_mat, tv.left_hand.copy())
        # change of basis
        head_mat = grd_yup2grd_zup @ self.vuer_head_mat @ fast_mat_inv(grd_yup2grd_zup)
        right_wrist_mat = grd_yup2grd_zup @ self.vuer_right_wrist_mat @ fast_mat_inv(grd_yup2grd_zup)
        left_wrist_mat = grd_yup2grd_zup @ self.vuer_left_wrist_mat @ fast_mat_inv(grd_yup2grd_zup)

        rel_left_wrist_mat = left_wrist_mat @ hand2inspire_l_arm
        rel_left_wrist_mat[0:3, 3] = (
            rel_left_wrist_mat[0:3, 3] - head_mat[0:3, 3]
        )  # relative position in world frame; orientation is inspire-related

        rel_right_wrist_mat = right_wrist_mat @ hand2inspire_r_arm  # wTr = wTh @ hTr
        rel_right_wrist_mat[0:3, 3] = rel_right_wrist_mat[0:3, 3] - head_mat[0:3, 3]

        # homogeneous
        left_fingers = np.concatenate([tv.left_landmarks.copy().T, np.ones((1, tv.left_landmarks.shape[0]))])
        right_fingers = np.concatenate([tv.right_landmarks.copy().T, np.ones((1, tv.right_landmarks.shape[0]))])

        # change of basis
        left_fingers = grd_yup2grd_zup @ left_fingers
        right_fingers = grd_yup2grd_zup @ right_fingers

        rel_left_fingers = fast_mat_inv(left_wrist_mat) @ left_fingers
        rel_right_fingers = fast_mat_inv(right_wrist_mat) @ right_fingers
        rel_left_fingers = (hand2inspire_l_finger.T @ rel_left_fingers)[0:3, :].T
        rel_right_fingers = (hand2inspire_r_finger.T @ rel_right_fingers)[0:3, :].T

        # it should be mat of inspire dexterous hand

        rel_left_wrist_mat[:3, :3] = rel_left_wrist_mat[:3, :3] @ align_transform_l.T
        rel_right_wrist_mat[:3, :3] = rel_right_wrist_mat[:3, :3] @ align_transform_r.T
        # head_mat[:3, :3] = align_transform_head @ head_mat[:3, :3]
        return head_mat, rel_left_wrist_mat, rel_right_wrist_mat, rel_left_fingers, rel_right_fingers

    def get_hand_gesture(self, tv):
        self.vuer_right_wrist_mat = mat_update(self.vuer_right_wrist_mat, tv.right_hand.copy())
        self.vuer_left_wrist_mat = mat_update(self.vuer_left_wrist_mat, tv.left_hand.copy())

        # change of basis
        right_wrist_mat = grd_yup2grd_zup @ self.vuer_right_wrist_mat @ fast_mat_inv(grd_yup2grd_zup)
        left_wrist_mat = grd_yup2grd_zup @ self.vuer_left_wrist_mat @ fast_mat_inv(grd_yup2grd_zup)

        left_fingers = np.concatenate([tv.left_landmarks.copy().T, np.ones((1, tv.left_landmarks.shape[0]))])
        right_fingers = np.concatenate([tv.right_landmarks.copy().T, np.ones((1, tv.right_landmarks.shape[0]))])

        # change of basis
        left_fingers = grd_yup2grd_zup @ left_fingers
        right_fingers = grd_yup2grd_zup @ right_fingers

        rel_left_fingers = fast_mat_inv(left_wrist_mat) @ left_fingers
        rel_right_fingers = fast_mat_inv(right_wrist_mat) @ right_fingers
        rel_left_fingers = (hand2inspire_l_finger.T @ rel_left_fingers)[0:3, :].T
        rel_right_fingers = (hand2inspire_r_finger.T @ rel_right_fingers)[0:3, :].T
        all_fingers = np.concatenate([rel_left_fingers, rel_right_fingers], axis=0)

        return all_fingers
