# yapf: disable
import numpy as np

tip_indices = [4, 9, 14, 19, 24]

hand2inspire_l_arm = np.array([[1, 0, 0, 0],
                               [0, 0, -1, 0],
                               [1, 1, 0, 0],
                               [0, 0, 0, 1]])

hand2inspire_r_arm = np.array([[1, 0, 0, 0],
                               [0, 0, 1, 0],
                               [1, -1, 0, 0],
                               [0, 0, 0, 1]])

hand2inspire_l_finger = np.array([[0, -1, 0, 0],
                                  [0, 0, -1, 0],
                                  [1, 0, 0, 0],
                                  [0, 0, 0, 1]])

hand2inspire_r_finger = np.array([[0, -1, 0, 0],
                                  [0, 0, -1, 0],
                                  [1, 0, 0, 0],
                                  [0, 0, 0, 1]])


grd_yup2grd_zup = np.array([[0, 0, -1, 0],
                            [-1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 0, 1]])


align_transform_l = np.array([
    [0, 0, 1],
    [0, 1, 0],
    [-1, 0, 0],
])

align_transform_r = np.array([
    [1, 0, 0],
    [0, -1, 0],
    [0, 0, -1]
]) @ np.array([
    [0, 0, 1],
    [0, -1, 0],
    [1, 0, 0],
])

align_transform_head = np.array([
    [0, -1, 0],
    [1, 0, 0],
    [0, 0, 1]
])
