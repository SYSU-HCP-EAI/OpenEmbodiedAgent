# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import List, Optional, Union

import numpy as np


class ArticulationAction:
    """
    Represents control commands for an articulation's joints.

    Args:
        joint_positions (Optional[Union[List, np.ndarray]], optional): List of joint positions or None. Defaults to None.
        joint_velocities (Optional[Union[List, np.ndarray]], optional): List of joint velocities or None. Defaults to None.
        joint_efforts (Optional[Union[List, np.ndarray]], optional): List of joint efforts or None. Defaults to None.
        joint_indices (Optional[Union[List, np.ndarray]], optional): List of joint indices or None. Defaults to None.
    """

    def __init__(
        self,
        joint_positions: Optional[Union[List, np.ndarray]] = None,
        joint_velocities: Optional[Union[List, np.ndarray]] = None,
        joint_efforts: Optional[Union[List, np.ndarray]] = None,
        joint_indices: Optional[Union[List, np.ndarray]] = None,
    ) -> None:
        self.joint_positions = joint_positions
        self.joint_velocities = joint_velocities
        self.joint_efforts = joint_efforts
        self.joint_indices = joint_indices

    def get_dof_action(self, index: int) -> dict:
        """
        Retrieve the action for a specific degree of freedom (DoF) at the given index.

        Args:
            index (int): index of degree of freedom (DoF)

        Returns:
            dict: the action for a specific degree of freedom (DoF) at the given index.
        """
        if self.joint_efforts is not None and self.joint_efforts[index] is not None:
            return {'effort': self.joint_efforts[index]}
        else:
            dof_action = dict()
            if self.joint_velocities is not None and self.joint_velocities[index] is not None:
                dof_action['velocity'] = self.joint_velocities[index]
            if self.joint_positions is not None and self.joint_positions[index] is not None:
                dof_action['position'] = self.joint_positions[index]
            return dof_action

    def get_dict(self) -> dict:
        """
        Convert the action to a dictionary.

        Returns:
            dict: A dictionary representation of the action with keys:
                - joint_positions (Optional[List | None]): List of joint positions or None.
                - joint_velocities (Optional[List | None]): List of joint velocities or None.
                - joint_efforts (Optional[List | None]): List of joint efforts or None.
        """
        result = dict()
        if self.joint_positions is not None:
            if isinstance(self.joint_positions, np.ndarray):
                result['joint_positions'] = self.joint_positions.tolist()
            else:
                result['joint_positions'] = self.joint_positions
        else:
            result['joint_positions'] = None
        if self.joint_velocities is not None:
            if isinstance(self.joint_velocities, np.ndarray):
                result['joint_velocities'] = self.joint_velocities.tolist()
            else:
                result['joint_velocities'] = self.joint_velocities
        else:
            result['joint_velocities'] = None
        if self.joint_efforts is not None:
            if isinstance(self.joint_efforts, np.ndarray):
                result['joint_efforts'] = self.joint_efforts.tolist()
            else:
                result['joint_efforts'] = self.joint_efforts
        else:
            result['joint_efforts'] = None
        return result

    def __str__(self) -> str:
        return str(self.get_dict())

    def get_length(self) -> Optional[int]:
        """
        Get the number of joints in the action.

        Returns:
            Optional[int]: the number of joints
        """
        size = None
        if self.joint_positions is not None:
            if size is None:
                size = 0
            if isinstance(self.joint_positions, np.ndarray):
                size = max(size, self.joint_positions.shape[0])
            else:
                size = max(size, len(self.joint_positions))
        if self.joint_velocities is not None:
            if size is None:
                size = 0
            if isinstance(self.joint_velocities, np.ndarray):
                size = max(size, self.joint_velocities.shape[0])
            else:
                size = max(size, len(self.joint_velocities))
        if self.joint_efforts is not None:
            if size is None:
                size = 0
            if isinstance(self.joint_efforts, np.ndarray):
                size = max(size, self.joint_efforts.shape[0])
            else:
                size = max(size, len(self.joint_efforts))
        return size
