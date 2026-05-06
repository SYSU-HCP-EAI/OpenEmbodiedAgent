# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import functools
from typing import List, Optional

import numpy as np

from internutopia.core.robot.articulation import IArticulation
from internutopia.core.robot.articulation_action import ArticulationAction


def require_initialized(func):
    """Prints a warning if the underlying articulation isn't initialized and returns None. If it
    is initialized, the function is called as usual and the value returned.
    """

    @functools.wraps(func)
    def decorator(self, *args, **kwargs):
        if not self.is_initialized:
            return None
        return func(self, *args, **kwargs)

    return decorator


class ArticulationSubset:
    """A utility class for viewing a subset of the joints in a robot Articulation object.

    This class can be helpful in two ways:

    1) The order of joints returned by a robot Articulation may not match the order of joints
       expected by a function

    2) A function may only care about a subset of the joint states that are returned by a robot
       Articulation.

    Example:

        Suppose the robot Articulation returns positions [0,1,2] for joints ["A","B","C"], and
        suppose that we pass joint_names = ["B","A"].

        ArticulationSubset.get_joint_positions() -> [1,0]
        ArticulationSubset.map_to_articulation_order([1,0]) -> [0,1,None]

    Args:
        articulation (Articulation):
            An initialized Articulation object representing the simulated robot
        joint_names (List[str]):
            A list of joint names whose order determines the order of the joints returned by
            functions like get_joint_positions()"""

    def __init__(self, articulation: IArticulation, joint_names: List[str]) -> None:
        self.articulation = articulation
        self.joint_names = joint_names
        self._joint_indices = None

    @property
    def is_initialized(self):
        """Returns whether or not the underlying articulation object has been initialized."""
        return self.articulation.handles_initialized

    @property
    def num_joints(self):
        """Returns the number of joints in the articulation subset.

        Returns:
            int: Number of joints.
        """
        return len(self.joint_names)

    @property
    def joint_indices(self):
        """Property to access the joint indices for this subset.

        Returns:
            np.array: The indices of the joints defining the subset, retrieved from `_get_joint_indices`.
        """
        return self._get_joint_indices()

    @require_initialized
    def get_joint_positions(self) -> np.array:
        """Get joint positions for the joint names that were passed into this articulation view on
        initialization.  The indices of the joint positions returned correspond to the indices of
        the joint names.

        Returns:
            np.array: joint positions
        """
        return self.articulation.get_joint_positions()[self._get_joint_indices()]

    @require_initialized
    def get_joint_velocities(self) -> np.array:
        """Get joint velocities for the joint names that were passed into this articulation view on
        initialization.  The indices of the joint velocities returned correspond to the indices of
        the joint names.

        Returns:
            np.array: joint velocities
        """
        return self.articulation.get_joint_velocities()[self._get_joint_indices()]

    @require_initialized
    def get_joint_efforts(self) -> np.array:
        """Get joint efforts for the joint names that were passed into this articulation view on
        initialization.  The indices of the joint efforts returned correspond to the indices of the
        joint names.

        Returns:
            np.array: joint efforts
        """
        return self.articulation.get_joint_efforts()[self._get_joint_indices()]

    @require_initialized
    def set_joint_positions(self, positions: np.array) -> None:
        """Set the joint positions for this view.

        Args:
            positions: The position values, one for each view joint in the order specified on
            construction.
        """
        self.articulation.set_joint_positions(positions, self._get_joint_indices())

    @require_initialized
    def set_joint_velocities(self, velocities: np.array) -> None:
        """Set the joint velocities for this view.

        Args:
            velocities: The velocity values, one for each view joint in the order specified on
            construction.
        """
        self.articulation.set_joint_velocities(velocities, self._get_joint_indices())

    @require_initialized
    def set_joint_efforts(self, efforts: np.array) -> None:
        """Set the joint efforts for this view.

        Args:
            efforts: The effort values, one for each view joint in the order specified on
            construction.
        """
        self.articulation.set_joint_efforts(efforts, self._get_joint_indices())

    @require_initialized
    def map_to_articulation_order(self, joint_values: np.array) -> np.array:
        """Map a set of joint values to a format consumable by the robot Articulation.

        Args:
            joint_values (np.array): a set of joint values corresponding to the joint_names used to initialize this class.
                joint_values may be either one or two dimensional.

                If one dimensional with shape (k,): A vector will be returned with length (self.articulation.num_dof) that may
                be consumed by the robot Articulation in an ArticulationAction.

                If two dimensional with shape (N, k): A matrix will be returned with shape (N, self.articulation.num_dof) that may be
                converted to N ArticulationActions

        Returns:
            np.array: a set of joint values that is padded with None to match the shape and order expected by the robot Articulation.
        """
        joint_indices = self._get_joint_indices()

        is_single_action = joint_values.ndim == 1
        if is_single_action:
            joint_values = joint_values.reshape((1, joint_values.size))

        actions = np.full((joint_values.shape[0], self.articulation.num_dof), None)
        actions[:, joint_indices] = joint_values

        if is_single_action:
            return actions[0]
        return actions

    @require_initialized
    def make_articulation_action(self, joint_positions: np.array, joint_velocities: np.array) -> ArticulationAction:
        """Make an articulation action for only this subset's joints using the given target
        position and velocity values.

        Args:
            joint_positions: Target joint positions for this subset's joints.
            joint_velocities: Target joint velocities for this subset's joints.

        Returns: An ArticulationAction object specifying the action for this subset's joints.
        """
        return ArticulationAction(
            joint_positions=joint_positions, joint_velocities=joint_velocities, joint_indices=self._get_joint_indices()
        )

    @require_initialized
    def apply_action(
        self, joint_positions: Optional[np.array] = None, joint_velocities: Optional[np.array] = None
    ) -> None:
        """Apply the specified control actions to this views joints.

        Args:
            joint_positions: Target joint positions for this subset's joints.
            joint_velocities: Target joint velocities for this subset's joints.
        """
        self.articulation.apply_action(self.make_articulation_action(joint_positions, joint_velocities))

    @require_initialized
    def get_applied_action(self) -> ArticulationAction:
        """Retrieves the latest applied action for this subset.

        Returns: The ArticulationAction for this subset. Each commanded entry is either None or
        contains one value for each of the subset's joints. The joint_indices is set to this
        subset's joint indices.
        """
        joint_indices = self._get_joint_indices()
        action = self.articulation.get_applied_action()

        if action.joint_positions is not None:
            action.joint_positions = action.joint_positions[joint_indices]

        if action.joint_velocities is not None:
            action.joint_velocities = action.joint_velocities[joint_indices]

        if action.joint_efforts is not None:
            action.joint_efforts = action.joint_efforts[joint_indices]

        action.joint_indices = joint_indices
        return action

    def get_joint_subset_indices(self) -> np.array:
        """Accessor for the joint indices for this subset. These are the indices into the full
        articulation degrees of freedom corresponding to this subset of joints.

        Returns:
            np.array: An array of joint indices defining the subset.
        """
        return self._get_joint_indices()

    def _get_joint_indices(self):
        """Internal member which initializes the subset's joint indices from the specified names
        the first time through and returns that from then on out.
        """
        if self._joint_indices is not None:
            return self._joint_indices

        # If the member isn't set, initialize it with all the joints in the order specified on
        # construction and cache the result.The next time this method is called, it'll retrieve the
        # cached values.
        if self.articulation.handles_initialized:
            self._joint_indices = [self.articulation.get_dof_index(joint) for joint in self.joint_names]

        return self._joint_indices
