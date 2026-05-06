from typing import List, Optional, Sequence, Tuple

import numpy as np


class PoseMixin:
    """
    Mixin class for managing poses of objects in a simulation environment.

    This class provides methods to get and set the pose of an object with respect to different frames
    (local, world, and environment). It also handles the offset of the object's position within its environment.

    Attributes:
        env_offset_map (dict): A dictionary mapping environment IDs to their respective offsets.
    """

    env_offset_map = {}

    def __init__(self):
        self.env_id = self.get_env_id()
        self.offset = self.get_offset()

    def unwrap(self):
        """
        Unwraps the object to its base form.
        """
        raise NotImplementedError('need to implement this method ')

    def get_env_id(self) -> str:
        """
        Extracts and returns the environment ID from the prim path of an unwrapped object.

        Raises:
            RuntimeError: If the environment ID cannot be found in the prim path.

        Returns:
            str: The extracted environment ID.
        """
        unwrapped = self.unwrap()
        if '/World/env_' not in unwrapped.prim_path:
            raise RuntimeError(f'cannot find env id in prim path of {unwrapped.name}')

        p_list = unwrapped.prim_path.split('/')
        _prim_path = '/'.join(p_list[0:3])
        return _prim_path.split('_')[-1]

    def get_offset(self) -> List[float]:
        """
        Returns the offset for the current environment ID from the env_offset_map.

        Raises:
            RuntimeError: If the environment ID is not found in the env_offset_map.

        Returns:
            List[float]: The list of offsets for the given environment ID.
        """
        if self.env_id in PoseMixin.env_offset_map:
            return [i for i in PoseMixin.env_offset_map[self.env_id]]
        else:
            raise RuntimeError(f'{self.env_id} is not in env_offset_map')

    def get_local_pose(self):
        """
        Get prim's pose with respect to the local frame (the prim's parent frame)

        Returns:
            Tuple[np.ndarray, np.ndarray]: first index is the position in the local frame (with shape (3, )).
            Second index is quaternion orientation (with shape (4, )) in the local frame
        """
        return self.unwrap().get_local_pose()

    def set_local_pose(
        self, translation: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None
    ) -> None:
        """
        Set prim's pose with respect to the local frame (the prim's parent frame).

        Args:
            translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                          (with respect to its parent prim). shape is (3, ).
                                                          Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the local frame of the prim.
                                                          quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                          Defaults to None, which means left unchanged.
        """
        self.unwrap().set_local_pose(translation, orientation)

    def get_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get prim's pose with respect to the world's frame

        Returns:
            Tuple[np.ndarray, np.ndarray]: first index is the position in the world frame (with shape (3, )).
            Second index is quaternion orientation (with shape (4, )) in the world frame
        """
        return self.unwrap().get_world_pose()

    def set_world_pose(self, position: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None):
        """Ses prim's pose with respect to the world's frame

        .. warning::

            This method will change (teleport) the prim pose immediately to the indicated value

        Args:
            position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                       Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the world frame of the prim.
                                                          quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                          Defaults to None, which means left unchanged.

        .. hint::

            This method belongs to the methods used to set the prim state
        """
        self.unwrap().set_world_pose(position, orientation)

    def get_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get prim's pose with respect to the env's frame (env in internutopia)

        Returns:
            Tuple[np.ndarray, np.ndarray]: first index is the position in the world frame (with shape (3, )).
            Second index is quaternion orientation (with shape (4, )) in the world frame
        """
        unwrapped = self.unwrap()
        position, orientation = unwrapped.get_world_pose()
        _pose = [0.0, 0.0, 0.0]
        if self.offset is not None:
            for idx, i in enumerate(position):
                _pose[idx] = i - self.offset[idx]
        return np.array(_pose), orientation

    def set_pose(
        self,
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
    ) -> None:
        """
        Ses prim's pose with respect to the env's frame (env in internutopia)

        Args:
            position (Optional[Sequence[float]], optional): position in the env frame of the prim. shape is (3, ).
                                                       Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the world frame of the prim.
                                                          quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                          Defaults to None, which means left unchanged.
        """
        _position = [0.0, 0.0, 0.0]

        for idx, i in enumerate(position):
            _position[idx] = i - self.offset[idx]

        self.unwrap().set_world_pose(position, orientation)
