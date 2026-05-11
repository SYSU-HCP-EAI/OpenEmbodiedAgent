from abc import abstractmethod
from typing import Optional, Tuple

import numpy as np

from internutopia.core.config import Simulator
from internutopia.core.util.pose_mixin import PoseMixin


class ICamera(PoseMixin):
    def __init__(self):
        super().__init__()

    @abstractmethod
    def get_rgba(self) -> np.ndarray:
        """
        Returns:
            rgba (np.ndarray): (N x 4) RGBa color data for each point.
        """
        raise NotImplementedError()

    @abstractmethod
    def get_distance_to_image_plane(self) -> np.ndarray:
        """
        Returns:
            Outputs a depth map from objects to image plane of the camera. The distance_to_image_plane annotator produces a 2d array of types np.float32 with 1 channel.
        """
        raise NotImplementedError()

    @abstractmethod
    def get_bounding_box_2d_tight(self) -> np.ndarray:
        """
        Returns:
            Outputs a 'tight' 2d bounding box of each entity with semantics in the camera's viewport. Tight bounding boxes bound only the visible pixels of entities. Completely occluded entities are omitted.
        """
        raise NotImplementedError()

    @abstractmethod
    def get_camera_params(self) -> np.ndarray:
        """
        Returns:
            The Camera Parameters annotator returns the camera details for the camera corresponding to the render product to which the annotator is attached.
        """
        raise NotImplementedError()

    @abstractmethod
    def cleanup(self) -> None:
        """
        Operations that need to be cleaned up before switching scenes (or resetting)
        """
        raise NotImplementedError()

    @classmethod
    def create(
        cls,
        simulator_type: str = Simulator.ISAACSIM.value,
        name: str = 'camera',
        prim_path: Optional[str] = None,
        rgba: Optional[bool] = True,
        distance_to_image_plane: Optional[bool] = False,
        bounding_box_2d_tight: Optional[bool] = False,
        camera_params: Optional[bool] = False,
        resolution: Optional[Tuple[int, int]] = None,
        position: Optional[Tuple[float, float, float]] = None,
        translation: Optional[Tuple[float, float, float]] = None,
        orientation: Optional[Tuple[float, float, float, float]] = None,
    ) -> 'ICamera':
        """Factory method to create ICamera instances based on simulator_type.

        Args:
            simulator_type (str): simulator type.
            name (str): The unique identifier for the camera.
            prim_path (Optional[str]): The primary path associated with the camera.
            rgba (Optional[bool], default=False): Whether to get rgba from the camera or not.
            distance_to_image_plane (Optional[bool], default=False): Whether to get distance_to_image_plane from the camera or not.
            bounding_box_2d_tight (Optional[bool], default=False): Whether to get bounding_box_2d_tight from the camera or not.
            camera_params (Optional[bool], default=False): Whether to get camera_params from the camera or not.
            resolution (Optional[Tuple[int, int]], optional): resolution of the camera (width, height). Defaults to None.
            position (Optional[Tuple[float, float, float]], optional): position in the world frame of the prim. shape is (3, ). Defaults to None, which means left unchanged.
            translation (Optional[Tuple[float, float, float]], optional): translation in the local frame of the prim (with respect to its parent prim). shape is (3, ). Defaults to None, which means left unchanged.
            orientation (Optional[Tuple[float, float, float, float]], optional): quaternion orientation in the world/ local frame of the prim (depends if translation or position is specified). quaternion is scalar-first (w, x, y, z). shape is (4, ). Defaults to None, which means left unchanged.
        """
        if simulator_type == Simulator.ISAACSIM.value:
            from internutopia.core.sensor.isaacsim.camera import IsaacsimCamera

            return IsaacsimCamera(
                name=name,
                prim_path=prim_path,
                rgba=rgba,
                distance_to_image_plane=distance_to_image_plane,
                bounding_box_2d_tight=bounding_box_2d_tight,
                camera_params=camera_params,
                resolution=resolution,
                position=position,
                translation=translation,
                orientation=orientation,
            )
        else:
            raise ValueError(f'Invalid simulator_type: {simulator_type}')
