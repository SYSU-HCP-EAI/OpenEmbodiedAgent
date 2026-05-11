from abc import abstractmethod
from typing import Optional

import numpy as np

from internutopia.core.config import Simulator
from internutopia.core.robot.articulation import IArticulation
from internutopia.core.util.pose_mixin import PoseMixin


class IRigidBody(PoseMixin):
    """
    Represents a rigid body in the simulation environment.

    A RigidBody is a solid, non-deformable physical entity that can either be:
        - A robot link within an articulation
        - An independent object such as a box.

    .. note::
    if rigid body is an independent object, it possesses six DoFs, corresponding to translational motion along the x, y, z axes and rotational motion and
    its motion like linear velocity can be directly controlled.
    However, if rigidBody is a rigid link within an articulation, its motion must be regulated by controlling the DoFs of the articulation.
    """

    def __init__(self):
        super().__init__()

    @abstractmethod
    def set_linear_velocity(self, velocity: np.ndarray):
        """Set the linear velocity of the rigid body in stage

        Args:
            velocity (np.ndarray): linear velocity to set the rigid prim to. Shape (3,).
        """
        raise NotImplementedError()

    @abstractmethod
    def get_linear_velocity(self) -> np.ndarray:
        """Get the linear velocity of the rigid body

        Returns:
            np.ndarray: current linear velocity of the the rigid prim. Shape (3,).
        """
        raise NotImplementedError()

    @abstractmethod
    def get_angular_velocity(self):
        """Get the angular velocity of the rigid body

        Returns:
            np.ndarray: current angular velocity of the the rigid prim. Shape (3,).

        """
        raise NotImplementedError()

    @abstractmethod
    def set_mass(self, mass: float) -> None:
        """Set the mass of the rigid body

        Args:
            mass (float): mass of the rigid body in kg.
        """
        raise NotImplementedError()

    @abstractmethod
    def get_mass(self) -> float:
        """Get the mass of the rigid body

        Returns:
            float: mass of the rigid body in kg.
        """
        raise NotImplementedError()

    @classmethod
    def create(
        cls,
        simulator_type: str = Simulator.ISAACSIM.value,
        prim_path: Optional[str] = None,
        usd_path: Optional[str] = None,
        name: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
        visible: Optional[bool] = None,
        mass: Optional[float] = None,
        linear_velocity: Optional[np.ndarray] = None,
        angular_velocity: Optional[np.ndarray] = None,
        owner_articulation: Optional[IArticulation] = None,
    ) -> 'IRigidBody':
        """
        Factory method to create IRigidBody instances based on simulator_type.

        Args:
            simulator_type (str): simulator type.
            prim_path (str, optional): prim path of the Prim to encapsulate or create.
            usd_path (str, optional):  The file path containing the rigid body definition.
            name (str, optional): shortname to be used as a key by Scene class.
                                Note: needs to be unique if the object is added to the Scene.
            position (Optional[np.ndarray], optional): position in the world frame of the prim. shape is (3, ).
                                                    Defaults to None, which means left unchanged.
            translation (Optional[np.ndarray], optional): translation in the local frame of the prim
                                                        (with respect to its parent prim). shape is (3, ).
                                                        Defaults to None, which means left unchanged.
            orientation (Optional[np.ndarray], optional): quaternion orientation in the world/ local frame of the prim
                                                        (depends if translation or position is specified).
                                                        quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                        Defaults to None, which means left unchanged.
            scale (Optional[np.ndarray], optional): local scale to be applied to the prim's dimensions. shape is (3, ).
                                                    Defaults to None, which means left unchanged.
            visible (bool, optional): set to false for an invisible prim in the stage while rendering. Defaults to True.
            mass (Optional[float], optional): mass in kg. Defaults to None.
            linear_velocity (Optional[np.ndarray], optional): linear velocity in the world frame. Defaults to None.
            angular_velocity (Optional[np.ndarray], optional): angular velocity in the world frame. Defaults to None.
            owner_articulation (Optional[IArticulation], optional): The articulation to which the rigid body belongs if rigid body represents a link. Defaults to None.
        """
        if simulator_type == Simulator.ISAACSIM.value:
            from internutopia.core.robot.isaacsim.rigid_body import IsaacsimRigidBody

            if prim_path is None:
                raise ValueError("'prim_path' is required.")

            if name is None:
                raise ValueError("'name' is required.")

            return IsaacsimRigidBody(
                prim_path=prim_path,
                usd_path=usd_path,
                name=name,
                position=position,
                translation=translation,
                orientation=orientation,
                scale=scale,
                visible=visible,
                mass=mass,
                linear_velocity=linear_velocity,
                angular_velocity=angular_velocity,
                owner_articulation=owner_articulation,
            )
        else:
            raise ValueError(f'Invalid simulator_type: {simulator_type}')
