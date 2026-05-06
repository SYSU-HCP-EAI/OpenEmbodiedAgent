from abc import abstractmethod
from typing import List, Optional, Union

import numpy as np

from internutopia.core.config import Simulator
from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.util.pose_mixin import PoseMixin


class IArticulation(PoseMixin):
    """
    Represents a kinematic chain of rigid bodies connected by joints.

    Enables modeling of multi-link robotic mechanisms with realistic motion and constraints.

    Note:
        Within Isaac Sim's implementation, it is exclusively supported for single-DoF joints
        and methods prefixed with set_joint_***() are functionally equivalent to their set_dof_***() counterparts.

        While Genesis natively supports multi-DoF joints (where a single joint may govern multiple degrees of freedom),
        we adhere to a one-to-one joint-to-DoF mapping convention for cross-simulator compatibility.
    """

    def __init__(self):
        super().__init__()

    @property
    @abstractmethod
    def handles_initialized(self) -> bool:
        """Check if articulation handler is initialized.

        Returns:
            bool: whether the handler was initialized
        """
        raise NotImplementedError()

    @property
    @abstractmethod
    def num_dof(self) -> int:
        """Number of DOF of the articulation

        Returns:
            int: amount of DOFs
        """
        raise NotImplementedError()

    @property
    @abstractmethod
    def num_bodies(self) -> int:
        """Number of articulation links.

        Returns:
            int: number of links
        """
        raise NotImplementedError()

    @property
    def dof_names(self) -> List[str]:
        """List of prim names for each DOF.

        Returns:
            list(string): prim names
        """
        raise NotImplementedError()

    @abstractmethod
    def get_dof_index(self, dof_name: str) -> int:
        """Get a DOF index given its name.

        Args:
            dof_name (str): name of the DOF

        Returns:
            int: DOF index
        """
        raise NotImplementedError()

    @abstractmethod
    def apply_action(self, action: ArticulationAction) -> None:
        """Apply joint positions, velocities and/or efforts to control an articulation.

        Args:
            action (ArticulationAction): actions to be applied for next physics step.

        .. hint::

            High stiffness makes the joints snap faster and harder to the desired target,
            and higher damping smoothes but also slows down the joint's movement to target

            * For position control, set relatively high stiffness and low damping (to reduce vibrations)
            * For velocity control, stiffness must be set to zero with a non-zero damping
            * For effort control, stiffness and damping must be set to zero
        """
        raise NotImplementedError()

    @abstractmethod
    def get_joint_positions(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        """Get the articulation joint positions.

        Args:
            joint_indices (Optional[Union[List, np.ndarray]], optional): indices to specify which joints to read.
                                                                         Defaults to None (all joints)

        Returns:
            np.ndarray: all or selected articulation joint positions
        """
        raise NotImplementedError()

    @abstractmethod
    def set_joint_positions(
        self, positions: np.ndarray, joint_indices: Optional[Union[List, np.ndarray]] = None
    ) -> None:
        """Set the articulation joint positions.

        Args:
            positions (np.ndarray): articulation joint positions
            joint_indices (Optional[Union[list, np.ndarray]], optional): indices to specify which joints to manipulate.                                                                      Defaults to None (all joints)
        """
        raise NotImplementedError()

    @abstractmethod
    def get_joint_velocities(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        """Get the articulation joint velocities.

        Args:
            joint_indices (Optional[Union[List, np.ndarray]], optional): indices to specify which joints to read.
                                                                         Defaults to None (all joints)

        Returns:
            np.ndarray: all or selected articulation joint velocities
        """
        raise NotImplementedError()

    @abstractmethod
    def set_joint_velocities(
        self, velocities: np.ndarray, joint_indices: Optional[Union[List, np.ndarray]] = None
    ) -> None:
        """Set the articulation joint velocities.

        Args:
            velocities (np.ndarray): articulation joint velocities
            joint_indices (Optional[Union[list, np.ndarray]], optional): indices to specify which joints to manipulate.
                                                                         Defaults to None (all joints)
        """
        raise NotImplementedError()

    @abstractmethod
    def set_enabled_self_collisions(self, flag: bool) -> None:
        """Set the enable self collisions flag.

        Args:
            flag (bool): whether to enable self collisions
        """
        raise NotImplementedError()

    @abstractmethod
    def set_solver_velocity_iteration_count(self, count: int) -> None:
        """Set the solver (velocity) iteration count for the articulation.

        Args:
            count (int): velocity iteration count
        """
        raise NotImplementedError()

    def set_solver_position_iteration_count(self, count: int) -> None:
        """Set the solver (position) iteration count for the articulation.

        Args:
            count (int): position iteration count
        """
        raise NotImplementedError()

    def set_gains(
        self,
        kps: Optional[np.ndarray] = None,
        kds: Optional[np.ndarray] = None,
        joint_indices: Optional[np.ndarray] = None,
    ) -> None:
        """Set the implicit PD controller's Kps (stiffnesses) and Kds (dampings) of the articulation.

        Args:
            kps (Optional[np.ndarray], optional): stiffness of the drives. shape is (M, K). Defaults to None.
            kds (Optional[np.ndarray], optional): damping of the drives. shape is (M, K).. Defaults to None.
            joint_indices (Optional[np.ndarray], optional): joint indices to specify which joints
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs)
        """
        raise NotImplementedError()

    @abstractmethod
    def get_angular_velocity(self) -> np.ndarray:
        """Get the articulation's root angular velocity.

        Returns:
            np.ndarray: 3D angular velocity vector. Shape (3,)
        """
        raise NotImplementedError()

    @classmethod
    def create(
        cls,
        simulator_type: str = Simulator.ISAACSIM.value,
        usd_path: Optional[str] = None,
        prim_path: Optional[str] = None,
        name: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
    ) -> 'IArticulation':
        """
        Factory method to create IArticulation instances based on simulator_type.

        Args:
            simulator_type (str): simulator type.
            usd_path (str, optional): The file path to the USD containing the robot definition.
            prim_path (str, optional): prim path of the Prim to encapsulate or create.
            name (str, optional): shortname to be used as a key by Scene class.
                                    Note: needs to be unique if the object is added to the Scene.
            position (Optional[np.ndarray], optional): position in the world frame of the prim. Shape is (3, ).
                                                        Defaults to None, which means left unchanged.
            orientation (Optional[np.ndarray], optional): quaternion orientation in the world/ local frame of the prim
                                                            quaternion is scalar-first (w, x, y, z). Shape is (4, ).
                                                            Defaults to None, which means left unchanged.
            scale (Optional[np.ndarray], optional): local scale to be applied to the prim's dimensions. Shape is (3, ).
                                                    Defaults to None, which means left unchanged.
        """

        if simulator_type == Simulator.ISAACSIM.value:
            from internutopia.core.robot.isaacsim.articulation import (
                IsaacsimArticulation,
            )

            return IsaacsimArticulation(
                usd_path=usd_path,
                prim_path=prim_path,
                name=name,
                position=position,
                orientation=orientation,
                scale=scale,
            )
        else:
            raise ValueError(f'Invalid simulator_type: {simulator_type}')
