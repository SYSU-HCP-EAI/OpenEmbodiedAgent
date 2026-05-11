import os
from typing import List, Optional, Union

import numpy as np

from internutopia.core.robot.articulation import IArticulation
from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.util import log
from internutopia.core.util.physics_status_util import (
    get_articulation_status,
    set_articulation_status,
)


class IsaacsimArticulation(IArticulation):
    """
    IsaacSim's implementation on `IArticulation` class.

    Args:
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

    def __init__(
        self,
        usd_path: Optional[str] = None,
        prim_path: Optional[str] = None,
        name: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        scale: Optional[np.ndarray] = None,
    ):

        if prim_path is None:
            raise ValueError("'prim_path' is required.")

        if name is None:
            raise ValueError("'name' is required.")

        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.prims import get_prim_at_path
        from omni.isaac.core.utils.stage import add_reference_to_stage

        prim = get_prim_at_path(prim_path)
        if prim.IsValid():
            if usd_path is not None:
                raise ValueError(f"Prim {prim_path} already exist, 'usd_path' should be None.")
        else:
            if usd_path is None:
                raise ValueError(f"Prim {prim_path} not exist, 'usd_path' is required.")
            add_reference_to_stage(prim_path=prim_path, usd_path=os.path.abspath(usd_path))

        self._articulation: SingleArticulation = SingleArticulation(
            prim_path=prim_path, name=name, position=position, orientation=orientation, scale=scale
        )
        super().__init__()
        self._articulation_view = self._articulation._articulation_view
        self.prim = self._articulation.prim
        self._articulation_controller = self._articulation._articulation_controller
        self.name = name
        self.status = {}

    @property
    def handles_initialized(self) -> bool:
        """See `IArticulation.handles_initialized` for documentation."""
        return self._articulation.handles_initialized

    @property
    def num_dof(self) -> int:
        """See `IArticulation.num_dof` for documentation."""
        return self._articulation.num_dof

    @property
    def num_bodies(self) -> int:
        """See `IArticulation.num_bodies` for documentation."""
        return self._articulation.num_bodies

    @property
    def dof_names(self) -> List[str]:
        """See `IArticulation.dof_names` for documentation."""
        return self._articulation.dof_names

    def get_dof_index(self, dof_name: str) -> int:
        """See `IArticulation.get_dof_index` for documentation."""
        return self._articulation.get_dof_index(dof_name=dof_name)

    def apply_action(self, action: ArticulationAction) -> None:
        """See `IArticulation.apply_action` for documentation."""
        self._articulation.apply_action(action)
        return

    def get_joint_positions(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        """See `IArticulation.get_joint_positions` for documentation."""
        return self._articulation.get_joint_positions(joint_indices=joint_indices)

    def set_joint_positions(
        self, positions: np.ndarray, joint_indices: Optional[Union[List, np.ndarray]] = None
    ) -> None:
        """See `IArticulation.set_joint_positions` for documentation."""
        self._articulation.set_joint_positions(positions=positions, joint_indices=joint_indices)

    def get_joint_velocities(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        """See `IArticulation.get_joint_velocities` for documentation."""
        return self._articulation.get_joint_velocities(joint_indices=joint_indices)

    def set_joint_velocities(
        self, velocities: np.ndarray, joint_indices: Optional[Union[List, np.ndarray]] = None
    ) -> None:
        """See `IArticulation.set_joint_velocities` for documentation."""
        self._articulation.set_joint_velocities(velocities=velocities, joint_indices=joint_indices)

    def set_enabled_self_collisions(self, flag: bool) -> None:
        """See `IArticulation.set_enabled_self_collisions` for documentation."""
        self._articulation.set_enabled_self_collisions(flag=flag)

    def set_solver_velocity_iteration_count(self, count: int) -> None:
        """See `IArticulation.set_solver_velocity_iteration_count` for documentation."""
        self._articulation.set_solver_velocity_iteration_count(count=count)

    def set_solver_position_iteration_count(self, count: int) -> None:
        """See `IArticulation.set_solver_position_iteration_count` for documentation."""
        self._articulation.set_solver_position_iteration_count(count=count)

    def set_gains(
        self,
        kps: Optional[np.ndarray] = None,
        kds: Optional[np.ndarray] = None,
        joint_indices: Optional[np.ndarray] = None,
    ) -> None:
        """See `IArticulation.set_gains` for documentation."""
        if kps is not None:
            kps = self._articulation._articulation_view._backend_utils.expand_dims(kps, 0)
        if kds is not None:
            kds = self._articulation._articulation_view._backend_utils.expand_dims(kds, 0)
        self._articulation._articulation_view.set_gains(
            kps=kps, kds=kds, save_to_usd=False, joint_indices=joint_indices
        )

    def get_angular_velocity(self) -> np.ndarray:
        """See `IArticulation.get_angular_velocity` for documentation."""
        return self._articulation.get_angular_velocity()

    def unwrap(self) -> any:
        return self._articulation

    def save_status(self):
        self.status = get_articulation_status(self._articulation)

    def restore_status(self):
        log.info(f'=============== restore info of robot {self.name} ==============')
        if self.status:
            set_articulation_status(self._articulation, self.status)

    def post_reset(self):
        self._articulation.post_reset()

    def is_valid(self) -> bool:
        return self._articulation.is_valid()
