from abc import ABC, abstractmethod
from typing import List

from internutopia.core.config import Simulator, TaskCfg
from internutopia.core.robot.rigid_body import IRigidBody


class IScene(ABC):
    """Abstract base class for all Scene classes."""

    def __init__(self):
        return

    @abstractmethod
    def load(self, task_config: TaskCfg, env_id: int, env_offset: List[float]):
        """Load the scene."""
        raise NotImplementedError()

    @abstractmethod
    def add(self, target: any):
        """Add an object to the scene."""
        raise NotImplementedError()

    @abstractmethod
    def remove(self, target: any, registry_only: bool = False):
        """Remove an object from the scene."""
        raise NotImplementedError()

    @abstractmethod
    def object_exists(self, target: any) -> bool:
        """Check if an object exists in the scene registry."""
        raise NotImplementedError()

    @abstractmethod
    def get(self, target: any) -> IRigidBody:
        """Get an object from the scene."""
        raise NotImplementedError()

    @abstractmethod
    def unwrap(self):
        """Unwraps the scene to its base form."""
        raise NotImplementedError()

    @classmethod
    def create(cls, simulator_type: str = Simulator.ISAACSIM.value) -> 'IScene':
        """Factory method to create IScene instances based on simulator_type."""
        if simulator_type == Simulator.ISAACSIM.value:
            from internutopia.core.scene.isaacsim.scene import IsaacsimScene

            return IsaacsimScene()
        else:
            raise ValueError(f'Invalid simulator_type: {simulator_type}')
