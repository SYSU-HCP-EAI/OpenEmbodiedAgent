from typing import Optional, Tuple

from internutopia.core.config.robot import BaseCfg


class ObjectCfg(BaseCfg):
    """
    Represents a customizable object within a scene, with support for physics and different object types.

    This class defines an object that can be placed within a virtual environment, providing essential properties for both graphical representation and physical simulation. It supports varying object types with specific parameters, ensuring flexibility and customizability for different use cases.

    Attributes:
        name (str): The unique identifier for the object.
        prim_path (str): The path indicating the object's position within the scene hierarchy.
        position (Tuple[float, float, float], optional): The 3D coordinates of the object's position. Defaults to (0.0, 0.0, 0.0).
        orientation (Tuple[float, float, float, float], optional): The quaternion representation of the object's orientation. Defaults to (1.0, 0.0, 0.0, 0.0).
        scale (Tuple[float, float, float], optional): Scaling factors along each axis. Defaults to (1.0, 1.0, 1.0).
        type (str): Specifies the type of the object.
    """

    name: str
    prim_path: str
    position: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)
    orientation: Optional[Tuple[float, float, float, float]] = (1.0, 0.0, 0.0, 0.0)
    scale: Optional[Tuple[float, float, float]] = (1.0, 1.0, 1.0)
    type: str
