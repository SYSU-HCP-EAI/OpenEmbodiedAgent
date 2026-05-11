from typing import List, Optional, Tuple

from internutopia.core.config.object import ObjectCfg


class DynamicCubeCfg(ObjectCfg):
    type: Optional[str] = 'DynamicCube'
    color: Optional[Tuple[float, float, float]] = None
    mass: Optional[float] = None
    density: Optional[float] = None
    static_friction: Optional[float] = None
    dynamic_friction: Optional[float] = None
    restitution: Optional[float] = None
    collider: Optional[bool] = True


class VisualCubeCfg(ObjectCfg):
    type: Optional[str] = 'VisualCube'
    color: Optional[List[float]] = None


class UsdObjCfg(ObjectCfg):
    type: Optional[str] = 'UsdObject'
    usd_path: str
    collider: Optional[bool] = True
