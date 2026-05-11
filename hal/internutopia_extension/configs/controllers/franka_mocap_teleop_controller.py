from typing import Optional, Tuple

from internutopia.core.config.robot import ControllerCfg


class FrankaMocapTeleopControllerCfg(ControllerCfg):
    type: Optional[str] = 'FrankaMocapTeleopController'
    scale: Tuple[float, float, float]
    target_position: Tuple[float, float, float]
    origin_xyz: Optional[Tuple[float, float, float]] = None
    origin_xyz_angle: Optional[Tuple[float, float, float]] = None
