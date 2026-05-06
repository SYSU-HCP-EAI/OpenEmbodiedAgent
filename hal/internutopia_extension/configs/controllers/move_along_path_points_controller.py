from typing import Optional

from internutopia.core.config.robot import ControllerCfg


class MoveAlongPathPointsControllerCfg(ControllerCfg):
    name: Optional[str] = 'move_along_path'
    type: Optional[str] = 'MoveAlongPathPointsController'
    forward_speed: Optional[float] = None
    rotation_speed: Optional[float] = None
    threshold: Optional[float] = None
