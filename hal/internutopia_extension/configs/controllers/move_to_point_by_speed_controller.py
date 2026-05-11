from typing import Optional

from internutopia.core.config.robot import ControllerCfg


class MoveToPointBySpeedControllerCfg(ControllerCfg):

    type: Optional[str] = 'MoveToPointBySpeedController'
    forward_speed: Optional[float] = None
    rotation_speed: Optional[float] = None
    threshold: Optional[float] = None
