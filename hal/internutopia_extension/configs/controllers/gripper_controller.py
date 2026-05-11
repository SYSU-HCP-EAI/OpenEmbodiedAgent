from typing import Optional

from internutopia.core.config.robot import ControllerCfg


class GripperControllerCfg(ControllerCfg):
    type: Optional[str] = 'GripperController'
