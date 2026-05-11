from typing import List, Optional

from internutopia.core.config.robot import ControllerCfg


class GR1MoveBySpeedControllerCfg(ControllerCfg):
    type: Optional[str] = 'GR1MoveBySpeedController'
    joint_names: List[str]
    policy_weights_path: str
