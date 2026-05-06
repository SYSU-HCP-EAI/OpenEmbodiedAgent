from typing import List, Optional

from internutopia.core.config.robot import ControllerCfg


class H1MoveBySpeedControllerCfg(ControllerCfg):

    type: Optional[str] = 'H1MoveBySpeedController'
    joint_names: List[str]
    policy_weights_path: str
