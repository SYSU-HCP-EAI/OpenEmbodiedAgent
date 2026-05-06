from typing import List, Optional

from internutopia.core.config.robot import ControllerCfg


class G1MoveBySpeedControllerCfg(ControllerCfg):
    type: Optional[str] = 'G1MoveBySpeedController'
    joint_names: List[str]
    policy_weights_path: str
