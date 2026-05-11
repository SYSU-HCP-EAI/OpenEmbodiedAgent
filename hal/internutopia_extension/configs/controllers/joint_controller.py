from typing import List, Optional

from internutopia.core.config.robot import ControllerCfg


class JointControllerCfg(ControllerCfg):

    type: Optional[str] = 'JointController'
    joint_names: List[str]
