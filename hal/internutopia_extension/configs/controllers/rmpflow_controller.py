from typing import Optional

from internutopia.core.config.robot import ControllerCfg


class RMPFlowControllerCfg(ControllerCfg):
    type: Optional[str] = 'RMPFlowController'
    robot_description_path: str
    robot_urdf_path: str
    rmpflow_config_path: str
    end_effector_frame_name: str
