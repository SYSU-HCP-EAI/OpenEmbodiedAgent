from typing import Optional

from internutopia.core.config.robot import ControllerCfg


class InverseKinematicsControllerCfg(ControllerCfg):
    type: Optional[str] = 'InverseKinematicsController'
    robot_description_path: str
    robot_urdf_path: str
    end_effector_frame_name: str
    threshold: float
    reference: Optional[str] = None
