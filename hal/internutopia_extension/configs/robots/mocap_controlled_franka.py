from typing import Optional

from internutopia.core.config import RobotCfg
from internutopia.macros import gm
from internutopia_extension.configs.controllers import (
    FrankaMocapTeleopControllerCfg,
    GripperControllerCfg,
    LayoutEditMocapControllerCfg,
    RMPFlowControllerCfg,
)
from internutopia_extension.configs.sensors import (
    LayoutEditMocapControlledCameraCfg,
    MocapControlledCameraCfg,
)

rmpflow_controller_cfg = RMPFlowControllerCfg(
    name='rmpflow_controller',
    robot_description_path=gm.ASSET_PATH + '/robots/franka/rmpflow/robot_descriptor.yaml',
    robot_urdf_path=gm.ASSET_PATH + '/robots/franka/lula_franka_gen.urdf',
    rmpflow_config_path=gm.ASSET_PATH + '/robots/franka/franka_rmpflow_common.yaml',
    end_effector_frame_name='right_gripper',
)

gripper_controller_cfg = GripperControllerCfg(
    name='gripper_controller',
)

teleop_cfg = FrankaMocapTeleopControllerCfg(
    name='franka_mocap_teleop_controller',
    scale=(0.25, 0.75, 0.75),
    target_position=(0.0, 0.0, 1.0),
    origin_xyz=(0.46, 0, 0.38),
    origin_xyz_angle=(0, 0, 0),
    sub_controllers=[rmpflow_controller_cfg, gripper_controller_cfg],
)

layout_cfg = LayoutEditMocapControllerCfg(
    name='layout_edit_mocap_controller',
    scale=(0.25, 0.75, 0.75),
    hand_scale=[0.01, 0.01, 0.01],
    target_position=(0.0, 0.0, 1.0),
    rh_origin_xyz=(-0.38, -0.20, 1.2),
    lh_origin_xyz=(-0.38, 0.10, 1.2),
    origin_xyz_angle=(0, 0, 0),
    right_hand_path=gm.ASSET_PATH + '/assets/objects/hand/right_hand.usd',
    left_hand_path=gm.ASSET_PATH + '/assets/objects/hand/left_hand.usd',
    left_hand_prim_path='/World/left_hand',
    right_hand_prim_path='/World/right_hand',
    save_asset_path='InternUtopia/out',
    save_robot=False,
    sub_controllers=[rmpflow_controller_cfg],
)

lh_controlled_camera_cfg = MocapControlledCameraCfg(
    name='lh_controlled_camera',
    prim_path='logo_link/lh_controlled_camera',
    translation=(0.0, 0.0, 5.0),
    orientation=(0.70710678, 0.0, 0.70710678, 0.0),
)

layout_controlled_camera_cfg = LayoutEditMocapControlledCameraCfg(
    name='layout_controlled_camera',
    prim_path='Camera/layout_controlled_camera',
    translation=(0.11, -0.04, 6.5),
    orientation=(0.70710678, 0.0, 0.70710678, 0.0),
)


class MocapControlledFrankaRobotCfg(RobotCfg):
    name: Optional[str] = 'mocap_controlled_franka'
    type: Optional[str] = 'MocapControlledFrankaRobot'
    prim_path: Optional[str] = '/mocap_controlled_franka'
    usd_path: Optional[str] = gm.ASSET_PATH + '/robots/franka/mocap_teleop_franka.usd'
