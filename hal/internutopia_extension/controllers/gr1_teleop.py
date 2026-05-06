import argparse
import logging
import os
from pathlib import Path

import lcm
import numpy as np
import yaml
from dex_retargeting.retargeting_config import RetargetingConfig
from lcmtypes.teleop import action, joints
from pin_ik_solver import PinIKSolver

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# conda run --no-capture-output -n sim-teleop python gr1t2_teleop.py
# cwd: InternUtopia/internutopia_extension/controllers


class GR1T2TeleOpHandler:
    def __init__(self, urdf_path: str, retargeting_config_path: str) -> None:
        self.lc = lcm.LCM()
        self.lc.subscribe('teleop_action', self.action_to_control)

        self.solver = PinIKSolver(urdf_path, False)
        self.head_idx = [13, 18, 21]
        self.left_arm_idx = [14, 19, 22, 24, 26, 28, 30]
        self.right_arm_idx = [15, 20, 23, 25, 27, 29, 31]

        RetargetingConfig.set_default_urdf_dir(os.path.dirname(retargeting_config_path))
        with Path(retargeting_config_path).open('r') as f:
            cfg = yaml.safe_load(f)

        self.left_retargeting = RetargetingConfig.from_dict(cfg['left']).build()
        self.right_retargeting = RetargetingConfig.from_dict(cfg['right']).build()
        self.dex2inspire = [4, 5, 6, 7, 10, 11, 8, 9, 0, 3, 2]
        self.tip_indices = [4, 9, 14, 19, 24]
        self.left_hand_idx = [32, 42, 33, 43, 34, 44, 35, 45, 36, 46, 52]
        self.right_hand_idx = [37, 47, 38, 48, 39, 49, 40, 50, 41, 51, 53]

    def action_to_control(self, channel, data):
        teleop_action = action.decode(data)
        target_joint_positions = np.zeros(54)

        # Solve IK.
        sol_q, success = self.solver.solve(
            np.array(teleop_action.left_wrist_mat),
            np.array(teleop_action.right_wrist_mat),
            np.array(teleop_action.head_mat),
        )
        target_joint_positions[self.head_idx] = sol_q[:3]
        target_joint_positions[self.left_arm_idx] = sol_q[3:10]
        target_joint_positions[self.right_arm_idx] = sol_q[10:17]

        # Retargeting.
        left_qpos = self.left_retargeting.retarget(np.array(teleop_action.left_hand_mat)[self.tip_indices])[
            self.dex2inspire
        ]
        right_qpos = self.right_retargeting.retarget(np.array(teleop_action.right_hand_mat)[self.tip_indices])[
            self.dex2inspire
        ]
        left_qpos[:-2] = -left_qpos[:-2]
        right_qpos[:-2] = -right_qpos[:-2]
        target_joint_positions[self.left_hand_idx] = left_qpos
        target_joint_positions[self.right_hand_idx] = right_qpos

        joint_control = joints()
        joint_control.joint_num = len(target_joint_positions)
        joint_control.joint_positions = target_joint_positions.tolist()

        self.lc.publish('teleop_joints', joint_control.encode())


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf_path', type=str, default='../../internutopia/assets/robots/gr1/urdf/robot.urdf')
    parser.add_argument(
        '--retargeting_config_path',
        type=str,
        default='../../internutopia/assets/robots/gr1/inspire_hand/inspire_hand.yml',
    )

    try:
        args = parser.parse_args()
        logging.info(f'args: {args}')
        teleop = GR1T2TeleOpHandler(**args.__dict__)
        logging.info('waiting for teleop action...')
        while True:
            teleop.lc.handle()
    except KeyboardInterrupt:
        pass
