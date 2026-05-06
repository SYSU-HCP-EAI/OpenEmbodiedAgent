from typing import List

import numpy as np


class gym_adapter:
    def __init__(self, joint_names_gym: List[str], joint_names_sim: List[str]):
        self.joint_names_gym = joint_names_gym
        self.joint_names_sim = joint_names_sim

    @staticmethod
    def rearange_order(src: list | np.ndarray, from_joint_names: List[str], to_joint_names: List[str]):
        src = src.tolist()
        sim_actions = [None] * len(from_joint_names)
        for i, gym_joint in enumerate(from_joint_names):
            sim_joint_index = to_joint_names.index(gym_joint)
            sim_actions[sim_joint_index] = src[i]

        return np.array(sim_actions)

    def gym2sim(self, action):
        return self.rearange_order(action, self.joint_names_gym, self.joint_names_sim)

    def sim2gym(self, obs):
        return self.rearange_order(obs, self.joint_names_sim, self.joint_names_gym)
