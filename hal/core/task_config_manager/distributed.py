from threading import Lock
from typing import Dict, Optional

import numpy as np

from internutopia.core.config import Config, TaskCfg
from internutopia.core.datahub import DataHub
from internutopia.core.task_config_manager.base import (
    BaseTaskConfigManager,
    setup_offset_for_assets,
)


class DistributedTaskConfigManager(BaseTaskConfigManager):
    def __init__(self, config: Config = None):
        super().__init__(config)
        self.env_num = config.env_num
        self.proc_num = config.distribution_config.proc_num
        self.lock = Lock()
        self._column_length = int(np.sqrt(self.env_num))
        self._current_env_ids = [0 for _ in range(self.proc_num)]
        self.active_task_configs = [{} for _ in range(self.proc_num)]
        self.env_id_to_offset_maps = [{} for _ in range(self.proc_num)]

    # TODO: This method should not be here, it can be put in Runner
    def _get_next_env_id(self, runner_id) -> int:
        env_id = self._current_env_ids[runner_id]
        row = int(env_id // self._column_length)
        column = env_id % self._column_length
        offset = [row * self.env_offset_size, column * self.env_offset_size, 0]
        self._current_env_ids[runner_id] += 1
        self.env_id_to_offset_maps[runner_id][env_id] = offset
        return env_id

    def get_next(
        self, env_id: Optional[int] = None, runner_id: Optional[int] = 0
    ) -> tuple[str, int, list[float] | None, TaskCfg | None]:
        """
        Get next task's [task_name, env_id, env_offset, task_config] with env of last episode.

        Args:
            env_id (Optional[int]): env_id of last episode.
            runner_id (Optional[int]): runner_id of last episode.

        Returns:
            str: Task name.
            int: env id of next task.
            List[float]: env_offset of next task.
            TaskCfg: TaskCfg for next task.

        Raises:
            ValueError: If too many sub envs have been created.
        """

        with self.lock:

            if env_id is None and self._current_env_ids[runner_id] >= self.env_num:
                raise ValueError('Too many sub envs have been created.')

            next_episode: TaskCfg = self._pop_next_episode()
            if next_episode is None:
                if env_id is not None:
                    del self.active_task_configs[runner_id][env_id]
                return '', env_id, None, None

            # create last_env if not exist
            if env_id is None:
                env_id = self._get_next_env_id(runner_id)
            env_offset = self.env_id_to_offset_maps[runner_id][env_id]

            # Update task_idx
            task_idx = DataHub.gen_task_idx()

            setup_offset_for_assets(next_episode, env_id, env_offset)

            self.active_task_configs[runner_id][env_id] = next_episode

            return task_idx, env_id, env_offset, next_episode

    def get_active_task_configs(self) -> Dict[int, TaskCfg]:
        """
        Get active task configs with env id as key.
        """
        active_task_configs = {}
        for runner_id, task_config_map in enumerate(self.active_task_configs):
            for env_id, active_config in task_config_map.items():
                index = self.env_num * runner_id + env_id
                active_task_configs[index] = active_config

        return active_task_configs
