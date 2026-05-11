from typing import Dict, Optional

import numpy as np

from internutopia.core.config import Config, TaskCfg
from internutopia.core.datahub import DataHub
from internutopia.core.task_config_manager.base import (
    BaseTaskConfigManager,
    setup_offset_for_assets,
)
from internutopia.core.util import log


class LocalTaskConfigManager(BaseTaskConfigManager):
    """
    Manages the configuration of tasks that run locally.
    """

    def __init__(self, config: Config):
        """
        Args:
            config (Config): Task config read from user input config file.
        """
        super().__init__(config=config)
        self.env_id_to_offset_map = {}
        self._current_env_id = 0
        self._column_length = int(np.sqrt(self.env_num))
        self.active_task_configs: Dict[int, TaskCfg] = {}

    def get_next(self, env_id: Optional[int] = None) -> tuple[str, int, list[float] | None, TaskCfg | None]:
        """
        Get next task's [task_name, env_id, env_offset, task_config] with env of last episode.

        Args:
            env_id (Optional[int]): env_id of last episode.

        Returns:
            str: Task name.
            int: env id of next task.
            List[float]: env_offset of next task.
            TaskCfg: TaskCfg for next task.

        Raises:
            ValueError: If too many sub envs have been created.
        """
        if env_id is None and self._current_env_id >= self.env_num:
            log.error(f'self._current_env_id is {self._current_env_id}')
            log.error(f'self.env_num is {self.env_num}')
            raise ValueError('Too many sub envs have been created.')

        next_episode: TaskCfg = self._pop_next_episode()
        if next_episode is None:
            if env_id is not None:
                del self.active_task_configs[env_id]
            return '', env_id, None, None

        # create last_env if not exist
        if env_id is None:
            env_id = self._get_next_env_id()
        env_offset = self.env_id_to_offset_map[env_id]

        # Update task_idx
        task_idx = DataHub.gen_task_idx()

        setup_offset_for_assets(next_episode, env_id, env_offset)

        self.active_task_configs[env_id] = next_episode

        return task_idx, env_id, env_offset, next_episode

    # TODO: This method should not be here, it can be put in Runner
    def _get_next_env_id(self):
        env_id = self._current_env_id
        row = int(env_id // self._column_length)
        column = env_id % self._column_length
        offset = [row * self.env_offset_size, column * self.env_offset_size, 0]
        self._current_env_id += 1
        self.env_id_to_offset_map[env_id] = offset
        return env_id

    def get_active_task_configs(self) -> Dict[int, TaskCfg]:
        """
        Get active task configs with env id as key.
        """
        return self.active_task_configs
