from typing import Optional

from internutopia.core.config.task import TaskCfg


class ManipulationTaskCfg(TaskCfg):
    type: Optional[str] = 'ManipulationTask'
    max_steps: int
    prompt: Optional[str] = ''
    target: str
    episode_idx: int
