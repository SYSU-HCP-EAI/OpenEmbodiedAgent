from typing import List, Optional, Tuple

from pydantic import BaseModel, Extra

from internutopia.core.config import ObjectCfg, RobotCfg
from internutopia.core.config.metric import MetricCfg
from internutopia.core.config.task.reward import RewardCfg


class TaskCfg(BaseModel, extra=Extra.allow):
    type: Optional[str] = None

    # inherit
    metrics: Optional[List[MetricCfg]] = []
    reward: Optional[RewardCfg] = None

    # path
    scene_root_path: Optional[str] = '/scene'
    robots_root_path: Optional[str] = '/robots'
    objects_root_path: Optional[str] = '/objects'

    # scene
    scene_asset_path: Optional[str] = None
    scene_scale: Optional[Tuple[float, float, float]] = (1.0, 1.0, 1.0)
    scene_position: Optional[Tuple[float, float, float]] = (0, 0, 0)
    scene_orientation: Optional[Tuple[float, float, float, float]] = (1.0, 0, 0, 0)

    # inherit
    robots: Optional[List[RobotCfg]] = []
    objects: Optional[List[ObjectCfg]] = []
