from collections import OrderedDict

from internutopia.core.config import TaskCfg
from internutopia.core.object.object import BaseObject, create_objects
from internutopia.core.scene.scene import IScene


def init_objects(task_config: TaskCfg, scene: IScene) -> OrderedDict[str, BaseObject]:
    return create_objects(task_config, scene)
