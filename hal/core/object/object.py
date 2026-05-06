from collections import OrderedDict

from internutopia.core.config import ObjectCfg as ObjectConfig
from internutopia.core.config import TaskCfg
from internutopia.core.scene.scene import IScene
from internutopia.core.util import log, remove_suffix


class BaseObject:
    """Base class of object."""

    objs = {}

    def __init__(self, config: ObjectConfig, scene: IScene):
        super().__init__()
        self.name = config.name
        self.config = config
        self._scene = scene

    def set_up_to_scene(self, scene: IScene):
        raise NotImplementedError

    @classmethod
    def register(cls, name: str):
        """
        Register an object class with the given name(decorator).

        Args:
            name(str): name of the object
        """

        def wrapper(object_class):
            """
            Register the object class.
            """
            cls.objs[name] = object_class
            return object_class

        return wrapper


def create_objects(task_config: TaskCfg, scene: IScene) -> OrderedDict[str, BaseObject]:
    """Create object instances in task config.

    Args:
        task_config (TaskCfg): task config.
        scene (Scene): isaac scene.

    Returns:
        OrderedDict[str, BaseObject]: robot instances dictionary.
    """
    object_map = OrderedDict()
    for object in task_config.objects:
        if object.type not in BaseObject.objs:
            raise KeyError(f'[create_robots] unknown robot type "{object.type}"')
        object_cls = BaseObject.objs[object.type]
        object_ins: BaseObject = object_cls(object, scene)
        object_map[remove_suffix(object.name)] = object_ins
        object_ins.set_up_to_scene(scene)
        log.debug(f'[create_objects] {object.name} loaded')
    return object_map
