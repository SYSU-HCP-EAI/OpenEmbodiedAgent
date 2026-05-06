import weakref
from abc import ABC, abstractmethod
from collections import OrderedDict
from typing import Any, Dict, List, Union

import numpy as np

from internutopia.core.config.robot import ControllerCfg, RobotCfg
from internutopia.core.robot.articulation_action import ArticulationAction
from internutopia.core.robot.articulation_subset import ArticulationSubset
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia.core.util import log


class BaseController(ABC):
    """Base class of controller."""

    controllers = {}

    def __init__(self, config: ControllerCfg, robot: BaseRobot, scene: IScene):
        """Initialize the controller.

        Args:
            config (ControllerCfg): controller configuration.
            robot (BaseRobot): robot owning the controller.
            scene (IScene): scene interface.

        """
        self.sub_controllers = None
        self.scene = scene
        if config.name is None:
            raise ValueError('must specify controller name.')
        self._obs = {}
        self.robot_ref = weakref.ref(robot)  # Use `weakref` to prevent memory leaks caused by circular references
        self.config = config
        self.sub_controllers: List[BaseController]
        self.obs_keys = []

    @abstractmethod
    def action_to_control(self, action: Union[np.ndarray, List]) -> ArticulationAction:
        """Convert input action (in 1d array format) to joint signals to apply.

        Args:
            action (Union[np.ndarray, List]): input control action.

        Returns:
            ArticulationAction: joint signals to apply
        """
        raise NotImplementedError()

    def get_obs(self) -> OrderedDict[str, Any]:
        """Get observation of controller.

        Returns:
            OrderedDict[str, Any]: observation key and value.
        """
        return OrderedDict()

    @classmethod
    def register(cls, name: str):
        """
        Register an controller class with the given name(decorator).

        Args:
            name(str): name of the controller
        """

        def wrapper(controller_class):
            """
            Register the controller class.
            """
            cls.controllers[name] = controller_class
            return controller_class

        return wrapper

    @property
    def robot(self):
        return self.robot_ref()

    def cleanup(self):
        """
        Operations that need to be cleaned up before switching scenes (or resetting)
        """
        pass

    def _make_ordered(self, obs: Dict = None) -> OrderedDict:
        if obs is None:
            return OrderedDict()
        if not self.obs_keys:
            self.obs_keys = [i for i in obs.keys()]
        return OrderedDict((key, obs[key]) for key in self.obs_keys)

    def get_joint_subset(self) -> ArticulationSubset | None:
        """Get the joint subset controlled by the controller.

        Returns:
            ArticulationSubset: joint subset.
        """
        if hasattr(self, 'joint_subset'):
            return self.joint_subset
        if not hasattr(self, 'sub_controllers'):
            return None
        if self.sub_controllers is None or len(self.sub_controllers) == 0:
            return None
        return self.sub_controllers[0].get_joint_subset()


def create_controllers(robot_cfg: RobotCfg, robot: BaseRobot, scene: IScene) -> OrderedDict[str, BaseController]:
    """Create all controllers of one robot.

    Args:
        robot_cfg (RobotCfg): config of the robot.
        robot (BaseRobot): robot instance.
        scene (Scene): scene from isaac sim.

    Returns:
        Dict[str, BaseController]: dict of controllers with controller name as key.
    """
    controller_map = {}

    if robot_cfg.controllers is None:
        return OrderedDict(controller_map)
    for controller_cfg in robot_cfg.controllers:
        controller_name = controller_cfg.name
        controller_cls = BaseController.controllers[controller_cfg.type]
        controller_ins: BaseController = controller_cls(config=controller_cfg, robot=robot, scene=scene)
        if controller_cfg.sub_controllers is not None:
            inject_sub_controllers(
                parent=controller_ins,
                configs=controller_cfg.sub_controllers,
                robot=robot,
                scene=scene,
            )

        controller_map[controller_name] = controller_ins
        log.debug(f'[create_controllers] {controller_name} loaded')

    return OrderedDict(
        (controller_cfg.name, controller_map[controller_cfg.name]) for controller_cfg in robot_cfg.controllers
    )


def inject_sub_controllers(
    parent: BaseController,
    configs: List[ControllerCfg],
    robot: BaseRobot,
    scene: IScene,
):
    """Recursively create and inject sub-controllers into parent controller.

    Args:
        parent (BaseController): parent controller instance.
        configs (List[ControllerParams]): user configs of sub-controllers.
        robot (BaseRobot): robot instance.
        scene (Scene): scene from isaac sim.
    """
    if len(configs) == 0:
        return
    sub_controllers: List[BaseController] = []
    for config in configs:
        controller_cls = BaseController.controllers[config.type]
        controller_ins = controller_cls(config=config, robot=robot, scene=scene)
        if config.sub_controllers is not None:
            inject_sub_controllers(controller_ins, configs=config.sub_controllers, robot=robot, scene=scene)
        sub_controllers.append(controller_ins)

    parent.sub_controllers = sub_controllers
