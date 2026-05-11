from collections import OrderedDict
from typing import Any, Dict, List, Tuple

import numpy as np

from internutopia.core.config import TaskCfg
from internutopia.core.config.robot import RobotCfg
from internutopia.core.robot.articulation import IArticulation
from internutopia.core.robot.rigid_body import IRigidBody
from internutopia.core.scene.scene import IScene
from internutopia.core.util import log, remove_suffix


class BaseRobot:
    """Base class of robot."""

    robots = {}

    def __init__(self, config: RobotCfg, scene: IScene):
        self.name = config.name
        self.config = config
        self.articulation: IArticulation | None = None
        self.controllers = {}
        self.sensors = {}
        self._scene = scene
        self.obs_keys = []
        self._rigid_body_map: Dict[str, IRigidBody] = {}

    def set_up_to_scene(self, scene: IScene):
        """Set up robot in the scene.

        Args:
            scene (Scene): scene to set up.
        """
        if self.articulation is None:
            raise RuntimeError('The attribute self.articulation needs to be initialized in subclass(robot extensions)')
        self.create_rigid_bodies()
        self._scene = scene
        robot_cfg = self.config
        scene.add(self.articulation)
        for rigid_body in self.get_rigid_bodies():
            scene.add(rigid_body)
        from internutopia.core.robot.controller import (
            BaseController,
            create_controllers,
        )
        from internutopia.core.sensor.sensor import BaseSensor, create_sensors

        self.controllers: OrderedDict[str, BaseController] = create_controllers(robot_cfg, self, scene)
        self.sensors: OrderedDict[str, BaseSensor] = create_sensors(robot_cfg, self, scene)

    def clean_stale_rigid_bodies(self):
        """
        Removes stale rigid bodies from the scene.
        """
        if self._rigid_body_map:
            for rb in self._rigid_body_map.values():
                if self._scene.object_exists(rb.name):
                    self._scene.remove(rb.name)
                    log.debug(f'[clean_stale_rigid_body] stale rigid body {rb.name} removed')

    def create_rigid_bodies(self):
        """
        Create rigid bodies.
        """
        from pxr import Usd

        _prim = self.articulation.prim

        # articulation on rigid-body situation
        if (
            'PhysicsArticulationRootAPI' in self.articulation.prim.GetAppliedSchemas()
            and 'PhysicsRigidBodyAPI' in self.articulation.prim.GetAppliedSchemas()
        ):
            parts = str(self.articulation.prim.GetPath()).rstrip('/').split('/')
            _root_prim_path = '/'.join(parts[:-1]) if len(parts) > 1 else ''
            _prim = self._scene.unwrap().stage.GetPrimAtPath(_root_prim_path)

        for prim in Usd.PrimRange.AllPrims(_prim):
            if prim.GetAttribute('physics:rigidBodyEnabled'):
                log.debug(f'[create_rigid_bodies] found rigid body at path: {prim.GetPath()}')
                _rb = IRigidBody.create(prim_path=str(prim.GetPath()), name=str(prim.GetPath()))
                self._rigid_body_map[str(prim.GetPath())] = _rb

    def save_robot_info(self):
        """
        Saves the current state of the robot's articulation information.
        """
        log.info('saving robot info')
        self.articulation.save_status()

    def restore_robot_info(self):
        """
        Restores the robot's information and its state within the simulation environment.
        """
        self.articulation._articulation_view._is_initialized = False
        self.articulation._articulation_view._on_physics_ready('reset')
        self.clean_stale_rigid_bodies()
        self.create_rigid_bodies()
        self.post_reset()
        self.articulation.restore_status()

    def post_reset(self):
        """Set up things after the env resets."""
        for sensor in self.sensors.values():
            sensor.post_reset()

    def cleanup(self):
        """
        Cleans up resources associated with the current instance by performing cleanup
        operations on controllers, sensors, and rigid bodies. Ensures that all associated
        objects are properly removed from the scene.
        """
        for controller in self.controllers.values():
            controller.cleanup()
        for sensor in self.sensors.values():
            sensor.cleanup()
        for rigid_body in self.get_rigid_bodies():
            try:
                if self._scene.object_exists(rigid_body.name):
                    self._scene.remove(rigid_body.name, registry_only=True)
                    log.debug(f'[cleanup] rigid body {rigid_body.name} removed from scene')
            except RuntimeError as e:
                log.error(e)
                continue
        log.debug(f'[cleanup] robot {self.name} clean up')

    def apply_action(self, action: dict):
        """Apply actions of controllers to robot.

        Args:
            action (dict): action dict.
              key: controller name.
              value: corresponding action array.
        """
        raise NotImplementedError()

    def get_obs(self) -> OrderedDict:
        """Get observation of robot, including controllers, sensors, and world pose.

        Raises:
            NotImplementedError: _description_
        """
        raise NotImplementedError()

    def _get_controllers_and_sensors_obs(self) -> Tuple[OrderedDict[str, Any], OrderedDict[str, Any]]:
        """
        Retrieves the observations from all controllers and sensors, returning them as two separate ordered dictionaries.

        Returns:
            Tuple[OrderedDict[str, Any], OrderedDict[str, Any]]: A tuple containing two ordered dictionaries. The first
            dictionary contains the observations from the controllers, and the second contains the data from the sensors.
        """
        controllers_obs = OrderedDict()
        sensors_obs = OrderedDict()
        for c_obs_name, controller_obs in self.controllers.items():
            controllers_obs[c_obs_name] = controller_obs.get_obs()
        for sensor_name, sensor_obs in self.sensors.items():
            sensors_obs[sensor_name] = sensor_obs.get_data()
        return controllers_obs, sensors_obs

    def get_robot_base(self) -> IRigidBody:
        """
        Get the base link of this robot.

        Returns:
            IRigidBody: rigid prim of the robot base link.
        """
        raise NotImplementedError()

    def get_robot_scale(self) -> np.ndarray:
        """Get robot scale.

        Returns:
            np.ndarray: robot scale in (x, y, z).
        """
        raise NotImplementedError()

    def get_robot_articulation(self) -> IArticulation:
        """Get isaac robots instance (articulation).

        Returns:
            Robot: robot articulation.
        """
        return self.articulation

    def get_controllers(self):
        return self.controllers

    def get_rigid_bodies(self) -> List[IRigidBody]:
        return self._rigid_body_map.values()

    def _make_ordered(self, obs: Dict = None) -> OrderedDict:
        if obs is None:
            return OrderedDict()
        if not self.obs_keys:
            self.obs_keys = [i for i in obs.keys()]
        return OrderedDict((key, obs[key]) for key in self.obs_keys)

    @classmethod
    def register(cls, name: str):
        """
        Register an robot class with the given name(decorator).

        Args:
            name(str): name of the robot
        """

        def wrapper(robot_class):
            """
            Register the robot class.
            """
            cls.robots[name] = robot_class
            return robot_class

        return wrapper


def init_robots(task_config: TaskCfg, scene: IScene) -> OrderedDict[str, BaseRobot]:
    return create_robots(task_config, scene)


def create_robots(task_config: TaskCfg, scene: IScene) -> OrderedDict[str, BaseRobot]:
    """Create robot instances in task config.

    Args:
        task_config (TaskCfg): task config.
        scene (Scene): isaac scene.

    Returns:
        OrderedDict[str, BaseRobot]: robot instances dictionary.
    """
    robot_map = OrderedDict()
    for robot in task_config.robots:
        if robot.type not in BaseRobot.robots:
            raise KeyError(f'[create_robots] unknown robot type "{robot.type}"')
        robot_cls = BaseRobot.robots[robot.type]
        robot_ins: BaseRobot = robot_cls(robot, scene)
        robot_map[remove_suffix(robot.name)] = robot_ins
        robot_ins.set_up_to_scene(scene)
        log.debug(f'[create_robots] {robot.name} loaded')
    return robot_map
