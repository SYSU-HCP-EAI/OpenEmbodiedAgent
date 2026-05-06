from abc import ABC, abstractmethod
from collections import OrderedDict
from typing import Dict

from internutopia.core.config.robot import RobotCfg, SensorCfg
from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia.core.util import log


class BaseSensor(ABC):
    """Base class of sensor."""

    sensors = {}

    def __init__(self, config: SensorCfg, robot: BaseRobot, scene: IScene):
        """Initialize the sensor.

        Args:
            config (SensorCfg): sensor configuration.
            robot (BaseRobot): robot owning the sensor.
            scene (Scene): scene from isaac sim.
        """
        if config.name is None:
            raise ValueError('must specify sensor name.')
        self.name = config.name
        self.config = config
        self._scene = scene
        self._robot = robot
        self.obs_keys = []

    @abstractmethod
    def get_data(self) -> OrderedDict:
        """Get data from sensor.

        Returns:
            Dict: data dict of sensor.
        """
        raise NotImplementedError()

    def post_reset(self):
        """
        Post reset operations.
        """
        pass

    def restore_sensor_info(self):
        """
        Restores the sensor information from a saved state or configuration.
        """
        pass

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

    @classmethod
    def register(cls, name: str):
        """
        Register an sensor class with the given name(decorator).

        Args:
            name(str): name of the sensor
        """

        def wrapper(sensor_class):
            """
            Register the sensor class.
            """
            cls.sensors[name] = sensor_class
            return sensor_class

        return wrapper


def create_sensors(robot_cfg: RobotCfg, robot: BaseRobot, scene: IScene) -> OrderedDict[str, BaseSensor]:
    """Create all sensors of one robot.

    Args:
        robot_cfg (RobotCfg): config of the robot.
        robot (BaseRobot): robot instance.
        scene (Scene): scene from isaac sim.

    Returns:
        Dict[str, BaseSensor]: dict of sensors with sensor name as key.
    """
    sensor_map = {}
    if robot_cfg.sensors is None:
        return OrderedDict()
    if robot_cfg.sensors is not None:
        for sensor_cfg in robot_cfg.sensors:
            sensor_cls = BaseSensor.sensors[sensor_cfg.type]
            sensor_ins = sensor_cls(sensor_cfg, robot=robot, scene=scene)
            sensor_map[sensor_cfg.name] = sensor_ins
            log.debug(f'[create_sensors] {sensor_cfg.name} loaded')

    return OrderedDict((sensor_cfg.name, sensor_map[sensor_cfg.name]) for sensor_cfg in robot_cfg.sensors)
