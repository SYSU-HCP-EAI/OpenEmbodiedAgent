from abc import ABC, abstractmethod

from internutopia.core.config.metric import MetricCfg
from internutopia.core.task_config_manager.base import TaskCfg


class BaseMetric(ABC):
    metrics = {}

    def __init__(self, config: MetricCfg, task_config: TaskCfg):
        self.env_offset = None
        self.env_id = None
        self.task_name = None
        self.config = config
        self.name = config.name
        self.task_config = task_config
        self.metric_config = config.metric_config

    @abstractmethod
    def update(self, *args):
        """
        This function is called at each world step.
        """
        raise NotImplementedError(f'`update` function of {self.name} is not implemented')

    @abstractmethod
    def calc(self):
        """
        This function is called to calculate the metrics when the episode is terminated.
        """
        raise NotImplementedError(f'`calc` function of {self.name} is not implemented')

    @classmethod
    def register(cls, name: str):
        """
        Register a metric class with the given name(decorator).

        Args:
            name(str): name of the metric
        """

        def wrapper(metric_class):
            """
            Register the metric class.
            """
            cls.metrics[name] = metric_class
            return metric_class

        return wrapper

    def set_up_runtime(self, task_name, env_id, env_offset):
        self.task_name = task_name
        self.env_id = env_id
        self.env_offset = env_offset


def create_metric(config: MetricCfg, task_config: TaskCfg):
    if config.type not in BaseMetric.metrics:
        raise KeyError(
            f"""The metric {config.type} is not registered, please register it using `@BaseMetric.register`"""
        )
    metric_cls = BaseMetric.metrics[config.type]
    return metric_cls(config, task_config)
