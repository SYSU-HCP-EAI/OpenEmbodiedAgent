from internutopia.core.config import TaskCfg
from internutopia.core.task.metric import BaseMetric
from internutopia_extension.configs.metrics.recording_metric import RecordingMetricCfg


@BaseMetric.register('RecordingMetric')
class RecordingMetric(BaseMetric):
    """
    Record any controller's output or joint actions during playing or teleoperating
    """

    def __init__(self, config: RecordingMetricCfg, task_config: TaskCfg):
        super().__init__(config, task_config)
        self.param = config
        self.fields = self.param.fields
        self.robot_name = self.param.robot_name
        self.step = 0
        self.obs = {}

    def update(self, obs: dict):
        """
        This function is called at each world step.
        """
        if self.fields is None:
            self.obs[self.step] = obs
        else:
            assert len(obs) == 1, f'only support one task currently. but len(task_obs) is : {len(obs)}'
            robot_name = list(obs.keys())[0]
            self.obs[self.step] = {robot_name: {}}
            for record_field in self.fields:
                self.obs[self.step][robot_name][record_field] = obs[robot_name][record_field]

        self.step += 1

    def calc(self):
        """
        This function is called to calculate the metrics when the episode is terminated.
        """
        return self.obs
