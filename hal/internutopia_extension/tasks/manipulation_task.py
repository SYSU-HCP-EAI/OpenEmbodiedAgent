from internutopia.core.datahub import DataHub
from internutopia.core.scene.scene import IScene
from internutopia.core.task import BaseTask
from internutopia.core.util import log
from internutopia_extension.configs.tasks.manipulation_task import ManipulationTaskCfg


@BaseTask.register('ManipulationTask')
class ManipulationTask(BaseTask):
    def __init__(self, config: ManipulationTaskCfg, scene: IScene):
        super().__init__(config, scene)
        self.step_counter = 0
        self.max_steps = config.max_steps
        self.prompt = config.prompt

        log.info(f'task_settings: max_step       : {self.max_steps}.)')
        # Episode
        log.info(f'Episode meta : prompt         : {self.prompt}.)')

    def is_done(self) -> bool:
        self.step_counter = self.step_counter + 1
        return DataHub.get_episode_finished(self.name) or self.step_counter > self.max_steps

    def update_metrics(self):
        return super().update_metrics()

    def calculate_metrics(self) -> dict:
        metrics_res = {}
        for name, metric in self.metrics.items():
            metrics_res[name] = metric.calc()

        return metrics_res
