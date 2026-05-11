from internutopia.core.config import Config, DistributedConfig
from internutopia.core.config.distribution import DistributionCfg, RayDistributionCfg
from internutopia.core.distribution.runner_proxy import RunnerProxy


class Launcher:
    def __init__(self, config, task_config_manager):
        from internutopia.core.runner import SimulatorRunner

        self.config: Config = config
        self.task_config_manager = task_config_manager
        self.runner_factory = SimulatorRunner
        if isinstance(config, DistributedConfig):
            self.distribution_config: DistributionCfg = self.config.distribution_config
            if not isinstance(self.distribution_config, RayDistributionCfg):
                raise Exception(f'unsupport distribution config type :{type(self.distribution_config)}')

    def start(self):

        if not isinstance(self.config, DistributedConfig):
            runner = self.runner_factory(
                config=self.config,
                task_config_manager=self.task_config_manager,
            )
            return RunnerProxy(runner, is_remote=False)
        else:
            import ray

            _remote_args = {
                'num_cpus': 1,
                'num_gpus': self.config.distribution_config.gpu_num_per_proc,
            }
            remote_class = ray.remote(**_remote_args)(self.runner_factory)
            remote_class = remote_class.options(placement_group_bundle_index=-1)
            runner = remote_class.remote(
                config=self.config,
                task_config_manager=self.task_config_manager,
            )
            return RunnerProxy(runner, is_remote=True)
