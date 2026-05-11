from typing import Any, Dict, List, OrderedDict, Tuple, Union

from internutopia.core.config import Config, DistributedConfig
from internutopia.core.distribution.launcher import Launcher
from internutopia.core.task_config_manager.base import create_task_config_manager
from internutopia.core.util import extensions_utils, log


class Env:
    """
    Vectorized environments to run multiple independent environments simultaneously with multiple agents.

    **NOT gymnasium compatible**.

    Parameters:
        config (Config): The config instance used for simulation
            management.
    """

    def __init__(self, config: Config) -> None:
        self._render = None
        self._config = config
        self.task_config_manager = create_task_config_manager(self._config)
        self._runner_list = []
        self.env_num = self._config.env_num
        self.proc_num = 1
        self.is_remote = False
        if isinstance(config, DistributedConfig):
            import ray

            extensions = extensions_utils.dump_extensions()
            self._config.distribution_config.extensions = extensions
            self.is_remote = True
            self.proc_num = self._config.distribution_config.proc_num
            cluster_gpu_count = ray.cluster_resources().get('GPU', 0)
            request_gpu_count = self.proc_num * self._config.distribution_config.gpu_num_per_proc
            if cluster_gpu_count < request_gpu_count:
                description = f'Insufficient cluster resources, requested GPU: {request_gpu_count}, total GPU: {cluster_gpu_count}, '
                description += (
                    'Please adjust proc_num and gpu_num_per_proc to ensure that resources can meet requirements'
                )
                raise RuntimeError(description)
            for runner_id in range(self.proc_num):
                self._config.distribution_config.runner_id = runner_id
                self._runner_list.append(Launcher(self._config, self.task_config_manager).start())
        else:
            self._runner_list.append(Launcher(self._config, self.task_config_manager).start())

        return

    def reset(self, env_ids: List[int] = None) -> Tuple[List, List]:
        """
        Resets the environments specified by the given environment IDs and returns the initial observations
        and task configs. If no environment IDs are provided, all environments will be reset. If no tasks
        are running after the reset, a log message is generated, and empty lists are returned.

        Parameters:
            env_ids (List[int]): A list of environment IDs to reset. If None, all environments will be reset.

        Returns:
            Tuple[List, List]: A tuple containing two lists: the initial observations and task configs for
            the reset environments. If no tasks are running, both lists will be empty.
        """
        obs = []
        task_configs = []
        new_env_ids = [None for _ in range(self.proc_num)]
        if env_ids is not None:
            result_list = []
            for env_id in env_ids:
                runner_id = env_id // self.env_num
                if new_env_ids[runner_id] is None:
                    new_env_ids[runner_id] = [env_id % self.env_num]
                else:
                    new_env_ids[runner_id].append(env_id % self.env_num)
            for runner_id in range(self.proc_num):
                if new_env_ids[runner_id]:
                    result_list.append(self._runner_list[runner_id].reset(env_ids=new_env_ids[runner_id]))
        else:
            result_list = [
                self._runner_list[runner_id].reset(env_ids=new_env_ids[runner_id]) for runner_id in range(self.proc_num)
            ]
        if self.is_remote:
            import ray

            result_list = ray.get(result_list)
        for _obs, _task_configs in result_list:
            obs.extend(_obs)
            task_configs.extend(_task_configs)

        if all(task_config is None for task_config in task_configs):
            log.info('No more episodes left')

        return obs, task_configs

    def warm_up(self, steps: int = 10, render: bool = True, physics: bool = True):
        """
        Warm up the env by running a specified number of steps.

        Args:
            steps (int): The number of warm-up steps to perform. Defaults to 10.
            render (bool): Whether to render the scene during warm-up. Defaults to True.
            physics (bool): Whether to enable physics during warm-up. Defaults to True.
        """
        result_list = [
            self._runner_list[index].warm_up(steps, render, physics) for index in range(len(self._runner_list))
        ]
        if self.is_remote:
            import ray

            result_list = ray.get(result_list)

    def step(self, action: List[Union[Dict, OrderedDict]]) -> Tuple[List, List, List, List, List]:
        """
        Perform a single step in the environment using the provided actions.

        This method takes a list of actions, validates its structure and length, then delegates
        the step execution to the internal runner. It computes and returns the observations,
        rewards, termination status, truncation status, and additional information.

        Args:
            action (List[Union[Dict, OrderedDict]]): A list of actions to be executed in the
                environment. Each action is either a dictionary or an ordered dictionary.

        Returns:
            Tuple[List, List, List, List, List]: A tuple containing the following elements:
                - obs (List): The observations resulting from the actions.
                - reward (List): The rewards obtained from the actions.
                - terminated (List): The termination status of the environments.
                - truncated (List): The truncation status of the environments.
                - info (List): Additional information about the step execution.
        """
        assert isinstance(action, list)
        assert len(action) == self.env_num * self.proc_num

        obs = []
        reward = []
        terminated = []
        truncated = [False for _ in action]
        info = [None for _ in action]

        result_list = [
            self._runner_list[index].step(action[index * self.env_num : (index + 1) * self.env_num])
            for index in range(len(self._runner_list))
        ]
        if self.is_remote:
            import ray

            result_list = ray.get(result_list)

        for _obs, _terminated, _reward in result_list:
            obs.extend(_obs)
            terminated.extend(_terminated)
            reward.extend(_reward)

        return obs, reward, terminated, truncated, info

    @property
    def runner(self):
        """
        The runner property provides access to the internal runner instance.
        """
        if not self.is_remote:
            return self._runner_list[0].runner
        raise NotImplementedError('not implemented in distribution mode.')

    @property
    def is_render(self):
        """
        Get render state.
        """
        return self._render

    @property
    def active_task_configs(self):
        """
        Get active task configs with env id as key.
        """
        if not self.is_remote:
            return self.task_config_manager.get_active_task_configs()
        else:
            import ray

            return ray.get(self.task_config_manager.get_active_task_configs.remote())

    def get_dt(self):
        """
        Get dt of simulation environment.
        """
        if not self.is_remote:
            return self._runner_list[0].runner.dt
        raise NotImplementedError('not implemented in distribution mode.')

    def get_observations(self) -> List | Any:
        """
        Get observations from Isaac environment
        """
        obs = []
        result_list = [self._runner_list[index].get_obs() for index in range(len(self._runner_list))]
        if self.is_remote:
            import ray

            result_list = ray.get(result_list)
        for _obs in result_list:
            obs.extend(_obs)
        return obs

    def close(self):
        """Close the environment"""
        if not self.is_remote:
            self._runner_list[0].runner.simulation_app.close()
        else:
            import ray

            for proxy in self._runner_list:
                ray.kill(proxy.runner)
            ray.kill(self.task_config_manager)

    @property
    def simulation_app(self):
        """Simulation app instance"""
        if not self.is_remote:
            return self._runner_list[0].runner.simulation_app
        raise NotImplementedError('not implemented in distribution mode.')

    def finished(self) -> bool:
        """Check if all tasks are finished"""
        if not self.is_remote:
            return len(self._runner_list[0].runner.current_tasks) == 0
        raise NotImplementedError('not implemented in distribution mode.')
