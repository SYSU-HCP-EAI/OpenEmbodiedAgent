import json
from typing import Dict, List, Optional, Tuple, Union

from internutopia.core.config import Config, DistributedConfig, TaskCfg
from internutopia.core.robot.rigid_body import IRigidBody

# Init
from internutopia.core.scene.scene import IScene
from internutopia.core.task.task import BaseTask, create_task
from internutopia.core.task_config_manager.base import BaseTaskConfigManager
from internutopia.core.util import extensions_utils, log
from internutopia.core.util.clear_task import clear_stage_by_prim_path


class SimulatorRunner:
    def __init__(self, config: Config, task_config_manager: BaseTaskConfigManager):
        self.config = config
        self.task_config_manager = task_config_manager
        self.env_num = self.config.env_num
        if isinstance(self.config, DistributedConfig):
            self.runner_id = self.config.distribution_config.runner_id
            extensions_utils.reload_extensions(self.config.distribution_config.extensions)
        self.setup_isaacsim()
        self.metrics_config = None
        self.metrics_save_path = self.config.metrics_save_path
        if self.metrics_save_path != 'console':
            try:
                with open(self.metrics_save_path, 'w'):
                    pass
            except Exception as e:
                log.error(f'Can not create result file at {self.metrics_save_path}.')
                raise e
        self.create_world()
        self._scene = IScene.create()
        self._stage = self._world.stage

        # Map task_name -> env_id:
        self.task_name_to_env_id_map = {}
        self.env_id_to_task_name_map = {}

        # finished_tasks contains all the finished tasks in current tasks dict
        self.finished_tasks = set()

        self.render_interval = (
            self.config.simulator.rendering_interval if self.config.simulator.rendering_interval is not None else 5
        )
        log.info(f'rendering interval: {self.render_interval}')
        self.render_trigger = 0
        self.loop = False
        self._render = False

    @property
    def current_tasks(self) -> dict[str, BaseTask]:
        return self._world._current_tasks

    def warm_up(self, steps: int = 10, render: bool = True, physics: bool = True):
        """
        Warm up the simulation by running a specified number of steps.

        Args:
            steps (int): The number of warm-up steps to perform. Defaults to 10.
            render (bool): Whether to render the scene during warm-up. Defaults to True.
            physics (bool): Whether to enable physics during warm-up. Defaults to True.

        Raises:
            ValueError: If both `render` and `physics` are set to False, or if `steps` is less than or equal to 0.
        """
        from omni.isaac.core.simulation_context import SimulationContext

        if not render and not physics:
            raise ValueError('both `render` and `physics` are set to False')
        if steps <= 0:
            raise ValueError('steps` is less than or equal to 0')
        if physics:
            for _ in range(steps):
                self._world.step(render=render)
        else:
            for _ in range(steps):
                SimulationContext.render(self._world)

    def step(
        self, actions: Union[List[Dict], None] = None, render: bool = True
    ) -> Tuple[List[Dict], List[bool], List[float]]:
        """
        Step function to advance the simulation environment by one time step.

        This method processes actions for active tasks, steps the simulation world,
        collects observations, updates metrics, and determines task terminations. It also
        handles rendering based on specified intervals.

        Args:
            actions (Union[List[Dict], None], optional): A dictionary mapping task names to
                another dictionary of robot names and their respective actions. If None,
                no actions are applied. Defaults to None.
            render (bool, optional): Flag indicating whether to render the simulation
                at this step. True triggers rendering if the render interval is met.
                Defaults to True.

        Returns:
            Tuple[Dict, Dict[str, bool], Dict[str, float]]:
                - obs (Dict): A dictionary containing observations for each task,
                  further divided by robot names and their observation data.
                - terminated_status (Dict[str, bool]): A dictionary mapping task names
                  to boolean values indicating whether the task has terminated.
                - reward (Dict[str, float]): A dictionary that would contain rewards
                  for each task or robot; however, the actual return and computation of
                  rewards is not shown in the provided code snippet.

        Raises:
            Exception: If an error occurs when applying an action to a robot, the
                exception is logged and re-raised, providing context about the task,
                robot, and current tasks state.

        Notes:
            - The `_world.step()` method advances the simulation, optionally rendering
              the environment based on the `render` flag and the render interval.
            - `get_obs()` is a method to collect observations from the simulation world,
              though its implementation details are not shown.
            - Metrics for each task are updated, and upon task completion, results are
              saved to a JSON file. This includes a flag 'normally_end' set to True,
              which seems to indicate normal termination of the task.
            - The function also manages a mechanism to prevent further action application
              and metric updates for tasks that have been marked as finished.
        """
        """ ================ TODO: Key optimization interval ================= """
        terminated_status = []
        reward = []

        for env_id, action_dict in enumerate(actions):
            # terminated tasks will no longer apply action
            if env_id not in self.env_id_to_task_name_map:
                continue

            task_name = self.env_id_to_task_name_map[env_id]

            if task_name in self.finished_tasks:
                continue

            if task_name not in self.current_tasks:
                continue

            task = self.current_tasks.get(task_name)
            for name, action in action_dict.items():
                if name in task.robots:
                    try:
                        task.robots[name].apply_action(action)
                    except Exception as e:
                        log.error('task_name     : %s', task_name)
                        log.error('robot_name    : %s', name)
                        log.error('current_tasks : %s', [i for i in self.current_tasks.keys()])
                        raise e

        self.render_trigger += 1
        self._render = render and self.render_trigger > self.render_interval
        if self.render_trigger > self.render_interval:
            self.render_trigger = 0

        # 始终先只做物理步；再在「本帧可能发生的渲染之前」跑各机器人的 post_physics（例如 XLe 用 USD
        # 把可视壳体对齐到 root）。若用 World.step(render=True)，Kit 往往在 step 内部已渲染完才返回，
        # 此时再改 USD 视口仍显示脱节位姿，看起来「完全不动」。
        self._world.step(render=False)

        for task in self.current_tasks.values():
            if task.name in self.finished_tasks:
                continue
            for robot in task.robots.values():
                hook = getattr(robot, "post_physics_step", None)
                if callable(hook):
                    try:
                        hook()
                    except Exception as exc:
                        log.warning("robot post_physics_step failed (%s): %s", getattr(robot, "name", robot), exc)

        if self._render:
            try:
                self._world.render()
            except AttributeError:
                try:
                    from omni.isaac.core.simulation_context import SimulationContext

                    SimulationContext.render(self._world)
                except Exception as exc:
                    log.warning("render after post_physics_step failed (SimulationContext.render): %s", exc)
            except Exception as exc:
                log.warning("world.render failed: %s", exc)

        # Get obs
        obs = self.get_obs()

        # update metrics
        for task in self.current_tasks.values():
            if task.is_done():
                self.finished_tasks.add(task.name)
                log.info(f'Task {task.name} finished.')
                metrics_results = task.calculate_metrics()
                if self.metrics_save_path == 'console':
                    print(json.dumps(metrics_results, indent=4))
                elif self.metrics_save_path == 'none' or self.metrics_save_path is None:
                    pass
                else:
                    with open(self.metrics_save_path, 'a') as f:
                        f.write(json.dumps(metrics_results))
                        f.write('\n')

            # finished tasks will no longer update metrics
            if task.name not in self.finished_tasks:
                for metric in task.metrics.values():
                    metric.update(obs[self.task_name_to_env_id_map[task.name]])

        # update terminated_status and rewards
        for env_id in range(self.env_num):
            # terminated tasks will no longer apply action

            if env_id not in self.env_id_to_task_name_map:
                terminated_status.append(True)
                reward.append(-1)
                continue

            task_name = self.env_id_to_task_name_map[env_id]

            if task_name in self.finished_tasks:
                terminated_status.append(True)
                reward.append(-1)
            else:
                terminated_status.append(False)
                r = self.current_tasks[task_name].reward
                reward.append(r.calc() if r is not None else -1)

        return obs, terminated_status, reward

    def get_obs(self) -> List[Dict | None]:
        """
        Get obs
        Returns:
            List[Dict]: obs from isaac sim.
        """
        obs = {}
        for task_name, task in self.current_tasks.items():
            obs[task_name] = task.get_observations()
        # Add render obs
        for task_name, task_obs in obs.items():
            for robot_name, robot_obs in task_obs.items():
                obs[task_name][robot_name]['render'] = self._render

        _obs = []
        for env_id in range(self.env_num):
            if env_id in self.env_id_to_task_name_map:
                _obs.append(obs[self.env_id_to_task_name_map[env_id]])
            else:
                _obs.append(None)
        return _obs

    def stop(self):
        """
        Stop all current operations and clean up the **World**
        """
        self._world.reset()
        self._world.clear()
        self._world.stop()

    def get_current_time_step_index(self) -> int:
        return self._world.current_time_step_index

    def reset(self, env_ids: Optional[List[int]] = None) -> Tuple[List, List]:
        """
        Resets the environment for the given environment IDs or initializes it if no IDs are provided.
        This method handles resetting the simulation context, generating new task configs, and finalizing
        tasks when necessary. It supports partial resets for specific environments and ensures proper
        handling of task transitions.

        Args:
            env_ids (Optional[List[int]]): A list of environment IDs to reset. If None, all environments
                are reset or initialized based on the current state.

        Returns:
            Tuple[List, List]: A tuple containing two lists. The first list contains observations for
                the reset environments, and the second list contains the new task configs.

        Raises:
            ValueError: If the provided `env_ids` are invalid or don't correspond to any existing tasks.
            RuntimeError: If the simulation context fails to reset or initialize properly.

        Notes:
            - If `env_ids` is None and there are current tasks, all environments are reset.
            - If `env_ids` is None and there are no current tasks, the simulation context is initialized.
            - Observations are collected only for environments that are reset or initialized.
            - Tasks corresponding to the reset environments are transitioned to new episodes.
        """
        from omni.isaac.core.simulation_context import SimulationContext

        new_task_configs = []

        if env_ids is None and self.current_tasks:
            # reset
            log.info('==================== reset all env ====================')
            env_ids = [i for i in range(self.env_num)]

        if env_ids is None:
            # init
            log.info('===================== init reset =====================')
            SimulationContext.reset(self._world, soft=False)
            new_task_configs = self._next_episodes()
        else:
            # switch to next episodes
            env_to_reset = [env_id for env_id in env_ids if env_id in self.env_id_to_task_name_map]

            if not env_to_reset:
                log.warning(f'Not reset empty envs: {env_ids}.')
                return [None for _ in env_ids], [None for _ in env_ids]

            tasks = [self.env_id_to_task_name_map[env_id] for env_id in env_to_reset]
            _task_configs = self._next_episodes(tasks)
            for env_id in env_ids:
                if env_id in self.env_id_to_task_name_map:
                    new_task_configs.append(_task_configs.pop(0))
                else:
                    new_task_configs.append(None)
            [self.finished_tasks.discard(task) for task in tasks]

        all_obs = self.get_obs()
        obs = [all_obs[i] for i in env_ids] if env_ids else all_obs

        if not self.current_tasks:
            # finished
            self._finalize()

        return obs, new_task_configs

    def _finalize(self):
        """
        Finalize the tasks and do some post-processing.
        """
        pass

    def world_clear(self):
        self._world.clear()

    def clear_single_task(self, task_name: str):
        """
        Clear single task with task_name

        Args:
            task_name (str): Task name to clear.

        """
        from omni.isaac.core.loggers import DataLogger

        if task_name not in self.current_tasks:
            log.warning(f'Clear task {task_name} fail. The task {task_name} is not in current_tasks.')
            return
        old_task = self.current_tasks[task_name]
        old_task.cleanup()
        del self.current_tasks[task_name]
        self._world._task_scene_built = False
        self._world._data_logger = DataLogger()
        log.info(f'Clear stage: /World/env_{self.task_name_to_env_id_map[task_name]}')
        clear_stage_by_prim_path(f'/World/env_{self.task_name_to_env_id_map[task_name]}')

    def _next_episodes(self, reset_tasks: List[str] = None) -> List[TaskCfg]:
        """
        Switch tasks that need to be reset to the next episode.

        This method handles cleaning-up tasks, resetting sim backend, creating new tasks, and
        restoring states for non-reset environments.

        Args:
            reset_tasks (List[str]): a list of task names that need to be reset. If None, the method
                initializes all environments without resetting specific tasks.

        Returns:
            List[TaskCfg]: a list of TaskCfg.

        Raises:
            RuntimeError: If a task specified in `reset_tasks` isn't found in the current tasks.
        """
        from isaacsim.core.simulation_manager import SimulationManager

        env_id_list = []
        env_offset_list = []
        task_name_list = []
        task_configs_list = []

        if reset_tasks:
            # recycling env_id
            for task_name in reset_tasks:
                if task_name not in self.current_tasks:
                    raise RuntimeError(f'Task with task_name {task_name} not in `current_tasks`.')
                old_task = self.current_tasks[task_name]
                env_id_list.append(old_task.env_id)

            # save the state of envs that need to be kept
            for _task_name, task in self.current_tasks.items():
                if _task_name in reset_tasks:
                    continue
                task.save_info()

            # clean up tasks that need to reset
            for task_name in reset_tasks:
                self.clear_single_task(task_name)

            # clear all rigid bodies in scene register and physics backend
            for task in self.current_tasks.values():
                task.clear_rigid_bodies()

            SimulationManager._on_stop('reset')
        else:
            # init
            SimulationManager._on_stop('reset')
            env_id_list = [None for _ in range(self.env_num)]

        # get next_task_configs
        for idx, env_id in enumerate(env_id_list):
            if not isinstance(self.config, DistributedConfig):
                next_task = self.task_config_manager.get_next(env_id)
            else:
                import ray

                next_task = ray.get(self.task_config_manager.get_next.remote(env_id, self.runner_id))
            new_task_name, new_env_id, new_env_offset, task_cfg = next_task
            if task_cfg is None and env_id in self.env_id_to_task_name_map:
                del self.env_id_to_task_name_map[env_id]
            env_id_list[idx] = new_env_id
            env_offset_list.append(new_env_offset)
            task_name_list.append(new_task_name)
            task_configs_list.append(task_cfg)

        # create tasks with new task configs
        _new_tasks = []
        _new_tasks_names = []
        for idx, task_config in enumerate(task_configs_list):
            if task_config is None:
                continue
            task = create_task(task_config, self._scene)
            task.set_up_runtime(task_name_list[idx], env_id_list[idx], env_offset_list[idx])
            self._world.add_task(task)
            _new_tasks.append(task)
            _new_tasks_names.append(task.name)
            task.set_up_scene(self._scene)

            # map task_name and env_id of new tasks
            self.task_name_to_env_id_map[task.name] = task.env_id
            self.env_id_to_task_name_map[task.env_id] = task.name

        # create sim_view
        SimulationManager._create_simulation_view('reset')

        # restore the state of envs that haven't been reset
        if reset_tasks:
            for t in self.current_tasks.values():
                if t.name in _new_tasks_names:
                    continue
                t.restore_info()

        self._scene.unwrap()._finalize(self._world.physics_sim_view)  # noqa

        # 烘焙场景可能带重力为 0 / 缺失 PhysicsScene；机器人 USD 也可能对个别刚体写了 disableGravity
        self._ensure_stage_physics_gravity()
        self._ensure_env_robots_not_disable_gravity()

        # post_reset for new tasks
        for task in _new_tasks:
            task.post_reset()

        # log new episodes
        log.info('===================== episodes ========================')
        for task in _new_tasks:
            log.info(f'Next episode: {task.name} at {str(task.env_id)}')
        log.info('======================================================')
        return task_configs_list

    def get_obj(self, name: str) -> IRigidBody:
        # Only supported in gym_env.
        # TODO: handle name by a more robust way and maybe support vec env
        return self._scene.get(name + '_0')

    def remove_collider(self, prim_path: str):
        from omni.physx.scripts import utils

        build = self._world.stage.GetPrimAtPath(prim_path)
        if build.IsValid():
            utils.removeCollider(build)

    def add_collider(self, prim_path: str):
        from omni.physx.scripts import utils

        build = self._world.stage.GetPrimAtPath(prim_path)
        if build.IsValid():
            utils.setCollider(build, approximationShape=None)

    def create_world(self):
        physics_dt = self.config.simulator.physics_dt
        rendering_dt = self.config.simulator.rendering_dt
        physics_dt = eval(physics_dt) if isinstance(physics_dt, str) else physics_dt
        self.dt = physics_dt
        rendering_dt = eval(rendering_dt) if isinstance(rendering_dt, str) else rendering_dt
        use_fabric = self.config.simulator.use_fabric
        log.info(f'simulator params: physics dt={physics_dt}, rendering dt={rendering_dt}, use_fabric={use_fabric}')
        from omni.isaac.core import World

        self._world: World = World(
            physics_dt=physics_dt,
            rendering_dt=rendering_dt,
            stage_units_in_meters=1.0,
            sim_params={'use_fabric': use_fabric},
        )

    def _ensure_stage_physics_gravity(self) -> None:
        """保证舞台上至少有一个可用的 PhysX 重力场（烘焙场景常漏设或为 0）。"""
        try:
            from pxr import Gf, Sdf, Usd, UsdPhysics
        except ImportError:
            return
        stage = getattr(self._world, "stage", None)
        if stage is None:
            return

        gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        gravity_mag = 9.81
        configured = []

        scene_cls = getattr(UsdPhysics, "Scene", None)
        if scene_cls is None:
            log.warning("_ensure_stage_physics_gravity: pxr.UsdPhysics.Scene missing, skip gravity patch")
            return

        def _write_gravity_on_scene_schema(sch) -> None:
            d_attr = sch.GetGravityDirectionAttr()
            if d_attr:
                d_attr.Set(gravity_dir)
            else:
                sch.CreateGravityDirectionAttr().Set(gravity_dir)
            m_attr = sch.GetGravityMagnitudeAttr()
            if m_attr:
                m_attr.Set(gravity_mag)
            else:
                sch.CreateGravityMagnitudeAttr().Set(gravity_mag)

        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            if not prim.IsValid():
                continue
            try:
                is_ps = prim.IsA(scene_cls)
            except Exception:
                is_ps = False
            if not is_ps and prim.GetTypeName() != "PhysicsScene":
                continue
            try:
                sch = scene_cls(prim)
            except Exception:
                continue
            if not sch:
                continue
            _write_gravity_on_scene_schema(sch)
            configured.append(str(prim.GetPath()))

        if not configured:
            path = Sdf.Path("/World/physicsScene")
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                sch = scene_cls.Define(stage, path)
            else:
                sch = scene_cls(prim)
            _write_gravity_on_scene_schema(sch)
            configured.append(str(path))

        if configured:
            log.info(
                "physics gravity ensured: magnitude=%s m/s^2, direction=(0,0,-1), scene_prims=%s",
                gravity_mag,
                configured,
            )

    def _ensure_env_robots_not_disable_gravity(self) -> None:
        """清除 /World/env_*/robots 下 PhysX 刚体的 disableGravity，避免整机「悬停」。"""
        try:
            from pxr import PhysxSchema, Usd
        except ImportError:
            return
        stage = getattr(self._world, "stage", None)
        if stage is None:
            return
        roots = []
        for i in range(max(1, int(self.env_num))):
            p = stage.GetPrimAtPath(f"/World/env_{i}/robots")
            if p.IsValid():
                roots.append(p)
        cleared = []
        for root in roots:
            for prim in Usd.PrimRange(root):
                if not prim.IsValid() or not prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                    continue
                api = PhysxSchema.PhysxRigidBodyAPI(prim)
                attr = api.GetDisableGravityAttr()
                if attr and attr.HasAuthoredValue() and attr.Get():
                    attr.Set(False)
                    cleared.append(str(prim.GetPath()))
        if cleared:
            log.info("cleared PhysxRigidBodyAPI.disableGravity on: %s", cleared)

    def setup_isaacsim(self):
        # Init Isaac Sim
        from isaacsim import SimulationApp  # noqa

        headless = self.config.simulator.headless
        native = self.config.simulator.native
        webrtc = self.config.simulator.webrtc
        self._simulation_app = SimulationApp(
            {'headless': headless, 'anti_aliasing': 0, 'hide_ui': False, 'multi_gpu': False}
        )
        self._simulation_app._carb_settings.set('/physics/cooking/ujitsoCollisionCooking', False)
        log.debug('SimulationApp init done')

        # Determine streaming mode based on isaacsim version.
        try:
            from isaacsim import util  # noqa
        except ImportError:
            # isaacsim 420 behavior
            self.setup_streaming_420(native, webrtc)
        else:
            # isaac 450 behavior
            if native:
                log.warning('native streaming is DEPRECATED, webrtc streaming is used instead')
            webrtc = native or webrtc
            self.setup_streaming_450(webrtc)

    def setup_streaming_420(self, native: bool, webrtc: bool):
        if webrtc:
            from omni.isaac.core.utils.extensions import enable_extension  # noqa

            self._simulation_app.set_setting('/app/window/drawMouse', True)
            self._simulation_app.set_setting('/app/livestream/proto', 'ws')
            self._simulation_app.set_setting('/app/livestream/websocket/framerate_limit', 60)
            self._simulation_app.set_setting('/ngx/enabled', False)
            enable_extension('omni.services.streamclient.webrtc')

        elif native:
            from omni.isaac.core.utils.extensions import enable_extension  # noqa

            self._simulation_app.set_setting('/app/window/drawMouse', True)
            self._simulation_app.set_setting('/app/livestream/proto', 'ws')
            self._simulation_app.set_setting('/app/livestream/websocket/framerate_limit', 120)
            self._simulation_app.set_setting('/ngx/enabled', False)
            enable_extension('omni.kit.streamsdk.plugins-3.2.1')
            enable_extension('omni.kit.livestream.core-3.2.0')
            enable_extension('omni.kit.livestream.native')

    def setup_streaming_450(self, webrtc: bool):
        if webrtc:
            from omni.isaac.core.utils.extensions import enable_extension

            self._simulation_app.set_setting('/app/window/drawMouse', True)
            enable_extension('omni.kit.livestream.webrtc')

    @property
    def simulation_app(self):
        return self._simulation_app
