import traceback
from abc import ABC
from typing import Any, Dict, List, Union

from internutopia.core.config import TaskCfg
from internutopia.core.object import init_objects
from internutopia.core.robot.rigid_body import IRigidBody
from internutopia.core.robot.robot import BaseRobot, init_robots
from internutopia.core.scene.scene import IScene
from internutopia.core.task.metric import BaseMetric, create_metric
from internutopia.core.util import log
from internutopia.core.util.pose_mixin import PoseMixin


class BaseTask(ABC):
    """
    wrap of omniverse isaac sim's base task

    * enable register for auto register task
    * contains scene/robots/objects.
    """

    tasks = {}

    def __init__(self, config: TaskCfg, scene: IScene):
        self.name = None
        self.env_id = 0
        self.env_offset = []
        self.root_path = None

        self.scene_prim = None
        self.objects = None
        self.robots: Union[Dict[str, BaseRobot], None] = None
        self._scene = scene
        self.scene_rigid_bodies: Dict[str, IRigidBody] = {}
        self.config = config

        self.metrics: Dict[str, BaseMetric] = {}
        self.steps = 0
        self.work = True
        self.loaded = False
        for metric_config in config.metrics:
            self.metrics[metric_config.name] = create_metric(metric_config, self.config)

        from internutopia.core.task.reward import BaseReward, create_reward  # noqa

        self.reward: Union[BaseReward, None] = create_reward(config.reward, self) if config.reward is not None else None

    def set_up_runtime(self, task_name, env_id, env_offset):
        """
        Sets up info (task_name, env_id, env_offset) for this task.

        TODO: refactor and rename

        Args:
            task_name (str): The name of the task.
            env_id (int): The env ID of this task.
            env_offset (List[float]): The env offset for the task.
        """
        self.env_id: int = env_id
        self.env_offset: List[float] = env_offset
        self.root_path = f'/World/env_{str(self.env_id)}'
        if env_id not in PoseMixin.env_offset_map:
            PoseMixin.env_offset_map[str(env_id)] = env_offset
            log.info(f'env {env_id} at {env_offset}')
        self.name = task_name
        for metric in self.metrics.values():
            metric.set_up_runtime(task_name, env_id, env_offset)

    def load(self):
        """

        Loads the environment scene and initializes robots and objects.

        This method first checks if a scene asset path is defined in the task config.
        If so, it creates a scene using the provided path and specified parameters such as scaling and positioning.
        The scene is then populated with robots and objects based on the configurations stored within `self.config`.

        Raises:
        - Exceptions may be raised during file operations or USD scene creation, but specific exceptions are not documented here.

        Attributes Modified:
        - **self.robots**: A collection of initialized robots set up within the scene.
        - **self.objects**: A dictionary mapping object names to their respective initialized instances within the scene.
        - **self.loaded**: A boolean flag indicating whether the environment has been successfully loaded, set to `True` upon successful completion of this method.

        Logs:
        - Information about the initialized robots and objects is logged using the `log.info` method after successful setup.
        """
        from pxr import Usd

        if self.config.scene_asset_path is not None:
            self._scene.load(self.config, self.env_id, self.env_offset)
            for prim in Usd.PrimRange.AllPrims(self._scene.scene_prim):
                if prim.GetAttribute('physics:rigidBodyEnabled').Get():
                    log.debug(f'[BaseTask.load] found rigid body at path: {prim.GetPath()}')
                    try:
                        _rb = IRigidBody.create(prim_path=str(prim.GetPath()), name=str(prim.GetPath()))
                        self.scene_rigid_bodies[str(prim.GetPath())] = _rb
                    except Exception as e:
                        log.error(f'Fail to create IRigidBody at {prim.GetPath()}: {e}')

            # 在生成 articulation 之前补全静态 Mesh 碰撞；reset 之后再改 USD 会破坏 PhysX / 机器人仍悬空
            if getattr(self.config, "enable_static_scene_mesh_collision_patch", False):
                self._patch_static_scene_mesh_collisions_for_physx()

        self.robots = init_robots(self.config, self._scene)
        self.objects = init_objects(self.config, self._scene)
        self.loaded = True

    def _patch_static_scene_mesh_collisions_for_physx(self) -> None:
        """为场景根下 Mesh 添加碰撞与 convexHull 近似，供 Merom 等烘焙地台与 PhysX 建立接触。"""
        try:
            from pxr import PhysxSchema, Usd, UsdPhysics
        except ImportError:
            log.warning("_patch_static_scene_mesh_collisions_for_physx: pxr not available, skip")
            return
        scene_prim = getattr(self._scene, "scene_prim", None)
        if scene_prim is None or not scene_prim.IsValid():
            log.warning("collision patch: invalid scene_prim, skip")
            return
        inst_count = 0
        mesh_count = 0
        for prim in Usd.PrimRange.AllPrims(scene_prim):
            if prim.IsInstance():
                prim.SetInstanceable(False)
                inst_count += 1
        for prim in Usd.PrimRange.AllPrims(scene_prim):
            if prim.GetTypeName() != "Mesh":
                continue
            try:
                UsdPhysics.CollisionAPI.Apply(prim)
                physx = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                if hasattr(physx, "CreateApproximationAttr"):
                    physx.CreateApproximationAttr().Set("convexHull")
                mesh_count += 1
            except Exception:
                continue
        log.info(
            "scene mesh collision patch (pre-robot): meshes=%s, instances_uninstanced=%s env_id=%s",
            mesh_count,
            inst_count,
            self.env_id,
        )

    def clear_rigid_bodies(self):
        for rigid_body_name in self.scene_rigid_bodies.keys():
            if self._scene.object_exists(rigid_body_name):
                self._scene.remove(target=rigid_body_name)

    def save_info(self):
        """
        Saves the robot information and rigidbody statuses.
        """
        self.save_robot_info()
        self._save_rigidbody_statuses()

    def _save_rigidbody_statuses(self):
        """
        Saves the current status of all rigid bodies in the scene by querying their physics properties excluding
        those in the robot.

        Note:
            rigid prims within articulations aren't included since those RigidBody' physical
            status (transform, velocity, etc) can't be set individually.
        """
        for rigid_body_name, rigid_body in self.scene_rigid_bodies.items():
            if not self._scene.object_exists(rigid_body_name):
                log.error(f'[cache_info] {rigid_body_name} does not exist.')
                continue
            rigid_body.save_status()

    def _restore_rigidbody_statuses(self):
        """
        Restores the statuses of all rigid bodies in the scene based on their stored status data excluding
        those in the robot.
        """
        for rigid_body_name, rigid_body in self.scene_rigid_bodies.items():
            if rigid_body.status is None or not self._scene.object_exists(rigid_body_name):
                continue
            rigid_body.restore_status()

    def set_up_scene(self, scene: IScene) -> None:
        """
        Adding assets to the stage as well as adding the encapsulated objects such as XFormPrim..etc
        to the task_objects happens here.

        Args:
            scene (Scene): [description]
        """
        self._scene = scene
        if not self.loaded:
            self.load()

    def get_observations(self) -> Dict[str, Any]:
        """
        Returns current observations from the objects needed for the behavioral layer.

        Return:
            Dict[str, Any]: observation of robots in this task
        """
        if not self.work:
            return {}
        obs = {}
        for robot_name, robot in self.robots.items():
            try:
                _obs = robot.get_obs()
                if _obs:
                    obs[robot_name] = _obs
            except Exception as e:
                log.error(self.name)
                log.error(e)
                traceback.print_exc()
                return {}
        return obs

    def update_metrics(self):
        """

        Updates all metrics stored within the instance.

        Scans through the dictionary of metrics kept by the current instance,
        invoking the 'update' method on each one. This facilitates the aggregation
        or recalculation of metric values as needed.

        Note:
        This method does not return any value; its purpose is to modify the state
        of the metric objects internally.
        """
        for _, metric in self.metrics.items():
            metric.update()

    def calculate_metrics(self) -> dict:
        """

        Calculates and aggregates the results of all metrics registered within the instance.

        This method iterates over the stored metrics, calling their respective `calc` methods to compute
        the metric values. The computed values are then compiled into a dictionary, where each key corresponds
        to the metrics' name, and each value is the result of the metric calculation.

        Returns:
            dict: A dictionary containing the calculated results of all metrics, with metric names as keys.

        Note:
            Ensure that all metrics added to the instance have a `calc` method implemented.

        Example Usage:
        ```python
        # Assuming `self.metrics` is populated with metric instances.
        results = calculate_metrics()
        print(results)
        # Output: {'metric1': 0.85, 'metric2': 0.92, 'metric3': 0.78}
        ```
        """
        metrics_res = {}
        for name, metric in self.metrics.items():
            metrics_res[name] = metric.calc()

        return metrics_res

    def is_done(self) -> bool:
        """
        Returns True of the task is done. The result should be decided by the state of the task.
        """
        raise NotImplementedError

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """
        Called before stepping the physics simulation.

        Args:
            time_step_index (int): [description]
            simulation_time (float): [description]
        """
        self.steps += 1
        return

    def save_robot_info(self):
        """
        Saves information of all robots in the task instance.
        """
        for robot in self.robots.values():
            robot.save_robot_info()

    def restore_info(self):
        """
        Restores the information and statuses of rigid bodies and robots.
        """
        self._restore_rigidbody_statuses()
        for robot in self.robots.values():
            robot.restore_robot_info()

    def post_reset(self) -> None:
        """Calls while doing a .reset() on the world."""
        self.steps = 0
        for robot in self.robots.values():
            robot.post_reset()
        # TODO: Verify whether RigidPrims' post_reset need to be called
        return

    def cleanup(self) -> None:
        """
        Used to clean up the resources loaded in the task.
        """
        for obj in self.objects.values():
            # Using try here because we want to ignore all exceptions
            try:
                self._scene.remove(obj.name)
            finally:
                log.info('[cleanup] objs cleaned.')
        for robot in self.robots.values():
            # Using try here because we want to ignore all exceptions
            log.info(f'[cleanup] cleanup robot {robot.articulation.name}')
            try:
                robot.cleanup()
                self._scene.remove(robot.articulation.name, registry_only=True)
            finally:
                log.info('[cleanup] robots cleaned.')

    @classmethod
    def register(cls, name: str):
        """
        Register an task class with the given name(decorator).

        Args:
            name(str): name of the task
        """

        def wrapper(task_class):
            """
            Register the task class.
            """
            cls.tasks[name] = task_class
            return task_class

        return wrapper


def create_task(config: TaskCfg, scene: IScene):
    task_cls: BaseTask = BaseTask.tasks[config.type](config, scene)
    return task_cls
