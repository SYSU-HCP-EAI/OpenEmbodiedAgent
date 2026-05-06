import os
from typing import List

from internutopia.core.config import TaskCfg
from internutopia.core.robot.rigid_body import IRigidBody
from internutopia.core.scene import validate_scene_file
from internutopia.core.scene.scene import IScene


class IsaacsimScene(IScene):
    """IsaacSim's implementation on `IScene` class."""

    def __init__(self):
        from omni.isaac.core import World
        from omni.isaac.core.scenes import Scene

        self._scene: Scene = World.instance().scene

    def load(self, task_config: TaskCfg, env_id: int, env_offset: List[float]):
        """See `IScene.load` for documentation."""
        usd_path = os.path.abspath(task_config.scene_asset_path)
        prim_path_root = f'World/env_{env_id}/scene'
        source, prim_path = validate_scene_file(usd_path, prim_path_root)

        from omni.isaac.core.utils.prims import create_prim

        position = [env_offset[idx] + i for idx, i in enumerate(task_config.scene_position)]
        scene_prim = create_prim(prim_path, usd_path=source, scale=task_config.scene_scale, translation=position)
        self.scene_prim = scene_prim

    def add(self, target: any):
        """See `IScene.add` for documentation."""
        if hasattr(target, 'initialize') and hasattr(target, 'unwrap'):
            # TODO: Implement initialize method on IArticulation._articulation to make
            # 'self._scene._scene_registry.add_articulated_system' -> 'self._scene.add'
            self._scene._scene_registry.add_articulated_system(name=target.name, articulated_system=target)
        elif hasattr(target, 'unwrap'):
            self._scene.add(target.unwrap())
        else:
            # For instance of isaac-sim native classes
            self._scene.add(target)

    def remove(self, target: any, registry_only: bool = False):
        """See `IScene.remove` for documentation."""
        self._scene.remove_object(name=target, registry_only=registry_only)

    def object_exists(self, target: any) -> bool:
        """See `IScene.object_exists` for documentation."""
        return self._scene.object_exists(target)

    def get(self, target: any) -> IRigidBody:
        """See `IScene.get` for documentation."""
        object = self._scene.get_object(target)
        return IRigidBody.create(prim_path=object.prim_path, name=object.prim_path)

    def unwrap(self):
        """See `IScene.unwrap` for documentation."""
        return self._scene
