import os
from typing import Optional, Sequence

import numpy as np

from internutopia.core.object import BaseObject
from internutopia.core.scene.scene import IScene
from internutopia_extension.configs.objects import UsdObjCfg


@BaseObject.register('UsdObject')
class UsdObject(BaseObject):
    def __init__(self, config: UsdObjCfg, scene: IScene):
        super().__init__(config, scene)
        self._config = config

    def set_up_to_scene(self, scene: IScene):
        from omni.isaac.core.prims import GeometryPrim, RigidPrim
        from omni.isaac.core.utils.prims import is_prim_path_valid
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.physx.scripts import utils

        class RigidObject(RigidPrim):
            def __init__(
                self,
                prim_path: str,
                usd_path: str,
                name: str = 'custom_obj',
                position: Optional[np.ndarray] = None,
                translation: Optional[np.ndarray] = None,
                orientation: Optional[np.ndarray] = None,
                scale: Optional[np.ndarray] = None,
                visible: Optional[bool] = None,
                mass: Optional[float] = None,
                density: Optional[float] = None,
                linear_velocity: Optional[Sequence[float]] = None,
                angular_velocity: Optional[Sequence[float]] = None,
                collider: Optional[bool] = True,
            ) -> None:
                if not is_prim_path_valid(prim_path):
                    if mass is None:
                        mass = 1
                prim = add_reference_to_stage(os.path.abspath(usd_path), prim_path)
                if collider:
                    utils.setCollider(prim, approximationShape=None)
                RigidPrim.__init__(
                    self,
                    prim_path=prim_path,
                    name=name,
                    position=position,
                    translation=translation,
                    orientation=orientation,
                    scale=scale,
                    visible=visible,
                    mass=mass,
                    density=density,
                    linear_velocity=linear_velocity,
                    angular_velocity=angular_velocity,
                )

        class GeometryObject(GeometryPrim):
            def __init__(
                self,
                prim_path: str,
                usd_path: str,
                name: str = 'visual_cube',
                position: Optional[Sequence[float]] = None,
                translation: Optional[Sequence[float]] = None,
                orientation: Optional[Sequence[float]] = None,
                scale: Optional[Sequence[float]] = None,
                visible: Optional[bool] = None,
                color: Optional[np.ndarray] = None,
                size: Optional[float] = None,
            ) -> None:
                add_reference_to_stage(usd_path, prim_path)
                self.size = size
                self.color = color
                GeometryPrim.__init__(
                    self,
                    prim_path=prim_path,
                    name=name,
                    position=position,
                    translation=translation,
                    orientation=orientation,
                    scale=scale,
                    visible=visible,
                    collision=False,
                )

        if self._config.collider:
            scene.add(
                RigidObject(
                    usd_path=self._config.usd_path,
                    prim_path=self._config.prim_path,
                    name=self._config.name,
                    position=self._config.position,
                    orientation=self._config.orientation,
                    scale=self._config.scale,
                )
            )
        else:
            scene.add(
                GeometryObject(
                    usd_path=self._config.usd_path,
                    prim_path=self._config.prim_path,
                    name=self._config.name,
                    position=self._config.position,
                    orientation=self._config.orientation,
                    scale=self._config.scale,
                )
            )
