import numpy as np

from internutopia.core.object import BaseObject
from internutopia.core.scene.scene import IScene
from internutopia_extension.configs.objects import DynamicCubeCfg


@BaseObject.register('DynamicCube')
class DynamicCube(BaseObject):
    def __init__(self, config: DynamicCubeCfg, scene: IScene):
        super().__init__(config, scene)
        self._config = config

    def set_up_to_scene(self, scene: IScene):
        from omni.isaac.core.objects import DynamicCuboid

        cube_kwargs = {
            "prim_path": self._config.prim_path,
            "name": self._config.name,
            "position": np.array(self._config.position),
            "orientation": np.array(self._config.orientation),
            "scale": np.array(self._config.scale),
            "color": np.array(self._config.color),
        }
        if getattr(self._config, "mass", None) is not None:
            cube_kwargs["mass"] = float(self._config.mass)
        if getattr(self._config, "density", None) is not None:
            cube_kwargs["density"] = float(self._config.density)

        scene.add(
            DynamicCuboid(**cube_kwargs)
        )
        self._apply_contact_material()

    def _apply_contact_material(self) -> None:
        static_friction = getattr(self._config, "static_friction", None)
        dynamic_friction = getattr(self._config, "dynamic_friction", None)
        restitution = getattr(self._config, "restitution", None)
        if static_friction is None and dynamic_friction is None and restitution is None:
            return
        try:
            import omni.usd  # type: ignore
            from pxr import PhysxSchema, UsdPhysics, UsdShade  # type: ignore
        except Exception:
            return

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        material_path = f"{self._config.prim_path}/PhysicsMaterial"
        material = UsdShade.Material.Define(stage, material_path)
        material_prim = material.GetPrim()
        if not material_prim or not material_prim.IsValid():
            return

        mat_api = PhysxSchema.PhysxMaterialAPI.Apply(material_prim)
        if static_friction is not None:
            attr = mat_api.GetStaticFrictionAttr() or mat_api.CreateStaticFrictionAttr()
            attr.Set(float(static_friction))
        if dynamic_friction is not None:
            attr = mat_api.GetDynamicFrictionAttr() or mat_api.CreateDynamicFrictionAttr()
            attr.Set(float(dynamic_friction))
        if restitution is not None:
            attr = mat_api.GetRestitutionAttr() or mat_api.CreateRestitutionAttr()
            attr.Set(float(restitution))

        root_prim = stage.GetPrimAtPath(self._config.prim_path)
        if not root_prim or not root_prim.IsValid():
            return

        bound_any = False
        for prim in Usd.PrimRange(root_prim):
            if not prim.IsValid() or not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            binding_api = (
                UsdShade.MaterialBindingAPI(prim)
                if prim.HasAPI(UsdShade.MaterialBindingAPI)
                else UsdShade.MaterialBindingAPI.Apply(prim)
            )
            binding_api.Bind(
                material,
                bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                materialPurpose="physics",
            )
            bound_any = True

        # Fallback: if no explicit collider API is found, bind on root prim.
        if not bound_any:
            binding_api = (
                UsdShade.MaterialBindingAPI(root_prim)
                if root_prim.HasAPI(UsdShade.MaterialBindingAPI)
                else UsdShade.MaterialBindingAPI.Apply(root_prim)
            )
            binding_api.Bind(
                material,
                bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                materialPurpose="physics",
            )
