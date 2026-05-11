import os


class SaveAsset:
    def __init__(self, stage):
        self.stage = stage

    def save_entire_scene(self, target_folder: str, robot_save_bool: bool, filename: str = 'saved_scene.usd'):
        import omni
        from omni.isaac.core import SimulationContext

        try:
            sim = SimulationContext.instance()
            sim.pause()
            if not robot_save_bool:
                self.deleted_robots = self.delete_robots(self.stage.GetPseudoRoot())
            os.makedirs(target_folder, exist_ok=True)
            save_path = os.path.join(target_folder, filename)
            self.stage.GetRootLayer().Export(save_path)
            print(f'asset save as {save_path}')
            sim.stop()
            omni.kit.app.get_app().shutdown()
        except Exception as e:
            print(f'Delete error: {str(e)}')

    def delete_robots(self, root_prim):
        from pxr import Sdf, Usd

        deleted_robots = []
        for prim in Usd.PrimRange(root_prim):
            prim_path = prim.GetPath().pathString
            if 'robots' in prim_path:
                deleted_robots.append(prim_path)
                prim.SetActive(False)
                print(f'Deleted robot: {prim_path}')

        for prim_path in deleted_robots:
            self.stage.RemovePrim(Sdf.Path(prim_path))
            print(f'Deleted robot: {prim_path}')
