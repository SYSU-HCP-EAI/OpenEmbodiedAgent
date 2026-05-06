class HandControl:
    def __init__(self, stage):
        self.stage = stage

    def add_hand_to_stage(self, hand_path, hand_prim_path, hand_scale):
        from pxr import Gf, UsdGeom

        try:
            # Create a Prim and add a USD reference.
            asset_prim = self.stage.DefinePrim(hand_prim_path)
            if not asset_prim.GetReferences().AddReference(hand_path):
                raise RuntimeError(f'Failed to add reference: {hand_path}')

            # Set the scale.
            xform = UsdGeom.Xformable(asset_prim)
            scale_op = xform.AddXformOp(UsdGeom.XformOp.TypeScale)
            scale_op.Set(Gf.Vec3f(hand_scale))

            print('Successfully added asset with scale')
            return True
        except Exception as e:
            print(f'Error: {str(e)}')
            return False

    def delete_hand(self, left_prim_path, right_prim_path):
        self.delete_asset(left_prim_path)
        self.delete_asset(right_prim_path)

    def delete_asset(self, prim_path):
        print(prim_path)
        try:
            if self.stage.GetPrimAtPath(prim_path).IsValid():
                # Delete the operation of the hand in the virtual scene.
                success = self.stage.RemovePrim(prim_path)
                print(f'Deleted {prim_path} : {success}')
                return success
            else:
                print(f'Prim {prim_path} does not exist')
                return False
        except Exception as e:
            print(f'Delete error: {str(e)}')
            return False
