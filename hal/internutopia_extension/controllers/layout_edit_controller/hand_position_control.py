class HandPositionControl:
    """
    Control hand position change.
    """

    def __init__(self, stage, right_hand_prim_path, left_hand_prim_path, object_scale):
        self.stage = stage
        self.right_hand_prim_path = right_hand_prim_path
        self.left_hand_prim_path = left_hand_prim_path
        self.object_scale = object_scale

    def update_object_hand_pose(self, hand_position, hand_orientation, hand='right'):
        """
        Update the objects in the scene based on the hand position/rotation obtained from gesture recognition.
        hand_position: np.array([x, y, z]), Current hand position
        hand_orientation: 3x3 Rotation matrix
        hand: 'right' or 'left', Indicate whether it is the right hand or the left hand.
        """
        from pxr import Gf, UsdGeom

        prim_path = self.right_hand_prim_path if hand == 'right' else self.left_hand_prim_path
        prim = self.stage.GetPrimAtPath(prim_path)

        if not prim:
            print(f'{hand}_hand_prim is not valid.')
            return

        try:
            translation = Gf.Vec3d(hand_position[0], hand_position[1], hand_position[2])

            attached_xform = UsdGeom.Xformable(prim)
            attached_xform.ClearXformOpOrder()
            attached_xform.AddTranslateOp().Set(translation)

            if hand == 'right':
                quatf_rotation = Gf.Quatd(
                    hand_orientation[0] - 0.27,
                    hand_orientation[1] - 0.55,
                    hand_orientation[2] - 0.04,
                    hand_orientation[3] - 0.11,
                )
                quatf_rotation = Gf.Quatd(quatf_rotation.GetReal(), Gf.Vec3d(*quatf_rotation.GetImaginary()))
            else:
                quatf_rotation = Gf.Quatd(
                    hand_orientation[0] - 0.286,
                    hand_orientation[1] + 0.74,
                    hand_orientation[2] - 0.08,
                    hand_orientation[3] - 0.07,
                )
                quatf_rotation = Gf.Quatd(quatf_rotation.GetReal(), Gf.Vec3d(*quatf_rotation.GetImaginary()))
            attached_xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quatf_rotation)
            attached_xform.AddScaleOp().Set(self.object_scale)

        except Exception as e:
            print(f'update_object_hand_pose error: {e}')
