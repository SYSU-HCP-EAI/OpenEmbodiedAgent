import numpy as np

from internutopia.core.util.joint import create_joint
from internutopia.core.util.physics import activate_collider, deactivate_collider


class FixjointHand:
    def __init__(self, stage, object_all):
        self.stage = stage
        self.object_all = object_all

    def bind_object(self, temp_object, hand_path):
        if temp_object is None:
            temp_object = self.calculate_nearest_object(hand_path)
        self.bound_object_and_robot(temp_object, hand_path)
        return temp_object

    def unbind_object(self, temp_object, hand_path):
        if temp_object is not None:
            self.unbound_object_and_robot(temp_object, hand_path)
            return None
        return None

    def bound_object_and_robot(self, goal_prim_path, robot_prim_path):
        import omni

        if 'right' in robot_prim_path:
            deactivate_collider(omni.isaac.core.utils.prims.get_prim_at_path(goal_prim_path))
            create_joint(
                prim_path=goal_prim_path + '/pick_joint',  # joint path(will be created)
                joint_type='FixedJoint',
                body0=goal_prim_path,  # object path
                body1=robot_prim_path + '/h1',  # hand path
                enabled=True,
            )
        elif 'left' in robot_prim_path:
            deactivate_collider(omni.isaac.core.utils.prims.get_prim_at_path(goal_prim_path))
            create_joint(
                prim_path=goal_prim_path + '/pick_joint',  # joint path(will be created)
                joint_type='FixedJoint',
                body0=goal_prim_path,  # object path
                body1=robot_prim_path + '/h1',  # hand path
                enabled=True,
            )

    def unbound_object_and_robot(self, goal_prim_path, robot_prim_path):
        import omni

        self.stage.RemovePrim(goal_prim_path + '/pick_joint')
        activate_collider(omni.isaac.core.utils.prims.get_prim_at_path(goal_prim_path))

    def calculate_nearest_object(self, robot):
        """
        Use Euclidean distance to get the object closest to the robot.
        """
        object_position = [self.get_world_position(obj) for obj in self.object_all]
        robot_position = self.get_world_position(robot)
        # Calculate the Euclidean distance between the robot and the objects in world coordinates,
        # and return the object with the smallest distance.
        coordinates = np.array(object_position)
        robot_position = np.array(robot_position)
        distances = np.linalg.norm(coordinates - robot_position, axis=1)
        nearest_index = np.argmin(distances)
        nearest_point = self.object_all[nearest_index]
        return nearest_point

    def get_world_position(self, prim_path):
        """
        Get the world position of a prim given its path.
        """
        from pxr import Usd, UsdGeom

        try:
            prim = self.stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                xformable = UsdGeom.Xformable(prim)
                localToWorldTransform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                world_position = localToWorldTransform.ExtractTranslation()
                return world_position
        except Exception as e:
            print(f'{prim_path} is not valid: {str(e)}')
