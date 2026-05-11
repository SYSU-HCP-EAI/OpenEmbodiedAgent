import math
from typing import OrderedDict

from internutopia.core.robot.robot import BaseRobot
from internutopia.core.scene.scene import IScene
from internutopia.core.sensor.camera import ICamera
from internutopia.core.sensor.sensor import BaseSensor
from internutopia_extension.configs.sensors import LayoutEditMocapControlledCameraCfg
from internutopia_extension.sensors.mocap_controlled_camera import CameraMover

DISPLACEMENT_THRESHOLD = 0.05
NUM_SPECIFIC_POSE_FRAMES = 60


@BaseSensor.register('LayoutEditMocapControlledCamera')
class LayoutEditMocapControlledCamera(BaseSensor):
    """
    wrap of isaac sim's Camera class
    """

    def __init__(self, config: LayoutEditMocapControlledCameraCfg, robot: BaseRobot, scene: IScene = None):
        super().__init__(config, robot, scene)
        self.config = config

        self.layout_camera_mover = LayoutCameraMover([0, 0, 0])

    def create_camera(self) -> ICamera:
        """Create an isaac-sim camera object.

        Initializes the camera's resolution and prim path based on configuration.

        Returns:
            ICamera: The initialized camera object.
        """
        # Initialize the params for the camera
        resolution = self.config.resolution if self.config.resolution else (320, 240)
        translation = self.config.translation if self.config.translation else None
        orientation = self.config.orientation if self.config.translation else None

        prim_path = '/World' + '/' + self.config.prim_path
        camera = ICamera.create(
            name=self.name,
            prim_path=prim_path,
            rgba=True,
            bounding_box_2d_tight=True,
            resolution=resolution,
            translation=translation,
            orientation=orientation,
        )
        return camera

    def post_reset(self):
        self._camera: ICamera = self.create_camera()

    def get_data(self) -> OrderedDict:
        rgba = self._camera.get_rgba()
        bounding_box_2d_tight = self._camera.get_bounding_box_2d_tight()

        obs = {'rgba': rgba, 'bounding_box_2d_tight': bounding_box_2d_tight}
        return self._make_ordered(obs)

    @property
    def camera(self):
        return self._camera

    def get_pose(self):
        return self._camera.get_pose()


class LayoutCameraMover(CameraMover):
    def __init__(self, target_point, rate=[3, 5, 5]):
        super().__init__(target_point, rate)

    def condition_judgment(self, results):
        lh_bones_kps = results.get('lh_bones_kps', None)
        if lh_bones_kps is None:
            self.is_move = False
            return

        thumb_finger = lh_bones_kps[4]
        index_finger = lh_bones_kps[12]
        distance = math.sqrt(
            (index_finger[0] - thumb_finger[0]) ** 2
            + (index_finger[1] - thumb_finger[1]) ** 2
            + (index_finger[2] - thumb_finger[2]) ** 2
        )

        if distance < DISPLACEMENT_THRESHOLD:
            self.is_move = True
        else:
            self.is_move = False
